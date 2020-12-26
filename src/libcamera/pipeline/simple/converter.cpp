/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 *
 * converter.cpp - Format converter for simple pipeline handler
 */

#include "converter.h"

#include <algorithm>
#include <limits.h>

#include <libcamera/buffer.h>
#include <libcamera/geometry.h>
#include <libcamera/signal.h>
#include <libcamera/stream.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(SimplePipeline)

SimpleConverter::SimpleConverter(MediaDevice *media)
{
	/*
	 * Locate the video node. There's no need to validate the pipeline
	 * further, the caller guarantees that this is a V4L2 mem2mem device.
	 */
	const std::vector<MediaEntity *> &entities = media->entities();
	auto it = std::find_if(entities.begin(), entities.end(),
			       [](MediaEntity *entity) {
				       return entity->function() == MEDIA_ENT_F_IO_V4L;
			       });
	if (it == entities.end())
		return;

	m2m_ = std::make_unique<V4L2M2MDevice>((*it)->deviceNode());

	m2m_->output()->bufferReady.connect(this, &SimpleConverter::outputBufferReady);
	m2m_->capture()->bufferReady.connect(this, &SimpleConverter::captureBufferReady);
}

int SimpleConverter::open()
{
	if (!m2m_)
		return -ENODEV;

	return m2m_->open();
}

void SimpleConverter::close()
{
	if (m2m_)
		m2m_->close();
}

std::vector<PixelFormat> SimpleConverter::formats(PixelFormat input)
{
	if (!m2m_)
		return {};

	/*
	 * Set the format on the input side (V4L2 output) of the converter to
	 * enumerate the conversion capabilities on its output (V4L2 capture).
	 */
	V4L2DeviceFormat v4l2Format;
	v4l2Format.fourcc = m2m_->output()->toV4L2PixelFormat(input);
	v4l2Format.size = { 1, 1 };

	int ret = m2m_->output()->setFormat(&v4l2Format);
	if (ret < 0) {
		LOG(SimplePipeline, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	std::vector<PixelFormat> pixelFormats;

	for (const auto &format : m2m_->capture()->formats()) {
		PixelFormat pixelFormat = format.first.toPixelFormat();
		if (pixelFormat)
			pixelFormats.push_back(pixelFormat);
	}

	return pixelFormats;
}

SizeRange SimpleConverter::sizes(const Size &input)
{
	if (!m2m_)
		return {};

	/*
	 * Set the size on the input side (V4L2 output) of the converter to
	 * enumerate the scaling capabilities on its output (V4L2 capture).
	 */
	V4L2DeviceFormat format;
	format.fourcc = V4L2PixelFormat();
	format.size = input;

	int ret = m2m_->output()->setFormat(&format);
	if (ret < 0) {
		LOG(SimplePipeline, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	SizeRange sizes;

	format.size = { 1, 1 };
	ret = m2m_->capture()->setFormat(&format);
	if (ret < 0) {
		LOG(SimplePipeline, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	sizes.min = format.size;

	format.size = { UINT_MAX, UINT_MAX };
	ret = m2m_->capture()->setFormat(&format);
	if (ret < 0) {
		LOG(SimplePipeline, Error)
			<< "Failed to set format: " << strerror(-ret);
		return {};
	}

	sizes.max = format.size;

	return sizes;
}

std::tuple<unsigned int, unsigned int>
SimpleConverter::strideAndFrameSize(const PixelFormat &pixelFormat,
				    const Size &size)
{
	V4L2DeviceFormat format;
	format.fourcc = m2m_->capture()->toV4L2PixelFormat(pixelFormat);
	format.size = size;

	int ret = m2m_->capture()->tryFormat(&format);
	if (ret < 0)
		return std::make_tuple(0, 0);

	return std::make_tuple(format.planes[0].bpl, format.planes[0].size);
}

int SimpleConverter::configure(PixelFormat inputFormat, const Size &inputSize,
			       const StreamConfiguration &outputCfg)
{
	V4L2DeviceFormat format;
	int ret;

	V4L2PixelFormat videoFormat = m2m_->output()->toV4L2PixelFormat(inputFormat);
	format.fourcc = videoFormat;
	format.size = inputSize;

	ret = m2m_->output()->setFormat(&format);
	if (ret < 0) {
		LOG(SimplePipeline, Error)
			<< "Failed to set input format: " << strerror(-ret);
		return ret;
	}

	if (format.fourcc != videoFormat || format.size != inputSize) {
		LOG(SimplePipeline, Error)
			<< "Input format not supported";
		return -EINVAL;
	}

	/* Set the pixel format and size on the output. */
	videoFormat = m2m_->capture()->toV4L2PixelFormat(outputCfg.pixelFormat);
	format = {};
	format.fourcc = videoFormat;
	format.size = outputCfg.size;

	ret = m2m_->capture()->setFormat(&format);
	if (ret < 0) {
		LOG(SimplePipeline, Error)
			<< "Failed to set output format: " << strerror(-ret);
		return ret;
	}

	if (format.fourcc != videoFormat || format.size != outputCfg.size) {
		LOG(SimplePipeline, Error)
			<< "Output format not supported";
		return -EINVAL;
	}

	return 0;
}

int SimpleConverter::exportBuffers(unsigned int count,
				   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return m2m_->capture()->exportBuffers(count, buffers);
}

int SimpleConverter::start(unsigned int count)
{
	int ret = m2m_->output()->importBuffers(count);
	if (ret < 0)
		return ret;

	ret = m2m_->capture()->importBuffers(count);
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = m2m_->output()->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = m2m_->capture()->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	return 0;
}

void SimpleConverter::stop()
{
	m2m_->capture()->streamOff();
	m2m_->output()->streamOff();
	m2m_->capture()->releaseBuffers();
	m2m_->output()->releaseBuffers();
}

int SimpleConverter::queueBuffers(FrameBuffer *input, FrameBuffer *output)
{
	int ret = m2m_->output()->queueBuffer(input);
	if (ret < 0)
		return ret;

	ret = m2m_->capture()->queueBuffer(output);
	if (ret < 0)
		return ret;

	return 0;
}

void SimpleConverter::captureBufferReady(FrameBuffer *buffer)
{
	if (!outputDoneQueue_.empty()) {
		FrameBuffer *other = outputDoneQueue_.front();
		outputDoneQueue_.pop();
		bufferReady.emit(other, buffer);
	} else {
		captureDoneQueue_.push(buffer);
	}
}

void SimpleConverter::outputBufferReady(FrameBuffer *buffer)
{
	if (!captureDoneQueue_.empty()) {
		FrameBuffer *other = captureDoneQueue_.front();
		captureDoneQueue_.pop();
		bufferReady.emit(buffer, other);
	} else {
		outputDoneQueue_.push(buffer);
	}
}

} /* namespace libcamera */
