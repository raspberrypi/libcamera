/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 *
 * converter.cpp - Format converter for simple pipeline handler
 */

#include "converter.h"

#include <algorithm>
#include <limits.h>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/utils.h>

#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(SimplePipeline)

/* -----------------------------------------------------------------------------
 * SimpleConverter::Stream
 */

SimpleConverter::Stream::Stream(SimpleConverter *converter, unsigned int index)
	: converter_(converter), index_(index)
{
	m2m_ = std::make_unique<V4L2M2MDevice>(converter->deviceNode_);

	m2m_->output()->bufferReady.connect(this, &Stream::outputBufferReady);
	m2m_->capture()->bufferReady.connect(this, &Stream::captureBufferReady);

	int ret = m2m_->open();
	if (ret < 0)
		m2m_.reset();
}

int SimpleConverter::Stream::configure(const StreamConfiguration &inputCfg,
				       const StreamConfiguration &outputCfg)
{
	V4L2PixelFormat videoFormat =
		m2m_->output()->toV4L2PixelFormat(inputCfg.pixelFormat);

	V4L2DeviceFormat format;
	format.fourcc = videoFormat;
	format.size = inputCfg.size;
	format.planesCount = 1;
	format.planes[0].bpl = inputCfg.stride;

	int ret = m2m_->output()->setFormat(&format);
	if (ret < 0) {
		LOG(SimplePipeline, Error)
			<< "Failed to set input format: " << strerror(-ret);
		return ret;
	}

	if (format.fourcc != videoFormat || format.size != inputCfg.size ||
	    format.planes[0].bpl != inputCfg.stride) {
		LOG(SimplePipeline, Error)
			<< "Input format not supported (requested "
			<< inputCfg.size << "-" << videoFormat
			<< ", got " << format << ")";
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

	inputBufferCount_ = inputCfg.bufferCount;
	outputBufferCount_ = outputCfg.bufferCount;

	return 0;
}

int SimpleConverter::Stream::exportBuffers(unsigned int count,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return m2m_->capture()->exportBuffers(count, buffers);
}

int SimpleConverter::Stream::start()
{
	int ret = m2m_->output()->importBuffers(inputBufferCount_);
	if (ret < 0)
		return ret;

	ret = m2m_->capture()->importBuffers(outputBufferCount_);
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

void SimpleConverter::Stream::stop()
{
	m2m_->capture()->streamOff();
	m2m_->output()->streamOff();
	m2m_->capture()->releaseBuffers();
	m2m_->output()->releaseBuffers();
}

int SimpleConverter::Stream::queueBuffers(FrameBuffer *input,
					  FrameBuffer *output)
{
	int ret = m2m_->output()->queueBuffer(input);
	if (ret < 0)
		return ret;

	ret = m2m_->capture()->queueBuffer(output);
	if (ret < 0)
		return ret;

	return 0;
}

std::string SimpleConverter::Stream::logPrefix() const
{
	return "stream" + std::to_string(index_);
}

void SimpleConverter::Stream::outputBufferReady(FrameBuffer *buffer)
{
	auto it = converter_->queue_.find(buffer);
	if (it == converter_->queue_.end())
		return;

	if (!--it->second) {
		converter_->inputBufferReady.emit(buffer);
		converter_->queue_.erase(it);
	}
}

void SimpleConverter::Stream::captureBufferReady(FrameBuffer *buffer)
{
	converter_->outputBufferReady.emit(buffer);
}

/* -----------------------------------------------------------------------------
 * SimpleConverter
 */

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

	deviceNode_ = (*it)->deviceNode();

	m2m_ = std::make_unique<V4L2M2MDevice>(deviceNode_);
	int ret = m2m_->open();
	if (ret < 0) {
		m2m_.reset();
		return;
	}
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

	if (v4l2Format.fourcc != m2m_->output()->toV4L2PixelFormat(input)) {
		LOG(SimplePipeline, Debug)
			<< "Input format " << input << " not supported.";
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

int SimpleConverter::configure(const StreamConfiguration &inputCfg,
			       const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs)
{
	int ret = 0;

	streams_.clear();
	streams_.reserve(outputCfgs.size());

	for (unsigned int i = 0; i < outputCfgs.size(); ++i) {
		Stream &stream = streams_.emplace_back(this, i);

		if (!stream.isValid()) {
			LOG(SimplePipeline, Error)
				<< "Failed to create stream " << i;
			ret = -EINVAL;
			break;
		}

		ret = stream.configure(inputCfg, outputCfgs[i]);
		if (ret < 0)
			break;
	}

	if (ret < 0) {
		streams_.clear();
		return ret;
	}

	return 0;
}

int SimpleConverter::exportBuffers(unsigned int output, unsigned int count,
				   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	if (output >= streams_.size())
		return -EINVAL;

	return streams_[output].exportBuffers(count, buffers);
}

int SimpleConverter::start()
{
	int ret;

	for (Stream &stream : streams_) {
		ret = stream.start();
		if (ret < 0) {
			stop();
			return ret;
		}
	}

	return 0;
}

void SimpleConverter::stop()
{
	for (Stream &stream : utils::reverse(streams_))
		stream.stop();
}

int SimpleConverter::queueBuffers(FrameBuffer *input,
				  const std::map<unsigned int, FrameBuffer *> &outputs)
{
	unsigned int mask = 0;
	int ret;

	/*
	 * Validate the outputs as a sanity check: at least one output is
	 * required, all outputs must reference a valid stream and no two
	 * outputs can reference the same stream.
	 */
	if (outputs.empty())
		return -EINVAL;

	for (auto [index, buffer] : outputs) {
		if (!buffer)
			return -EINVAL;
		if (index >= streams_.size())
			return -EINVAL;
		if (mask & (1 << index))
			return -EINVAL;

		mask |= 1 << index;
	}

	/* Queue the input and output buffers to all the streams. */
	for (auto [index, buffer] : outputs) {
		ret = streams_[index].queueBuffers(input, buffer);
		if (ret < 0)
			return ret;
	}

	/*
	 * Add the input buffer to the queue, with the number of streams as a
	 * reference count. Completion of the input buffer will be signalled by
	 * the stream that releases the last reference.
	 */
	queue_.emplace(std::piecewise_construct,
		       std::forward_as_tuple(input),
		       std::forward_as_tuple(outputs.size()));

	return 0;
}

} /* namespace libcamera */
