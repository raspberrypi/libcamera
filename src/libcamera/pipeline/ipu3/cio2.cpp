/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * cio2.cpp - Intel IPU3 CIO2
 */

#include "cio2.h"

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPU3)

namespace {

const std::map<uint32_t, PixelFormat> mbusCodesToPixelFormat = {
	{ MEDIA_BUS_FMT_SBGGR10_1X10, formats::SBGGR10_IPU3 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, formats::SGBRG10_IPU3 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, formats::SGRBG10_IPU3 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, formats::SRGGB10_IPU3 },
};

} /* namespace */

CIO2Device::CIO2Device()
	: sensor_(nullptr), csi2_(nullptr), output_(nullptr)
{
}

CIO2Device::~CIO2Device()
{
	delete output_;
	delete csi2_;
	delete sensor_;
}

/**
 * \brief Retrieve the list of supported PixelFormats
 *
 * Retrieve the list of supported pixel formats by matching the sensor produced
 * media bus codes with the formats supported by the CIO2 unit.
 *
 * \return The list of supported PixelFormat
 */
std::vector<PixelFormat> CIO2Device::formats() const
{
	if (!sensor_)
		return {};

	std::vector<PixelFormat> formats;
	for (unsigned int code : sensor_->mbusCodes()) {
		auto it = mbusCodesToPixelFormat.find(code);
		if (it != mbusCodesToPixelFormat.end())
			formats.push_back(it->second);
	}

	return formats;
}

/**
 * \brief Retrieve the list of supported size ranges
 * \return The list of supported SizeRange
 */
std::vector<SizeRange> CIO2Device::sizes() const
{
	if (!sensor_)
		return {};

	std::vector<SizeRange> sizes;
	for (const Size &size : sensor_->sizes())
		sizes.emplace_back(size, size);

	return sizes;
}

/**
 * \brief Initialize components of the CIO2 device with \a index
 * \param[in] media The CIO2 media device
 * \param[in] index The CIO2 device index
 *
 * Create and open the video device and subdevices in the CIO2 instance at \a
 * index, if a supported image sensor is connected to the CSI-2 receiver of
 * this CIO2 instance.  Enable the media links connecting the CIO2 components
 * to prepare for capture operations and cached the sensor maximum size.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV No supported image sensor is connected to this CIO2 instance
 */
int CIO2Device::init(const MediaDevice *media, unsigned int index)
{
	int ret;

	/*
	 * Verify that a sensor subdevice is connected to this CIO2 instance
	 * and enable the media link between the two.
	 */
	std::string csi2Name = "ipu3-csi2 " + std::to_string(index);
	MediaEntity *csi2Entity = media->getEntityByName(csi2Name);
	const std::vector<MediaPad *> &pads = csi2Entity->pads();
	if (pads.empty())
		return -ENODEV;

	/* IPU3 CSI-2 receivers have a single sink pad at index 0. */
	MediaPad *sink = pads[0];
	const std::vector<MediaLink *> &links = sink->links();
	if (links.empty())
		return -ENODEV;

	MediaLink *link = links[0];
	MediaEntity *sensorEntity = link->source()->entity();
	sensor_ = new CameraSensor(sensorEntity);
	ret = sensor_->init();
	if (ret)
		return ret;

	ret = link->setEnabled(true);
	if (ret)
		return ret;

	/*
	 * Make sure the sensor produces at least one format compatible with
	 * the CIO2 requirements.
	 *
	 * utils::set_overlap requires the ranges to be sorted, keep the
	 * cio2Codes vector sorted in ascending order.
	 */
	std::vector<unsigned int> cio2Codes = utils::map_keys(mbusCodesToPixelFormat);
	const std::vector<unsigned int> &sensorCodes = sensor_->mbusCodes();
	if (!utils::set_overlap(sensorCodes.begin(), sensorCodes.end(),
				cio2Codes.begin(), cio2Codes.end())) {
		LOG(IPU3, Error)
			<< "Sensor " << sensor_->entity()->name()
			<< " has not format compatible with the IPU3";
		return -EINVAL;
	}

	/*
	 * \todo Define when to open and close video device nodes, as they
	 * might impact on power consumption.
	 */

	csi2_ = new V4L2Subdevice(csi2Entity);
	ret = csi2_->open();
	if (ret)
		return ret;

	std::string cio2Name = "ipu3-cio2 " + std::to_string(index);
	output_ = V4L2VideoDevice::fromEntityName(media, cio2Name);
	return output_->open();
}

/**
 * \brief Configure the CIO2 unit
 * \param[in] size The requested CIO2 output frame size
 * \param[out] outputFormat The CIO2 unit output image format
 * \return 0 on success or a negative error code otherwise
 */
int CIO2Device::configure(const Size &size, V4L2DeviceFormat *outputFormat)
{
	V4L2SubdeviceFormat sensorFormat;
	int ret;

	/*
	 * Apply the selected format to the sensor, the CSI-2 receiver and
	 * the CIO2 output device.
	 */
	std::vector<unsigned int> mbusCodes = utils::map_keys(mbusCodesToPixelFormat);
	sensorFormat = sensor_->getFormat(mbusCodes, size);
	ret = sensor_->setFormat(&sensorFormat);
	if (ret)
		return ret;

	ret = csi2_->setFormat(0, &sensorFormat);
	if (ret)
		return ret;

	const auto &itInfo = mbusCodesToPixelFormat.find(sensorFormat.mbus_code);
	if (itInfo == mbusCodesToPixelFormat.end())
		return -EINVAL;

	const PixelFormatInfo &info = PixelFormatInfo::info(itInfo->second);

	outputFormat->fourcc = info.v4l2Format;
	outputFormat->size = sensorFormat.size;
	outputFormat->planesCount = 1;

	ret = output_->setFormat(outputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "CIO2 output format " << outputFormat->toString();

	return 0;
}

StreamConfiguration
CIO2Device::generateConfiguration(Size size) const
{
	StreamConfiguration cfg;

	/* If no desired size use the sensor resolution. */
	if (size.isNull())
		size = sensor_->resolution();

	/* Query the sensor static information for closest match. */
	std::vector<unsigned int> mbusCodes = utils::map_keys(mbusCodesToPixelFormat);
	V4L2SubdeviceFormat sensorFormat = sensor_->getFormat(mbusCodes, size);
	if (!sensorFormat.mbus_code) {
		LOG(IPU3, Error) << "Sensor does not support mbus code";
		return {};
	}

	cfg.size = sensorFormat.size;
	cfg.pixelFormat = mbusCodesToPixelFormat.at(sensorFormat.mbus_code);
	cfg.bufferCount = CIO2_BUFFER_COUNT;

	return cfg;
}

int CIO2Device::exportBuffers(unsigned int count,
			      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return output_->exportBuffers(count, buffers);
}

int CIO2Device::start()
{
	int ret = output_->exportBuffers(CIO2_BUFFER_COUNT, &buffers_);
	if (ret < 0)
		return ret;

	ret = output_->importBuffers(CIO2_BUFFER_COUNT);
	if (ret)
		LOG(IPU3, Error) << "Failed to import CIO2 buffers";

	for (std::unique_ptr<FrameBuffer> &buffer : buffers_)
		availableBuffers_.push(buffer.get());

	ret = output_->streamOn();
	if (ret)
		freeBuffers();

	return ret;
}

int CIO2Device::stop()
{
	int ret = output_->streamOff();

	freeBuffers();

	return ret;
}

int CIO2Device::queueBuffer(Request *request, FrameBuffer *rawBuffer)
{
	FrameBuffer *buffer = rawBuffer;

	/* If no buffer is provided in the request, use an internal one. */
	if (!buffer) {
		if (availableBuffers_.empty()) {
			LOG(IPU3, Error) << "CIO2 buffer underrun";
			return -EINVAL;
		}

		buffer = availableBuffers_.front();
		availableBuffers_.pop();
	}

	buffer->setRequest(request);

	return output_->queueBuffer(buffer);
}

void CIO2Device::tryReturnBuffer(FrameBuffer *buffer)
{
	/*
	 * \todo Once more pipelines deal with buffers that may be allocated
	 * internally or externally this pattern might become a common need. At
	 * that point this check should be moved to something clever in
	 * FrameBuffer.
	 */
	for (const std::unique_ptr<FrameBuffer> &buf : buffers_) {
		if (buf.get() == buffer) {
			availableBuffers_.push(buffer);
			break;
		}
	}
}

void CIO2Device::freeBuffers()
{
	/* The default std::queue constructor is explicit with gcc 5 and 6. */
	availableBuffers_ = std::queue<FrameBuffer *>{};

	buffers_.clear();

	if (output_->releaseBuffers())
		LOG(IPU3, Error) << "Failed to release CIO2 buffers";
}

} /* namespace libcamera */
