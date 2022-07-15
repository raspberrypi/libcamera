/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * cio2.cpp - Intel IPU3 CIO2
 */

#include "cio2.h"

#include <limits>
#include <math.h>

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/framebuffer.h"
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
{
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
 * \param[in] format The pixel format
 *
 * Retrieve the list of supported sizes for a particular \a format by matching
 * the sensor produced media bus codes formats supported by the CIO2 unit.
 *
 * \return A list of supported sizes for the \a format or an empty list
 * otherwise
 */
std::vector<SizeRange> CIO2Device::sizes(const PixelFormat &format) const
{
	int mbusCode = -1;

	if (!sensor_)
		return {};

	std::vector<SizeRange> sizes;
	for (const auto &iter : mbusCodesToPixelFormat) {
		if (iter.second != format)
			continue;

		mbusCode = iter.first;
		break;
	}

	if (mbusCode == -1)
		return {};

	for (const Size &sz : sensor_->sizes(mbusCode))
		sizes.emplace_back(sz);

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
	sensor_ = std::make_unique<CameraSensor>(sensorEntity);
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

	csi2_ = std::make_unique<V4L2Subdevice>(csi2Entity);
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
	sensorFormat = getSensorFormat(mbusCodes, size);
	ret = sensor_->setFormat(&sensorFormat);
	if (ret)
		return ret;

	ret = csi2_->setFormat(0, &sensorFormat);
	if (ret)
		return ret;

	const auto &itInfo = mbusCodesToPixelFormat.find(sensorFormat.mbus_code);
	if (itInfo == mbusCodesToPixelFormat.end())
		return -EINVAL;

	outputFormat->fourcc = output_->toV4L2PixelFormat(itInfo->second);
	outputFormat->size = sensorFormat.size;
	outputFormat->planesCount = 1;

	ret = output_->setFormat(outputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "CIO2 output format " << *outputFormat;

	return 0;
}

StreamConfiguration CIO2Device::generateConfiguration(Size size) const
{
	StreamConfiguration cfg;

	/* If no desired size use the sensor resolution. */
	if (size.isNull())
		size = sensor_->resolution();

	/* Query the sensor static information for closest match. */
	std::vector<unsigned int> mbusCodes = utils::map_keys(mbusCodesToPixelFormat);
	V4L2SubdeviceFormat sensorFormat = getSensorFormat(mbusCodes, size);
	if (!sensorFormat.mbus_code) {
		LOG(IPU3, Error) << "Sensor does not support mbus code";
		return {};
	}

	cfg.size = sensorFormat.size;
	cfg.pixelFormat = mbusCodesToPixelFormat.at(sensorFormat.mbus_code);
	cfg.bufferCount = kBufferCount;

	return cfg;
}

/**
 * \brief Retrieve the best sensor format for a desired output
 * \param[in] mbusCodes The list of acceptable media bus codes
 * \param[in] size The desired size
 *
 * Media bus codes are selected from \a mbusCodes, which lists all acceptable
 * codes in decreasing order of preference. Media bus codes supported by the
 * sensor but not listed in \a mbusCodes are ignored. If none of the desired
 * codes is supported, it returns an error.
 *
 * \a size indicates the desired size at the output of the sensor. This method
 * selects the best media bus code and size supported by the sensor according
 * to the following criteria.
 *
 * - The desired \a size shall fit in the sensor output size to avoid the need
 *   to up-scale.
 * - The aspect ratio of sensor output size shall be as close as possible to
 *   the sensor's native resolution field of view.
 * - The sensor output size shall be as small as possible to lower the required
 *   bandwidth.
 * - The desired \a size shall be supported by one of the media bus code listed
 *   in \a mbusCodes.
 *
 * When multiple media bus codes can produce the same size, the code at the
 * lowest position in \a mbusCodes is selected.
 *
 * The returned sensor output format is guaranteed to be acceptable by the
 * setFormat() method without any modification.
 *
 * \return The best sensor output format matching the desired media bus codes
 * and size on success, or an empty format otherwise.
 */
V4L2SubdeviceFormat CIO2Device::getSensorFormat(const std::vector<unsigned int> &mbusCodes,
						const Size &size) const
{
	unsigned int desiredArea = size.width * size.height;
	unsigned int bestArea = std::numeric_limits<unsigned int>::max();
	const Size &resolution = sensor_->resolution();
	float desiredRatio = static_cast<float>(resolution.width) /
			     resolution.height;
	float bestRatio = std::numeric_limits<float>::max();
	Size bestSize;
	uint32_t bestCode = 0;

	for (unsigned int code : mbusCodes) {
		const auto sizes = sensor_->sizes(code);
		if (!sizes.size())
			continue;

		for (const Size &sz : sizes) {
			if (sz.width < size.width || sz.height < size.height)
				continue;

			float ratio = static_cast<float>(sz.width) / sz.height;
			/*
			 * Ratios can differ by small mantissa difference which
			 * can affect the selection of the sensor output size
			 * wildly. We are interested in selection of the closest
			 * size with respect to the desired output size, hence
			 * comparing it with a single precision digit is enough.
			 */
			ratio = static_cast<unsigned int>(ratio * 10) / 10.0;
			float ratioDiff = fabsf(ratio - desiredRatio);
			unsigned int area = sz.width * sz.height;
			unsigned int areaDiff = area - desiredArea;

			if (ratioDiff > bestRatio)
				continue;

			if (ratioDiff < bestRatio || areaDiff < bestArea) {
				bestRatio = ratioDiff;
				bestArea = areaDiff;
				bestSize = sz;
				bestCode = code;
			}
		}
	}

	if (bestSize.isNull()) {
		LOG(IPU3, Debug) << "No supported format or size found";
		return {};
	}

	V4L2SubdeviceFormat format{};
	format.mbus_code = bestCode;
	format.size = bestSize;

	return format;
}

int CIO2Device::exportBuffers(unsigned int count,
			      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return output_->exportBuffers(count, buffers);
}

int CIO2Device::start()
{
	int ret = output_->exportBuffers(kBufferCount, &buffers_);
	if (ret < 0)
		return ret;

	ret = output_->importBuffers(kBufferCount);
	if (ret)
		LOG(IPU3, Error) << "Failed to import CIO2 buffers";

	for (std::unique_ptr<FrameBuffer> &buffer : buffers_)
		availableBuffers_.push(buffer.get());

	ret = output_->streamOn();
	if (ret) {
		freeBuffers();
		return ret;
	}

	ret = csi2_->setFrameStartEnabled(true);
	if (ret) {
		stop();
		return ret;
	}

	return 0;
}

int CIO2Device::stop()
{
	int ret;

	csi2_->setFrameStartEnabled(false);

	ret = output_->streamOff();

	freeBuffers();

	return ret;
}

FrameBuffer *CIO2Device::queueBuffer(Request *request, FrameBuffer *rawBuffer)
{
	FrameBuffer *buffer = rawBuffer;

	/* If no buffer is provided in the request, use an internal one. */
	if (!buffer) {
		if (availableBuffers_.empty()) {
			LOG(IPU3, Debug) << "CIO2 buffer underrun";
			return nullptr;
		}

		buffer = availableBuffers_.front();
		availableBuffers_.pop();
		buffer->_d()->setRequest(request);
	}

	int ret = output_->queueBuffer(buffer);
	if (ret)
		return nullptr;

	return buffer;
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

	bufferAvailable.emit();
}

void CIO2Device::freeBuffers()
{
	availableBuffers_ = {};
	buffers_.clear();

	if (output_->releaseBuffers())
		LOG(IPU3, Error) << "Failed to release CIO2 buffers";
}

} /* namespace libcamera */
