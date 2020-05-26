/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * cio2.cpp - Intel IPU3 CIO2
 */

#include "cio2.h"

#include <linux/media-bus-format.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPU3)

CIO2Device::CIO2Device()
	: output_(nullptr), csi2_(nullptr), sensor_(nullptr)
{
}

CIO2Device::~CIO2Device()
{
	delete output_;
	delete csi2_;
	delete sensor_;
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
	const std::vector<unsigned int> cio2Codes{ MEDIA_BUS_FMT_SBGGR10_1X10,
						   MEDIA_BUS_FMT_SGRBG10_1X10,
						   MEDIA_BUS_FMT_SGBRG10_1X10,
						   MEDIA_BUS_FMT_SRGGB10_1X10 };
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
	ret = output_->open();
	if (ret)
		return ret;

	return 0;
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
	sensorFormat = sensor_->getFormat({ MEDIA_BUS_FMT_SBGGR10_1X10,
					    MEDIA_BUS_FMT_SGBRG10_1X10,
					    MEDIA_BUS_FMT_SGRBG10_1X10,
					    MEDIA_BUS_FMT_SRGGB10_1X10 },
					  size);
	ret = sensor_->setFormat(&sensorFormat);
	if (ret)
		return ret;

	ret = csi2_->setFormat(0, &sensorFormat);
	if (ret)
		return ret;

	V4L2PixelFormat v4l2Format;
	switch (sensorFormat.mbus_code) {
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SBGGR10);
		break;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SGBRG10);
		break;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SGRBG10);
		break;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SRGGB10);
		break;
	default:
		return -EINVAL;
	}

	outputFormat->fourcc = v4l2Format;
	outputFormat->size = sensorFormat.size;
	outputFormat->planesCount = 1;

	ret = output_->setFormat(outputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "CIO2 output format " << outputFormat->toString();

	return 0;
}

/**
 * \brief Allocate frame buffers for the CIO2 output
 *
 * Allocate frame buffers in the CIO2 video device to be used to capture frames
 * from the CIO2 output. The buffers are stored in the CIO2Device::buffers_
 * vector.
 *
 * \return Number of buffers allocated or negative error code
 */
int CIO2Device::allocateBuffers()
{
	int ret = output_->allocateBuffers(CIO2_BUFFER_COUNT, &buffers_);
	if (ret < 0)
		return ret;

	for (std::unique_ptr<FrameBuffer> &buffer : buffers_)
		availableBuffers_.push(buffer.get());

	return ret;
}

void CIO2Device::freeBuffers()
{
	/* The default std::queue constructor is explicit with gcc 5 and 6. */
	availableBuffers_ = std::queue<FrameBuffer *>{};

	buffers_.clear();

	if (output_->releaseBuffers())
		LOG(IPU3, Error) << "Failed to release CIO2 buffers";
}

FrameBuffer *CIO2Device::getBuffer()
{
	if (availableBuffers_.empty()) {
		LOG(IPU3, Error) << "CIO2 buffer underrun";
		return nullptr;
	}

	FrameBuffer *buffer = availableBuffers_.front();

	availableBuffers_.pop();

	return buffer;
}

void CIO2Device::putBuffer(FrameBuffer *buffer)
{
	availableBuffers_.push(buffer);
}

int CIO2Device::start()
{
	return output_->streamOn();
}

int CIO2Device::stop()
{
	return output_->streamOff();
}

} /* namespace libcamera */
