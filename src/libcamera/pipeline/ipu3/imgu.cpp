/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * imgu.cpp - Intel IPU3 ImgU
 */

#include "imgu.h"

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPU3)

/**
 * \brief Initialize components of the ImgU instance
 * \param[in] mediaDevice The ImgU instance media device
 * \param[in] index The ImgU instance index
 *
 * Create and open the V4L2 devices and subdevices of the ImgU instance
 * with \a index.
 *
 * In case of errors the created V4L2VideoDevice and V4L2Subdevice instances
 * are destroyed at pipeline handler delete time.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::init(MediaDevice *media, unsigned int index)
{
	int ret;

	name_ = "ipu3-imgu " + std::to_string(index);
	media_ = media;

	/*
	 * The media entities presence in the media device has been verified
	 * by the match() function: no need to check for newly created
	 * video devices and subdevice validity here.
	 */
	imgu_ = V4L2Subdevice::fromEntityName(media, name_);
	ret = imgu_->open();
	if (ret)
		return ret;

	input_ = V4L2VideoDevice::fromEntityName(media, name_ + " input");
	ret = input_->open();
	if (ret)
		return ret;

	output_.dev = V4L2VideoDevice::fromEntityName(media, name_ + " output");
	ret = output_.dev->open();
	if (ret)
		return ret;

	output_.pad = PAD_OUTPUT;
	output_.name = "output";

	viewfinder_.dev = V4L2VideoDevice::fromEntityName(media,
							  name_ + " viewfinder");
	ret = viewfinder_.dev->open();
	if (ret)
		return ret;

	viewfinder_.pad = PAD_VF;
	viewfinder_.name = "viewfinder";

	stat_.dev = V4L2VideoDevice::fromEntityName(media, name_ + " 3a stat");
	ret = stat_.dev->open();
	if (ret)
		return ret;

	stat_.pad = PAD_STAT;
	stat_.name = "stat";

	return 0;
}

/**
 * \brief Configure the ImgU unit input
 * \param[in] size The ImgU input frame size
 * \param[in] inputFormat The format to be applied to ImgU input
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configureInput(const Size &size,
			       V4L2DeviceFormat *inputFormat)
{
	/* Configure the ImgU input video device with the requested sizes. */
	int ret = input_->setFormat(inputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU input format = " << inputFormat->toString();

	/*
	 * \todo The IPU3 driver implementation shall be changed to use the
	 * input sizes as 'ImgU Input' subdevice sizes, and use the desired
	 * GDC output sizes to configure the crop/compose rectangles.
	 *
	 * The current IPU3 driver implementation uses GDC sizes as the
	 * 'ImgU Input' subdevice sizes, and the input video device sizes
	 * to configure the crop/compose rectangles, contradicting the
	 * V4L2 specification.
	 */
	Rectangle rect = {
		.x = 0,
		.y = 0,
		.width = inputFormat->size.width,
		.height = inputFormat->size.height,
	};
	ret = imgu_->setSelection(PAD_INPUT, V4L2_SEL_TGT_CROP, &rect);
	if (ret)
		return ret;

	ret = imgu_->setSelection(PAD_INPUT, V4L2_SEL_TGT_COMPOSE, &rect);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU input feeder and BDS rectangle = "
			 << rect.toString();

	V4L2SubdeviceFormat imguFormat = {};
	imguFormat.mbus_code = MEDIA_BUS_FMT_FIXED;
	imguFormat.size = size;

	ret = imgu_->setFormat(PAD_INPUT, &imguFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU GDC format = " << imguFormat.toString();

	return 0;
}

/**
 * \brief Configure a video device on the ImgU
 * \param[in] dev The video device to configure
 * \param[in] pad The pad of the ImgU subdevice
 * \param[in] cfg The requested configuration
 * \param[out] outputFormat The format set on the video device
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configureVideoDevice(V4L2VideoDevice *dev, unsigned int pad,
				     const StreamConfiguration &cfg,
				     V4L2DeviceFormat *outputFormat)
{
	V4L2SubdeviceFormat imguFormat = {};
	imguFormat.mbus_code = MEDIA_BUS_FMT_FIXED;
	imguFormat.size = cfg.size;

	int ret = imgu_->setFormat(pad, &imguFormat);
	if (ret)
		return ret;

	/* No need to apply format to the stat node. */
	if (dev == stat_.dev)
		return 0;

	*outputFormat = {};
	outputFormat->fourcc = dev->toV4L2PixelFormat(formats::NV12);
	outputFormat->size = cfg.size;
	outputFormat->planesCount = 2;

	ret = dev->setFormat(outputFormat);
	if (ret)
		return ret;

	const char *name = dev == output_.dev ? "output" : "viewfinder";
	LOG(IPU3, Debug) << "ImgU " << name << " format = "
			 << outputFormat->toString();

	return 0;
}

/**
 * \brief Allocate buffers for all the ImgU video devices
 */
int ImgUDevice::allocateBuffers(unsigned int bufferCount)
{
	/* Share buffers between CIO2 output and ImgU input. */
	int ret = input_->importBuffers(bufferCount);
	if (ret) {
		LOG(IPU3, Error) << "Failed to import ImgU input buffers";
		return ret;
	}

	/*
	 * The kernel fails to start if buffers are not either imported or
	 * allocated for the statistics video device. As statistics buffers are
	 * not yet used by the pipeline import buffers to save memory.
	 *
	 * \todo To be revised when we'll actually use the stat node.
	 */
	ret = stat_.dev->importBuffers(bufferCount);
	if (ret < 0) {
		LOG(IPU3, Error) << "Failed to allocate ImgU stat buffers";
		goto error;
	}

	/*
	 * Import buffers for all outputs, regardless of whether the
	 * corresponding stream is active or inactive, as the driver needs
	 * buffers to be requested on the V4L2 devices in order to operate.
	 */
	ret = output_.dev->importBuffers(bufferCount);
	if (ret < 0) {
		LOG(IPU3, Error) << "Failed to import ImgU output buffers";
		goto error;
	}

	ret = viewfinder_.dev->importBuffers(bufferCount);
	if (ret < 0) {
		LOG(IPU3, Error) << "Failed to import ImgU viewfinder buffers";
		goto error;
	}

	return 0;

error:
	freeBuffers();

	return ret;
}

/**
 * \brief Release buffers for all the ImgU video devices
 */
void ImgUDevice::freeBuffers()
{
	int ret;

	ret = output_.dev->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU output buffers";

	ret = stat_.dev->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU stat buffers";

	ret = viewfinder_.dev->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU viewfinder buffers";

	ret = input_->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU input buffers";
}

int ImgUDevice::start()
{
	int ret;

	/* Start the ImgU video devices. */
	ret = output_.dev->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU output";
		return ret;
	}

	ret = viewfinder_.dev->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU viewfinder";
		return ret;
	}

	ret = stat_.dev->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU stat";
		return ret;
	}

	ret = input_->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU input";
		return ret;
	}

	return 0;
}

int ImgUDevice::stop()
{
	int ret;

	ret = output_.dev->streamOff();
	ret |= viewfinder_.dev->streamOff();
	ret |= stat_.dev->streamOff();
	ret |= input_->streamOff();

	return ret;
}

/**
 * \brief Enable or disable a single link on the ImgU instance
 *
 * This method assumes the media device associated with the ImgU instance
 * is open.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::linkSetup(const std::string &source, unsigned int sourcePad,
			  const std::string &sink, unsigned int sinkPad,
			  bool enable)
{
	MediaLink *link = media_->link(source, sourcePad, sink, sinkPad);
	if (!link) {
		LOG(IPU3, Error)
			<< "Failed to get link: '" << source << "':"
			<< sourcePad << " -> '" << sink << "':" << sinkPad;
		return -ENODEV;
	}

	return link->setEnabled(enable);
}

/**
 * \brief Enable or disable all media links in the ImgU instance to prepare
 * for capture operations
 *
 * \todo This method will probably be removed or changed once links will be
 * enabled or disabled selectively.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::enableLinks(bool enable)
{
	std::string viewfinderName = name_ + " viewfinder";
	std::string outputName = name_ + " output";
	std::string statName = name_ + " 3a stat";
	std::string inputName = name_ + " input";
	int ret;

	ret = linkSetup(inputName, 0, name_, PAD_INPUT, enable);
	if (ret)
		return ret;

	ret = linkSetup(name_, PAD_OUTPUT, outputName, 0, enable);
	if (ret)
		return ret;

	ret = linkSetup(name_, PAD_VF, viewfinderName, 0, enable);
	if (ret)
		return ret;

	return linkSetup(name_, PAD_STAT, statName, 0, enable);
}

} /* namespace libcamera */
