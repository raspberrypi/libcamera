/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * rkisp1path.cpp - Rockchip ISP1 path helper
 */

#include "rkisp1_path.h"

#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RkISP1)

RkISP1Path::RkISP1Path(const char *name)
	: video_(nullptr), name_(name), resizer_(nullptr)
{
}

RkISP1Path::~RkISP1Path()
{
	delete video_;
	delete resizer_;
}

bool RkISP1Path::init(MediaDevice *media)
{
	std::string resizer = std::string("rkisp1_resizer_") + name_ + "path";
	std::string video = std::string("rkisp1_") + name_ + "path";

	resizer_ = V4L2Subdevice::fromEntityName(media, resizer);
	if (resizer_->open() < 0)
		return false;

	video_ = V4L2VideoDevice::fromEntityName(media, video);
	if (video_->open() < 0)
		return false;

	return true;
}

int RkISP1Path::configure(const StreamConfiguration &config,
			  const V4L2SubdeviceFormat &inputFormat)
{
	int ret;

	V4L2SubdeviceFormat ispFormat = inputFormat;

	ret = resizer_->setFormat(0, &ispFormat);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "Configured " << name_ << " resizer input pad with "
		<< ispFormat.toString();

	ispFormat.size = config.size;

	LOG(RkISP1, Debug)
		<< "Configuring " << name_ << " resizer output pad with "
		<< ispFormat.toString();

	ret = resizer_->setFormat(1, &ispFormat);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "Configured " << name_ << " resizer output pad with "
		<< ispFormat.toString();

	const PixelFormatInfo &info = PixelFormatInfo::info(config.pixelFormat);
	V4L2DeviceFormat outputFormat = {};
	outputFormat.fourcc = video_->toV4L2PixelFormat(config.pixelFormat);
	outputFormat.size = config.size;
	outputFormat.planesCount = info.numPlanes();

	ret = video_->setFormat(&outputFormat);
	if (ret)
		return ret;

	if (outputFormat.size != config.size ||
	    outputFormat.fourcc != video_->toV4L2PixelFormat(config.pixelFormat)) {
		LOG(RkISP1, Error)
			<< "Unable to configure capture in " << config.toString();
		return -EINVAL;
	}

	return 0;
}

RkISP1MainPath::RkISP1MainPath()
	: RkISP1Path("main")
{
}

RkISP1SelfPath::RkISP1SelfPath()
	: RkISP1Path("self")
{
}

} /* namespace libcamera */
