/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * rkisp1path.cpp - Rockchip ISP1 path helper
 */

#include "rkisp1_path.h"

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

RkISP1Path::RkISP1Path(const char *name)
	: resizer_(nullptr), video_(nullptr), name_(name)
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

RkISP1MainPath::RkISP1MainPath()
	: RkISP1Path("main")
{
}

RkISP1SelfPath::RkISP1SelfPath()
	: RkISP1Path("self")
{
}

} /* namespace libcamera */
