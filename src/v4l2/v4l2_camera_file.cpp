/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * v4l2_camera_file.h - V4L2 compatibility camera file information
 */

#include "v4l2_camera_file.h"

#include <linux/videodev2.h>

#include "v4l2_camera_proxy.h"

using namespace libcamera;

V4L2CameraFile::V4L2CameraFile(int efd, bool nonBlocking, V4L2CameraProxy *proxy)
	: proxy_(proxy), nonBlocking_(nonBlocking), efd_(efd),
	  priority_(V4L2_PRIORITY_DEFAULT)
{
	proxy_->open(this);
}

V4L2CameraFile::~V4L2CameraFile()
{
	proxy_->close(this);
}
