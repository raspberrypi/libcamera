/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * V4L2 compatibility camera file information
 */

#include "v4l2_camera_file.h"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "v4l2_camera_proxy.h"

using namespace libcamera;

V4L2CameraFile::V4L2CameraFile(int dirfd, const char *path, int efd,
			       bool nonBlocking, V4L2CameraProxy *proxy)
	: proxy_(proxy), nonBlocking_(nonBlocking), efd_(efd),
	  priority_(V4L2_PRIORITY_DEFAULT)
{
	proxy_->open(this);

	if (path[0] != '/') {
		if (dirfd == AT_FDCWD) {
			char *cwd = getcwd(nullptr, 0);
			if (cwd) {
				description_ = std::string(cwd) + "/";
				free(cwd);
			} else {
				description_ = std::string("(unreachable)/");
			}
		} else {
			description_ = "(dirfd:" + std::to_string(dirfd) + ")/";
		}
	}

	description_ += std::string(path) + " (fd:" + std::to_string(efd) + ")";
}

V4L2CameraFile::~V4L2CameraFile()
{
	proxy_->close(this);
}

const std::string &V4L2CameraFile::description() const
{
	return description_;
}
