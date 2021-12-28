/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * v4l2_camera_file.h - V4L2 compatibility camera file information
 */

#pragma once

#include <string>

#include <linux/videodev2.h>

class V4L2CameraProxy;

class V4L2CameraFile
{
public:
	V4L2CameraFile(int dirfd, const char *path, int efd, bool nonBlocking,
		       V4L2CameraProxy *proxy);
	~V4L2CameraFile();

	V4L2CameraProxy *proxy() const { return proxy_; }

	bool nonBlocking() const { return nonBlocking_; }
	int efd() const { return efd_; }

	enum v4l2_priority priority() const { return priority_; }
	void setPriority(enum v4l2_priority priority) { priority_ = priority; }

	const std::string &description() const;

private:
	V4L2CameraProxy *proxy_;

	std::string description_;
	bool nonBlocking_;
	int efd_;
	enum v4l2_priority priority_;
};
