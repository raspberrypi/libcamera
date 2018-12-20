/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.h - Camera object interface
 */
#ifndef __LIBCAMERA_CAMERA_H__
#define __LIBCAMERA_CAMERA_H__

#include <string>

namespace libcamera {

class Camera
{
public:
	Camera(const std::string &name);

	const std::string &name() const;
	void get();
	void put();

private:
	virtual ~Camera() { };
	int ref_;
	std::string name_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CAMERA_H__ */
