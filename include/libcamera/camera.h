/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.h - Camera object interface
 */
#ifndef __LIBCAMERA_CAMERA_H__
#define __LIBCAMERA_CAMERA_H__

#include <memory>
#include <string>

namespace libcamera {

class Camera final
{
public:
	static std::shared_ptr<Camera> create(const std::string &name);

	Camera(const Camera &) = delete;
	void operator=(const Camera &) = delete;

	const std::string &name() const;

private:
	explicit Camera(const std::string &name);
	~Camera();

	std::string name_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CAMERA_H__ */
