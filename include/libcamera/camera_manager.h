/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera_manager.h - Camera management
 */
#ifndef __LIBCAMERA_CAMERA_MANAGER_H__
#define __LIBCAMERA_CAMERA_MANAGER_H__

#include <vector>
#include <string>

namespace libcamera {

class Camera;
class DeviceEnumerator;
class PipelineHandler;

class CameraManager
{
public:
	CameraManager();

	int start();
	void stop();

	std::vector<std::string> list() const;
	Camera *get(const std::string &name);

private:
	DeviceEnumerator *enumerator_;
	std::vector<PipelineHandler *> pipes_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CAMERA_MANAGER_H__ */
