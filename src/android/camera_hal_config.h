/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_hal_config.h - Camera HAL configuration file manager
 */

#pragma once

#include <map>
#include <string>

#include <libcamera/base/class.h>

struct CameraConfigData {
	int facing = -1;
	int rotation = -1;
};

class CameraHalConfig final : public libcamera::Extensible
{
	LIBCAMERA_DECLARE_PRIVATE()

public:
	CameraHalConfig();

	bool exists() const { return exists_; }
	bool isValid() const { return valid_; }

	const CameraConfigData *cameraConfigData(const std::string &cameraId) const;

private:
	bool exists_;
	bool valid_;
	std::map<std::string, CameraConfigData> cameras_;

	int parseConfigurationFile();
};
