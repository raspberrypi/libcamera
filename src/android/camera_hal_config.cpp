/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_hal_config.cpp - Camera HAL configuration file manager
 */
#include "camera_hal_config.h"

#include <stdlib.h>
#include <string>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include <hardware/camera3.h>

using namespace libcamera;

LOG_DEFINE_CATEGORY(HALConfig)

class CameraHalConfig::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(CameraHalConfig)

public:
	Private();

	int parseConfigFile(File &file, std::map<std::string, CameraConfigData> *cameras);

private:
	int parseCameraConfigData(const std::string &cameraId, const YamlObject &);
	int parseLocation(const YamlObject &, CameraConfigData &cameraConfigData);
	int parseRotation(const YamlObject &, CameraConfigData &cameraConfigData);

	std::map<std::string, CameraConfigData> *cameras_;
};

CameraHalConfig::Private::Private()
{
}

int CameraHalConfig::Private::parseConfigFile(File &file,
					      std::map<std::string, CameraConfigData> *cameras)
{
	/*
	 * Parse the HAL properties.
	 *
	 * Each camera properties block is a list of properties associated
	 * with the ID (as assembled by CameraSensor::generateId()) of the
	 * camera they refer to.
	 *
	 * cameras:
	 *   "camera0 id":
	 *     location: value
	 *     rotation: value
	 *     ...
	 *
	 *   "camera1 id":
	 *     location: value
	 *     rotation: value
	 *     ...
	 */

	cameras_ = cameras;

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	if (!root)
		return -EINVAL;

	if (!root->isDictionary())
		return -EINVAL;

	/* Parse property "cameras" */
	if (!root->contains("cameras"))
		return -EINVAL;

	const YamlObject &yamlObjectCameras = (*root)["cameras"];

	if (!yamlObjectCameras.isDictionary())
		return -EINVAL;

	for (const auto &[cameraId, configData] : yamlObjectCameras.asDict()) {
		if (parseCameraConfigData(cameraId, configData))
			return -EINVAL;
	}

	return 0;
}

int CameraHalConfig::Private::parseCameraConfigData(const std::string &cameraId,
						    const YamlObject &cameraObject)

{
	if (!cameraObject.isDictionary())
		return -EINVAL;

	CameraConfigData &cameraConfigData = (*cameras_)[cameraId];

	/* Parse property "location" */
	if (parseLocation(cameraObject, cameraConfigData))
		return -EINVAL;

	/* Parse property "rotation" */
	if (parseRotation(cameraObject, cameraConfigData))
		return -EINVAL;

	return 0;
}

int CameraHalConfig::Private::parseLocation(const YamlObject &cameraObject,
					    CameraConfigData &cameraConfigData)
{
	if (!cameraObject.contains("location"))
		return -EINVAL;

	std::string location = cameraObject["location"].get<std::string>("");

	if (location == "front")
		cameraConfigData.facing = CAMERA_FACING_FRONT;
	else if (location == "back")
		cameraConfigData.facing = CAMERA_FACING_BACK;
	else
		return -EINVAL;

	return 0;
}

int CameraHalConfig::Private::parseRotation(const YamlObject &cameraObject,
					    CameraConfigData &cameraConfigData)
{
	if (!cameraObject.contains("rotation"))
		return -EINVAL;

	int32_t rotation = cameraObject["rotation"].get<int32_t>(-1);

	if (rotation < 0 || rotation >= 360) {
		LOG(HALConfig, Error)
			<< "Unknown rotation: " << rotation;
		return -EINVAL;
	}

	cameraConfigData.rotation = rotation;
	return 0;
}

CameraHalConfig::CameraHalConfig()
	: Extensible(std::make_unique<Private>()), exists_(false), valid_(false)
{
	parseConfigurationFile();
}

/*
 * Open the HAL configuration file and validate its content.
 * Return 0 on success, a negative error code otherwise
 * retval -ENOENT The configuration file is not available
 * retval -EINVAL The configuration file is available but not valid
 */
int CameraHalConfig::parseConfigurationFile()
{
	std::string filePath = LIBCAMERA_SYSCONF_DIR "/camera_hal.yaml";

	File file(filePath);
	if (!file.exists()) {
		LOG(HALConfig, Debug)
			<< "Configuration file: \"" << filePath << "\" not found";
		return -ENOENT;
	}

	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(HALConfig, Error) << "Failed to open configuration file "
				      << filePath << ": " << strerror(-ret);
		return ret;
	}

	exists_ = true;

	int ret = _d()->parseConfigFile(file, &cameras_);
	if (ret)
		return -EINVAL;

	valid_ = true;

	for (const auto &c : cameras_) {
		const std::string &cameraId = c.first;
		const CameraConfigData &camera = c.second;
		LOG(HALConfig, Debug) << "'" << cameraId << "' "
				      << "(" << camera.facing << ")["
				      << camera.rotation << "]";
	}

	return 0;
}

const CameraConfigData *CameraHalConfig::cameraConfigData(const std::string &cameraId) const
{
	const auto &it = cameras_.find(cameraId);
	if (it == cameras_.end()) {
		LOG(HALConfig, Error)
			<< "Camera '" << cameraId
			<< "' not described in the HAL configuration file";
		return nullptr;
	}

	return &it->second;
}
