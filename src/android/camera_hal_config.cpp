/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_hal_config.cpp - Camera HAL configuration file manager
 */
#include "camera_hal_config.h"

#if defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE < 8
#include <experimental/filesystem>
namespace std {
namespace filesystem = std::experimental::filesystem;
}
#else
#include <filesystem>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <yaml.h>

#include <hardware/camera3.h>

#include <libcamera/base/log.h>

using namespace libcamera;

LOG_DEFINE_CATEGORY(HALConfig)

class CameraHalConfig::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(CameraHalConfig)

public:
	Private();

	int parseConfigFile(FILE *fh, std::map<std::string, CameraConfigData> *cameras);

private:
	std::string parseValue();
	std::string parseKey();
	int parseValueBlock();
	int parseCameraLocation(CameraConfigData *cameraConfigData,
				const std::string &location);
	int parseCameraConfigData(const std::string &cameraId);
	int parseCameras();
	int parseEntry();

	yaml_parser_t parser_;
	std::map<std::string, CameraConfigData> *cameras_;
};

CameraHalConfig::Private::Private()
{
}

std::string CameraHalConfig::Private::parseValue()
{
	yaml_token_t token;

	/* Make sure the token type is a value and get its content. */
	yaml_parser_scan(&parser_, &token);
	if (token.type != YAML_VALUE_TOKEN) {
		yaml_token_delete(&token);
		return "";
	}
	yaml_token_delete(&token);

	yaml_parser_scan(&parser_, &token);
	if (token.type != YAML_SCALAR_TOKEN) {
		yaml_token_delete(&token);
		return "";
	}

	std::string value(reinterpret_cast<char *>(token.data.scalar.value),
			  token.data.scalar.length);
	yaml_token_delete(&token);

	return value;
}

std::string CameraHalConfig::Private::parseKey()
{
	yaml_token_t token;

	/* Make sure the token type is a key and get its value. */
	yaml_parser_scan(&parser_, &token);
	if (token.type != YAML_SCALAR_TOKEN) {
		yaml_token_delete(&token);
		return "";
	}

	std::string value(reinterpret_cast<char *>(token.data.scalar.value),
			  token.data.scalar.length);
	yaml_token_delete(&token);

	return value;
}

int CameraHalConfig::Private::parseValueBlock()
{
	yaml_token_t token;

	/* Make sure the next token are VALUE and BLOCK_MAPPING_START. */
	yaml_parser_scan(&parser_, &token);
	if (token.type != YAML_VALUE_TOKEN) {
		yaml_token_delete(&token);
		return -EINVAL;
	}
	yaml_token_delete(&token);

	yaml_parser_scan(&parser_, &token);
	if (token.type != YAML_BLOCK_MAPPING_START_TOKEN) {
		yaml_token_delete(&token);
		return -EINVAL;
	}
	yaml_token_delete(&token);

	return 0;
}

int CameraHalConfig::Private::parseCameraLocation(CameraConfigData *cameraConfigData,
						  const std::string &location)
{
	if (location == "front")
		cameraConfigData->facing = CAMERA_FACING_FRONT;
	else if (location == "back")
		cameraConfigData->facing = CAMERA_FACING_BACK;
	else
		return -EINVAL;

	return 0;
}

int CameraHalConfig::Private::parseCameraConfigData(const std::string &cameraId)
{
	int ret = parseValueBlock();
	if (ret)
		return ret;

	/*
	 * Parse the camera properties and store them in a cameraConfigData
	 * instance.
	 *
	 * Add a safety counter to make sure we don't loop indefinitely in case
	 * the configuration file is malformed.
	 */
	CameraConfigData cameraConfigData;
	unsigned int sentinel = 100;
	bool blockEnd = false;
	yaml_token_t token;

	do {
		yaml_parser_scan(&parser_, &token);
		switch (token.type) {
		case YAML_KEY_TOKEN: {
			yaml_token_delete(&token);

			/*
			 * Parse the camera property key and make sure it is
			 * valid.
			 */
			std::string key = parseKey();
			std::string value = parseValue();
			if (key.empty() || value.empty())
				return -EINVAL;

			if (key == "location") {
				ret = parseCameraLocation(&cameraConfigData, value);
				if (ret) {
					LOG(HALConfig, Error)
						<< "Unknown location: " << value;
					return -EINVAL;
				}
			} else if (key == "rotation") {
				ret = std::stoi(value);
				if (ret < 0 || ret >= 360) {
					LOG(HALConfig, Error)
						<< "Unknown rotation: " << value;
					return -EINVAL;
				}
				cameraConfigData.rotation = ret;
			} else {
				LOG(HALConfig, Error)
					<< "Unknown key: " << key;
				return -EINVAL;
			}
			break;
		}

		case YAML_BLOCK_END_TOKEN:
			blockEnd = true;
			[[fallthrough]];
		default:
			yaml_token_delete(&token);
			break;
		}

		--sentinel;
	} while (!blockEnd && sentinel);
	if (!sentinel)
		return -EINVAL;

	(*cameras_)[cameraId] = cameraConfigData;

	return 0;
}

int CameraHalConfig::Private::parseCameras()
{
	int ret = parseValueBlock();
	if (ret) {
		LOG(HALConfig, Error) << "Configuration file is not valid";
		return ret;
	}

	/*
	 * Parse the camera properties.
	 *
	 * Each camera properties block is a list of properties associated
	 * with the ID (as assembled by CameraSensor::generateId()) of the
	 * camera they refer to.
	 *
	 * cameras:
	 *   "camera0 id":
	 *     key: value
	 *     key: value
	 *     ...
	 *
	 *   "camera1 id":
	 *     key: value
	 *     key: value
	 *     ...
	 */
	bool blockEnd = false;
	yaml_token_t token;
	do {
		yaml_parser_scan(&parser_, &token);
		switch (token.type) {
		case YAML_KEY_TOKEN: {
			yaml_token_delete(&token);

			/* Parse the camera ID as key of the property list. */
			std::string cameraId = parseKey();
			if (cameraId.empty())
				return -EINVAL;

			ret = parseCameraConfigData(cameraId);
			if (ret)
				return -EINVAL;
			break;
		}
		case YAML_BLOCK_END_TOKEN:
			blockEnd = true;
			[[fallthrough]];
		default:
			yaml_token_delete(&token);
			break;
		}
	} while (!blockEnd);

	return 0;
}

int CameraHalConfig::Private::parseEntry()
{
	int ret = -EINVAL;

	/*
	 * Parse each key we find in the file.
	 *
	 * The 'cameras' keys maps to a list of (lists) of camera properties.
	 */

	std::string key = parseKey();
	if (key.empty())
		return ret;

	if (key == "cameras")
		ret = parseCameras();
	else
		LOG(HALConfig, Error) << "Unknown key: " << key;

	return ret;
}

int CameraHalConfig::Private::parseConfigFile(FILE *fh,
					      std::map<std::string, CameraConfigData> *cameras)
{
	cameras_ = cameras;

	int ret = yaml_parser_initialize(&parser_);
	if (!ret) {
		LOG(HALConfig, Error) << "Failed to initialize yaml parser";
		return -EINVAL;
	}
	yaml_parser_set_input_file(&parser_, fh);

	yaml_token_t token;
	yaml_parser_scan(&parser_, &token);
	if (token.type != YAML_STREAM_START_TOKEN) {
		LOG(HALConfig, Error) << "Configuration file is not valid";
		yaml_token_delete(&token);
		yaml_parser_delete(&parser_);
		return -EINVAL;
	}
	yaml_token_delete(&token);

	yaml_parser_scan(&parser_, &token);
	if (token.type != YAML_BLOCK_MAPPING_START_TOKEN) {
		LOG(HALConfig, Error) << "Configuration file is not valid";
		yaml_token_delete(&token);
		yaml_parser_delete(&parser_);
		return -EINVAL;
	}
	yaml_token_delete(&token);

	/* Parse the file and parse each single key one by one. */
	do {
		yaml_parser_scan(&parser_, &token);
		switch (token.type) {
		case YAML_KEY_TOKEN:
			yaml_token_delete(&token);
			ret = parseEntry();
			break;

		case YAML_STREAM_END_TOKEN:
			ret = -ENOENT;
			[[fallthrough]];
		default:
			yaml_token_delete(&token);
			break;
		}
	} while (ret >= 0);
	yaml_parser_delete(&parser_);

	if (ret && ret != -ENOENT)
		LOG(HALConfig, Error) << "Configuration file is not valid";

	return ret == -ENOENT ? 0 : ret;
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
	std::filesystem::path filePath = LIBCAMERA_SYSCONF_DIR;
	filePath /= "camera_hal.yaml";
	if (!std::filesystem::is_regular_file(filePath)) {
		LOG(HALConfig, Debug)
			<< "Configuration file: \"" << filePath << "\" not found";
		return -ENOENT;
	}

	FILE *fh = fopen(filePath.c_str(), "r");
	if (!fh) {
		int ret = -errno;
		LOG(HALConfig, Error) << "Failed to open configuration file "
				      << filePath << ": " << strerror(-ret);
		return ret;
	}

	exists_ = true;

	int ret = _d()->parseConfigFile(fh, &cameras_);
	fclose(fh);
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
