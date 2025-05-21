/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Virtual cameras helper to parse config file
 */

#include "config_parser.h"

#include <string.h>
#include <utility>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>

#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/yaml_parser.h"

#include "virtual.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Virtual)

std::vector<std::unique_ptr<VirtualCameraData>>
ConfigParser::parseConfigFile(File &file, PipelineHandler *pipe)
{
	std::vector<std::unique_ptr<VirtualCameraData>> configurations;

	std::unique_ptr<YamlObject> cameras = YamlParser::parse(file);
	if (!cameras) {
		LOG(Virtual, Error) << "Failed to parse config file.";
		return configurations;
	}

	if (!cameras->isDictionary()) {
		LOG(Virtual, Error) << "Config file is not a dictionary at the top level.";
		return configurations;
	}

	/* Look into the configuration of each camera */
	for (const auto &[cameraId, cameraConfigData] : cameras->asDict()) {
		std::unique_ptr<VirtualCameraData> data =
			parseCameraConfigData(cameraConfigData, pipe);
		/* Parse configData to data */
		if (!data) {
			/* Skip the camera if it has invalid config */
			LOG(Virtual, Error) << "Failed to parse config of the camera: "
					    << cameraId;
			continue;
		}

		data->config_.id = cameraId;
		ControlInfoMap::Map controls;
		/* \todo Check which resolution's frame rate to be reported */
		controls[&controls::FrameDurationLimits] =
			ControlInfo(1000000 / data->config_.resolutions[0].frameRates[1],
				    1000000 / data->config_.resolutions[0].frameRates[0]);

		std::vector<ControlValue> supportedFaceDetectModes{
			static_cast<int32_t>(controls::draft::FaceDetectModeOff),
		};
		controls[&controls::draft::FaceDetectMode] = ControlInfo(supportedFaceDetectModes);

		data->controlInfo_ = ControlInfoMap(std::move(controls), controls::controls);
		configurations.push_back(std::move(data));
	}

	return configurations;
}

std::unique_ptr<VirtualCameraData>
ConfigParser::parseCameraConfigData(const YamlObject &cameraConfigData,
				    PipelineHandler *pipe)
{
	std::vector<VirtualCameraData::Resolution> resolutions;
	if (parseSupportedFormats(cameraConfigData, &resolutions))
		return nullptr;

	std::unique_ptr<VirtualCameraData> data =
		std::make_unique<VirtualCameraData>(pipe, resolutions);

	if (parseFrameGenerator(cameraConfigData, data.get()))
		return nullptr;

	if (parseLocation(cameraConfigData, data.get()))
		return nullptr;

	if (parseModel(cameraConfigData, data.get()))
		return nullptr;

	return data;
}

int ConfigParser::parseSupportedFormats(const YamlObject &cameraConfigData,
					std::vector<VirtualCameraData::Resolution> *resolutions)
{
	if (cameraConfigData.contains("supported_formats")) {
		const YamlObject &supportedResolutions = cameraConfigData["supported_formats"];

		for (const YamlObject &supportedResolution : supportedResolutions.asList()) {
			unsigned int width = supportedResolution["width"].get<unsigned int>(1920);
			unsigned int height = supportedResolution["height"].get<unsigned int>(1080);
			if (width == 0 || height == 0) {
				LOG(Virtual, Error) << "Invalid width or/and height";
				return -EINVAL;
			}
			if (width % 2 != 0) {
				LOG(Virtual, Error) << "Invalid width: width needs to be even";
				return -EINVAL;
			}

			std::vector<int64_t> frameRates;
			if (supportedResolution.contains("frame_rates")) {
				auto frameRatesList =
					supportedResolution["frame_rates"].getList<int>();
				if (!frameRatesList || (frameRatesList->size() != 1 &&
							frameRatesList->size() != 2)) {
					LOG(Virtual, Error) << "Invalid frame_rates: either one or two values";
					return -EINVAL;
				}

				if (frameRatesList->size() == 2 &&
				    frameRatesList.value()[0] > frameRatesList.value()[1]) {
					LOG(Virtual, Error) << "frame_rates's first value(lower bound)"
							    << " is higher than the second value(upper bound)";
					return -EINVAL;
				}
				/*
                                 * Push the min and max framerates. A
                                 * single rate is duplicated.
                                 */
				frameRates.push_back(frameRatesList.value().front());
				frameRates.push_back(frameRatesList.value().back());
			} else {
				frameRates.push_back(30);
				frameRates.push_back(60);
			}

			resolutions->emplace_back(
				VirtualCameraData::Resolution{ Size{ width, height },
							       frameRates });
		}
	} else {
		resolutions->emplace_back(
			VirtualCameraData::Resolution{ Size{ 1920, 1080 },
						       { 30, 60 } });
	}

	return 0;
}

int ConfigParser::parseFrameGenerator(const YamlObject &cameraConfigData, VirtualCameraData *data)
{
	const std::string testPatternKey = "test_pattern";
	const std::string framesKey = "frames";
	if (cameraConfigData.contains(testPatternKey)) {
		if (cameraConfigData.contains(framesKey)) {
			LOG(Virtual, Error) << "A camera should use either "
					    << testPatternKey << " or " << framesKey;
			return -EINVAL;
		}

		auto testPattern = cameraConfigData[testPatternKey].get<std::string>("");

		if (testPattern == "bars") {
			data->config_.frame = TestPattern::ColorBars;
		} else if (testPattern == "lines") {
			data->config_.frame = TestPattern::DiagonalLines;
		} else {
			LOG(Virtual, Debug) << "Test pattern: " << testPattern
					    << " is not supported";
			return -EINVAL;
		}

		return 0;
	}

	const YamlObject &frames = cameraConfigData[framesKey];

	/* When there is no frames provided in the config file, use color bar test pattern */
	if (!frames) {
		data->config_.frame = TestPattern::ColorBars;
		return 0;
	}

	if (!frames.isDictionary()) {
		LOG(Virtual, Error) << "'frames' is not a dictionary.";
		return -EINVAL;
	}

	auto path = frames["path"].get<std::string>();

	if (!path) {
		LOG(Virtual, Error) << "Test pattern or path should be specified.";
		return -EINVAL;
	}

	std::vector<std::filesystem::path> files;

	switch (std::filesystem::symlink_status(*path).type()) {
	case std::filesystem::file_type::regular:
		files.push_back(*path);
		break;

	case std::filesystem::file_type::directory:
		for (const auto &dentry : std::filesystem::directory_iterator{ *path }) {
			if (dentry.is_regular_file())
				files.push_back(dentry.path());
		}

		std::sort(files.begin(), files.end(), [](const auto &a, const auto &b) {
			return ::strverscmp(a.c_str(), b.c_str()) < 0;
		});

		if (files.empty()) {
			LOG(Virtual, Error) << "Directory has no files: " << *path;
			return -EINVAL;
		}
		break;

	default:
		LOG(Virtual, Error) << "Frame: " << *path << " is not supported";
		return -EINVAL;
	}

	data->config_.frame = ImageFrames{ std::move(files) };

	return 0;
}

int ConfigParser::parseLocation(const YamlObject &cameraConfigData, VirtualCameraData *data)
{
	/* Default value is properties::CameraLocationFront */
	int32_t location = properties::CameraLocationFront;

	if (auto l = cameraConfigData["location"].get<std::string>()) {
		auto it = properties::LocationNameValueMap.find(*l);
		if (it == properties::LocationNameValueMap.end()) {
			LOG(Virtual, Error)
				<< "location: " << *l << " is not supported";
			return -EINVAL;
		}

		location = it->second;
	}

	data->properties_.set(properties::Location, location);

	return 0;
}

int ConfigParser::parseModel(const YamlObject &cameraConfigData, VirtualCameraData *data)
{
	std::string model = cameraConfigData["model"].get<std::string>("Unknown");

	data->properties_.set(properties::Model, model);

	return 0;
}

} /* namespace libcamera */
