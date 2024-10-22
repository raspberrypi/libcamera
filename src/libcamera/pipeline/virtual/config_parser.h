/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Virtual cameras helper to parse config file
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/base/file.h>

#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/yaml_parser.h"

#include "virtual.h"

namespace libcamera {

class ConfigParser
{
public:
	std::vector<std::unique_ptr<VirtualCameraData>>
	parseConfigFile(File &file, PipelineHandler *pipe);

private:
	std::unique_ptr<VirtualCameraData>
	parseCameraConfigData(const YamlObject &cameraConfigData, PipelineHandler *pipe);

	int parseSupportedFormats(const YamlObject &cameraConfigData,
				  std::vector<VirtualCameraData::Resolution> *resolutions);
	int parseFrameGenerator(const YamlObject &cameraConfigData, VirtualCameraData *data);
	int parseLocation(const YamlObject &cameraConfigData, VirtualCameraData *data);
	int parseModel(const YamlObject &cameraConfigData, VirtualCameraData *data);
};

} /* namespace libcamera */
