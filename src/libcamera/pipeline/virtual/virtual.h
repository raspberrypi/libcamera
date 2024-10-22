/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Pipeline handler for virtual cameras
 */

#pragma once

#include <vector>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/pipeline_handler.h"

#include "test_pattern_generator.h"

namespace libcamera {

class VirtualCameraData : public Camera::Private
{
public:
	const static unsigned int kMaxStream = 3;

	struct Resolution {
		Size size;
		std::vector<int> frameRates;
	};
	struct StreamConfig {
		Stream stream;
		std::unique_ptr<FrameGenerator> frameGenerator;
	};

	VirtualCameraData(PipelineHandler *pipe,
			  const std::vector<Resolution> &supportedResolutions);

	~VirtualCameraData() = default;

	TestPattern testPattern_ = TestPattern::ColorBars;

	const std::vector<Resolution> supportedResolutions_;
	Size maxResolutionSize_;
	Size minResolutionSize_;

	std::vector<StreamConfig> streamConfigs_;
};

} /* namespace libcamera */