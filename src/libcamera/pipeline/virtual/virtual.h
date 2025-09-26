/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Pipeline handler for virtual cameras
 */

#pragma once

#include <string>
#include <variant>
#include <vector>

#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/pipeline_handler.h"

#include "frame_generator.h"
#include "image_frame_generator.h"
#include "test_pattern_generator.h"

namespace libcamera {

using VirtualFrame = std::variant<TestPattern, ImageFrames>;

class VirtualCameraData : public Camera::Private,
			  public Thread,
			  public Object
{
public:
	const static unsigned int kMaxStream = 3;

	struct Resolution {
		Size size;
		std::vector<int64_t> frameRates;
	};
	struct StreamConfig {
		Stream stream;
		std::unique_ptr<FrameGenerator> frameGenerator;
		unsigned int seq = 0;
	};
	/* The config file is parsed to the Configuration struct */
	struct Configuration {
		std::string id;
		std::vector<Resolution> resolutions;
		VirtualFrame frame;

		Size maxResolutionSize;
		Size minResolutionSize;
	};

	VirtualCameraData(PipelineHandler *pipe,
			  const std::vector<Resolution> &supportedResolutions);

	~VirtualCameraData() = default;

	void processRequest(Request *request);

	Configuration config_;

	std::vector<StreamConfig> streamConfigs_;
	Signal<FrameBuffer *> bufferCompleted;
};

} /* namespace libcamera */
