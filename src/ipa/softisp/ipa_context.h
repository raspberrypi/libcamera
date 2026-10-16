/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat, Inc.
 *
 * Software ISP IPA Context
 */

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <stdint.h>

#include <libcamera/controls.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include <libipa/agc.h>
#include <libipa/awb.h>
#include <libipa/camera_sensor_helper.h>
#include <libipa/ccm.h>
#include <libipa/fc_queue.h>

#include "core_ipa_interface.h"

namespace libcamera {

namespace ipa::softisp {

struct IPASessionConfiguration {
	ipa::agc::Session agc;
	struct {
		std::optional<uint8_t> level;
	} black;
};

struct IPAActiveState {
	ipa::awb::ActiveState awb;
	ipa::ccm::ActiveState ccm;
	ipa::agc::ActiveState agc;

	struct {
		uint8_t level;
		uint32_t lastExposure;
		double lastGain;
	} blc;

	Matrix<float, 3, 3> combinedMatrix;

	struct {
		float gamma;
		/* 0..2 range, 1.0 = normal */
		std::optional<float> contrast;
		std::optional<float> saturation;
	} knobs;
};

struct IPAFrameContext : public FrameContext {
	ipa::awb::FrameContext awb;
	ipa::ccm::FrameContext ccm;
	ipa::agc::FrameContext agc;

	struct {
		uint32_t exposure;
		double gain;
	} sensor;

	float gamma;
	std::optional<float> contrast;
	std::optional<float> saturation;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
	std::unique_ptr<CameraSensorHelper> camHelper;
	IPASessionConfiguration configuration;
	IPAActiveState activeState;
	FCQueue<IPAFrameContext> frameContexts;
	ControlInfoMap::Map ctrlMap;
	bool ccmEnabled = false;
};

} /* namespace ipa::softisp */

} /* namespace libcamera */
