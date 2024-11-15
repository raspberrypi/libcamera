/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * ipa_context.h - Mali-C55 IPA Context
 */

#pragma once

#include <libcamera/base/utils.h>
#include <libcamera/controls.h>

#include "libcamera/internal/bayer_format.h"

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::mali_c55 {

struct IPASessionConfiguration {
	struct {
		utils::Duration minShutterSpeed;
		utils::Duration maxShutterSpeed;
		uint32_t defaultExposure;
		double minAnalogueGain;
		double maxAnalogueGain;
	} agc;

	struct {
		BayerFormat::Order bayerOrder;
		utils::Duration lineDuration;
		uint32_t blackLevel;
	} sensor;
};

struct IPAActiveState {
	struct {
		struct {
			uint32_t exposure;
			double sensorGain;
			double ispGain;
		} automatic;
		struct {
			uint32_t exposure;
			double sensorGain;
			double ispGain;
		} manual;
		bool autoEnabled;
		uint32_t constraintMode;
		uint32_t exposureMode;
		uint32_t temperatureK;
	} agc;

	struct {
		double rGain;
		double bGain;
	} awb;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double sensorGain;
		double ispGain;
	} agc;

	struct {
		double rGain;
		double bGain;
	} awb;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;
};

} /* namespace ipa::mali_c55 */

} /* namespace libcamera*/
