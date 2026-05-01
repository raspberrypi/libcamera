/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Mali-C55 IPA Context
 */

#pragma once

#include <libcamera/base/utils.h>
#include <libcamera/controls.h>

#include "libcamera/internal/bayer_format.h"

#include <libipa/fc_queue.h>

#include "libipa/fixedpoint.h"

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
			UQ<5, 8> ispGain;
		} automatic;
		struct {
			uint32_t exposure;
			double sensorGain;
			UQ<5, 8> ispGain;
		} manual;
		bool autoEnabled;
		uint32_t constraintMode;
		uint32_t exposureMode;
		uint32_t temperatureK;
	} agc;

	struct {
		UQ<4, 8> rGain;
		UQ<4, 8> bGain;
	} awb;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double sensorGain;
		UQ<5, 8> ispGain;
	} agc;

	struct {
		UQ<4, 8> rGain;
		UQ<4, 8> bGain;
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
