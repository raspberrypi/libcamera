/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Mali-C55 IPA Context
 */

#pragma once

#include <libcamera/base/utils.h>
#include <libcamera/controls.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"

#include <libipa/fc_queue.h>

#include "libipa/awb.h"
#include "libipa/ccm.h"
#include "libipa/fixedpoint.h"
#include "libipa/lsc.h"

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

	ipa::awb::ActiveState awb;
	ipa::ccm::ActiveState ccm;
	ipa::lsc::ActiveState lsc;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double sensorGain;
		UQ<5, 8> ispGain;
	} agc;

	ipa::awb::FrameContext awb;
	ipa::ccm::FrameContext ccm;
	ipa::lsc::FrameContext lsc;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPACameraSensorInfo sensorInfo;
	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;
};

} /* namespace ipa::mali_c55 */

} /* namespace libcamera*/
