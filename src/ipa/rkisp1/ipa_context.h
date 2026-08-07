/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 IPA Context
 *
 */

#pragma once

#include <memory>

#include <linux/rkisp1-config.h>

#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/debug_controls.h"
#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include "libipa/agc_mean_luminance.h"
#include "libipa/awb.h"
#include "libipa/camera_sensor_helper.h"
#include "libipa/ccm.h"
#include "libipa/fc_queue.h"
#include "libipa/fixedpoint.h"
#include "libipa/lsc.h"

namespace libcamera {

namespace ipa::rkisp1 {

/* Fixed point types used by CPROC */
using BrightnessQ = Q<1, 7>;
using ContrastQ = UQ<1, 7>;
using HueQ = Q<1, 7>;
using SaturationQ = UQ<1, 7>;

struct IPAHwSettings {
	unsigned int numAeCells;
	unsigned int numHistogramBins;
	unsigned int numHistogramWeights;
	unsigned int numGammaOutSamples;
	uint32_t supportedBlocks;
	bool compand;
};

struct RKISP1AwbSession {
	struct rkisp1_cif_isp_window measureWindow;
	bool enabled;
};

struct IPASessionConfiguration {
	struct {
		struct rkisp1_cif_isp_window measureWindow;
	} agc;

	struct RKISP1AwbSession awb;

	struct {
		bool supported;
	} compress;

	struct {
		utils::Duration minExposureTime;
		utils::Duration maxExposureTime;
		double minAnalogueGain;
		double maxAnalogueGain;

		int32_t defVBlank;
		utils::Duration lineDuration;
		Size size;
	} sensor;

	bool raw;
	uint32_t paramFormat;
};

struct IPAActiveState {
	struct {
		struct {
			uint32_t exposure;
			double gain;
		} manual;
		struct {
			uint32_t exposure;
			double gain;
			double quantizationGain;
			double yTarget;
		} automatic;

		bool autoExposureEnabled;
		bool autoGainEnabled;
		double exposureValue;
		controls::AeConstraintModeEnum constraintMode;
		controls::AeExposureModeEnum exposureMode;
		controls::AeMeteringModeEnum meteringMode;
		utils::Duration minFrameDuration;
		utils::Duration maxFrameDuration;
	} agc;

	ipa::awb::ActiveState awb;

	ipa::ccm::ActiveState ccm;

	struct {
		float requestedBrightness;
		float actualBrightness;
		BrightnessQ brightness;
		ContrastQ contrast;
		HueQ hue;
		SaturationQ saturation;
	} cproc;

	struct {
		bool denoise;
	} dpf;

	struct {
		uint8_t denoise;
		uint8_t sharpness;
	} filter;

	struct {
		double gamma;
	} goc;

	struct {
		double lux;
	} lux;

	struct {
		controls::WdrModeEnum mode;
		AgcMeanLuminance::AgcConstraint constraint;
		double gain;
		double strength;
	} wdr;

	ipa::lsc::ActiveState lsc;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
		double exposureValue;
		double quantizationGain;
		uint32_t vblank;
		double yTarget;
		bool autoExposureEnabled;
		bool autoGainEnabled;
		controls::AeConstraintModeEnum constraintMode;
		controls::AeExposureModeEnum exposureMode;
		controls::AeMeteringModeEnum meteringMode;
		utils::Duration minFrameDuration;
		utils::Duration maxFrameDuration;
		utils::Duration frameDuration;
		bool updateMetering;
		bool autoExposureModeChange;
		bool autoGainModeChange;
	} agc;

	ipa::awb::FrameContext awb;

	struct {
		float actualBrightness;
		BrightnessQ brightness;
		ContrastQ contrast;
		HueQ hue;
		SaturationQ saturation;

		bool update;
	} cproc;

	struct {
		bool enable;
		double gain;
	} compress;

	struct {
		bool denoise;
		bool update;
	} dpf;

	struct {
		uint8_t denoise;
		uint8_t sharpness;
		bool update;
	} filter;

	struct {
		double gamma;
		bool update;
	} goc;

	struct {
		uint32_t exposure;
		double gain;
	} sensor;

	ipa::ccm::FrameContext ccm;

	struct {
		double lux;
	} lux;

	struct {
		controls::WdrModeEnum mode;
		double strength;
		double gain;
	} wdr;

	ipa::lsc::FrameContext lsc;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPAHwSettings hw;
	IPACameraSensorInfo sensorInfo;
	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;

	DebugMetadata debugMetadata;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/
