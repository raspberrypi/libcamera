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

#include <libipa/camera_sensor_helper.h>
#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::rkisp1 {

struct IPAHwSettings {
	unsigned int numAeCells;
	unsigned int numHistogramBins;
	unsigned int numHistogramWeights;
	unsigned int numGammaOutSamples;
	bool compand;
};

struct IPASessionConfiguration {
	struct {
		struct rkisp1_cif_isp_window measureWindow;
	} agc;

	struct {
		struct rkisp1_cif_isp_window measureWindow;
		bool enabled;
	} awb;

	struct {
		bool enabled;
	} lsc;

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
		} automatic;

		bool autoExposureEnabled;
		bool autoGainEnabled;
		controls::AeConstraintModeEnum constraintMode;
		controls::AeExposureModeEnum exposureMode;
		controls::AeMeteringModeEnum meteringMode;
		utils::Duration minFrameDuration;
		utils::Duration maxFrameDuration;
	} agc;

	struct {
		struct AwbState {
			RGB<double> gains;
			unsigned int temperatureK;
		};

		AwbState manual;
		AwbState automatic;

		bool autoEnabled;
	} awb;

	struct {
		Matrix<float, 3, 3> manual;
		Matrix<float, 3, 3> automatic;
	} ccm;

	struct {
		int8_t brightness;
		uint8_t contrast;
		uint8_t saturation;
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
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
		uint32_t vblank;
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

	struct {
		RGB<double> gains;
		bool autoEnabled;
		unsigned int temperatureK;
	} awb;

	struct {
		int8_t brightness;
		uint8_t contrast;
		uint8_t saturation;
		bool update;
	} cproc;

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

	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;

	struct {
		double lux;
	} lux;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: hw(nullptr), frameContexts(frameContextSize)
	{
	}

	const IPAHwSettings *hw;
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
