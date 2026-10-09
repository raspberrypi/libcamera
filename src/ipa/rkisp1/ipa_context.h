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

#include "libipa/agc.h"
#include "libipa/awb.h"
#include "libipa/camera_sensor_helper.h"
#include "libipa/ccm.h"
#include "libipa/fc_queue.h"
#include "libipa/fixedpoint.h"
#include "libipa/gamma.h"
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
	struct Agc : agc::Session {
		struct rkisp1_cif_isp_window measureWindow;
	} agc;

	struct RKISP1AwbSession awb;

	struct {
		bool supported;
	} compress;

	struct {
		Size size;
	} sensor;

	bool raw;
	uint32_t paramFormat;
};

struct IPAActiveState {
	struct Agc : agc::ActiveState {
		controls::AeMeteringModeEnum meteringMode;
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

	ipa::gamma::ActiveState gamma;

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
	struct Agc : agc::FrameContext {
		controls::AeMeteringModeEnum meteringMode;
		bool updateMetering;
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

	ipa::gamma::FrameContext gamma;

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

	ControlInfoMap sensorControls;

	DebugMetadata debugMetadata;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/
