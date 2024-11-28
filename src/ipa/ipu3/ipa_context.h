/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * IPU3 IPA Context
 *
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::ipu3 {

struct IPASessionConfiguration {
	struct {
		ipu3_uapi_grid_config bdsGrid;
		Size bdsOutputSize;
		uint32_t stride;
	} grid;

	struct {
		ipu3_uapi_grid_config afGrid;
	} af;

	struct {
		utils::Duration minExposureTime;
		utils::Duration maxExposureTime;
		double minAnalogueGain;
		double maxAnalogueGain;
	} agc;

	struct {
		int32_t defVBlank;
		utils::Duration lineDuration;
		Size size;
	} sensor;
};

struct IPAActiveState {
	struct {
		uint32_t focus;
		double maxVariance;
		bool stable;
	} af;

	struct {
		uint32_t exposure;
		double gain;
		uint32_t constraintMode;
		uint32_t exposureMode;
	} agc;

	struct {
		struct {
			double red;
			double green;
			double blue;
		} gains;

		double temperatureK;
	} awb;

	struct {
		double gamma;
		struct ipu3_uapi_gamma_corr_lut gammaCorrection;
	} toneMapping;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
	} sensor;
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

} /* namespace ipa::ipu3 */

} /* namespace libcamera*/
