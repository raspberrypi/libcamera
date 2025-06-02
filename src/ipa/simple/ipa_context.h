/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat, Inc.
 *
 * Simple pipeline IPA Context
 */

#pragma once

#include <array>
#include <optional>
#include <stdint.h>

#include <libcamera/controls.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include <libipa/fc_queue.h>

#include "core_ipa_interface.h"

namespace libcamera {

namespace ipa::soft {

struct IPASessionConfiguration {
	float gamma;
	struct {
		int32_t exposureMin, exposureMax;
		double againMin, againMax, againMinStep;
		utils::Duration lineDuration;
	} agc;
	struct {
		std::optional<uint8_t> level;
	} black;
};

struct IPAActiveState {
	struct {
		uint8_t level;
		int32_t lastExposure;
		double lastGain;
	} blc;

	struct {
		RGB<float> gains;
		unsigned int temperatureK;
	} awb;

	static constexpr unsigned int kGammaLookupSize = 1024;
	struct {
		std::array<double, kGammaLookupSize> gammaTable;
		uint8_t blackLevel;
		double contrast;
	} gamma;

	struct {
		Matrix<float, 3, 3> ccm;
		bool changed;
	} ccm;

	struct {
		/* 0..2 range, 1.0 = normal */
		std::optional<double> contrast;
		std::optional<float> saturation;
	} knobs;
};

struct IPAFrameContext : public FrameContext {
	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;

	struct {
		int32_t exposure;
		double gain;
	} sensor;

	struct {
		double red;
		double blue;
	} gains;

	std::optional<double> contrast;
	std::optional<float> saturation;
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
	bool ccmEnabled = false;
};

} /* namespace ipa::soft */

} /* namespace libcamera */
