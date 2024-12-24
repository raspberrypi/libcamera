/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Red Hat, Inc.
 *
 * Simple pipeline IPA Context
 */

#pragma once

#include <array>
#include <optional>
#include <stdint.h>

#include <libcamera/controls.h>

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::soft {

struct IPASessionConfiguration {
	float gamma;
	struct {
		int32_t exposureMin, exposureMax;
		double againMin, againMax, againMinStep;
	} agc;
	struct {
		std::optional<uint8_t> level;
	} black;
};

struct IPAActiveState {
	struct {
		uint8_t level;
	} blc;

	struct {
		double red;
		double green;
		double blue;
	} gains;

	static constexpr unsigned int kGammaLookupSize = 1024;
	struct {
		std::array<double, kGammaLookupSize> gammaTable;
		uint8_t blackLevel;
		double contrast;
	} gamma;
	struct {
		/* 0..2 range, 1.0 = normal */
		std::optional<double> contrast;
	} knobs;
};

struct IPAFrameContext : public FrameContext {
	struct {
		int32_t exposure;
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

} /* namespace ipa::soft */

} /* namespace libcamera */
