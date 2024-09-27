/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Red Hat, Inc.
 *
 * Simple pipeline IPA Context
 */

#pragma once

#include <array>
#include <stdint.h>

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::soft {

struct IPASessionConfiguration {
	float gamma;
};

struct IPAActiveState {
	struct {
		uint8_t level;
	} blc;

	struct {
		unsigned int red;
		unsigned int green;
		unsigned int blue;
	} gains;

	static constexpr unsigned int kGammaLookupSize = 1024;
	struct {
		std::array<double, kGammaLookupSize> gammaTable;
		uint8_t blackLevel;
	} gamma;
};

struct IPAFrameContext : public FrameContext {
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAActiveState activeState;
	FCQueue<IPAFrameContext> frameContexts;
};

} /* namespace ipa::soft */

} /* namespace libcamera */
