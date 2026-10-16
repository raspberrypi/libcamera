/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Luminance mean sample value based AGC algorithm
 */

#pragma once

#include <array>
#include <stdint.h>

namespace libcamera {

namespace ipa {

class Histogram;

class AgcMSV
{
public:
	struct Limits {
		std::array<uint32_t, 2> exposure;
		std::array<double, 2> gain;
		double gainMinStep;
		double gain1;
	};

	struct Params {
		const Histogram &yHist;
		uint32_t exposure;
		double gain;
	};

	struct Result {
		uint32_t exposure;
		double analogueGain;
	};

	void setLimits(const Limits &limits);
	[[nodiscard]] Result calculateNewEv(const Params &params);

private:
	AgcMSV::Result updateExposure(uint32_t exposure, double again, float exposureMSV);

	Limits limits_ = {};
};

} /* namespace ipa */

} /* namespace libcamera */
