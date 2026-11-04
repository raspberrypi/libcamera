/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, Will Whang
 *
 * cam_helper_starvis2.cpp - camera information for Sony Starvis 2 family of sensors
 */

#include <algorithm>
#include <cmath>

#include "cam_helper.h"

namespace {

using namespace RPiController;

class CamHelperStarvis2 : public CamHelper
{
public:
	CamHelperStarvis2();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

CamHelperStarvis2::CamHelperStarvis2()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperStarvis2::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperStarvis2::gain(uint32_t gainCode) const
{
	return std::pow(10, 0.015 * gainCode);
}

CamHelper *create()
{
	return new CamHelperStarvis2();
}

RegisterCamHelper reg662("imx662", &create);
RegisterCamHelper reg675("imx675", &create);
RegisterCamHelper reg678("imx678", &create);
RegisterCamHelper reg832("imx832", &create);

} /* namespace */
