/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, Will Whang
 *
 * cam_helper_imx678.cpp - camera information for Sony IMX678 sensor
 */

#include <algorithm>
#include <cmath>

#include "cam_helper.h"

using namespace RPiController;

class CamHelperImx678 : public CamHelper
{
public:
	CamHelperImx678();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

/*
 * IMX678 driver currently doesn't expose a metadata stream, so we have to use
 * the "unicam parser" which works by counting frames.
 */

CamHelperImx678::CamHelperImx678()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx678::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx678::gain(uint32_t gainCode) const
{
	return std::pow(10, 0.015 * gainCode);
}

static CamHelper *create()
{
	return new CamHelperImx678();
}

static RegisterCamHelper reg("imx678", &create);
