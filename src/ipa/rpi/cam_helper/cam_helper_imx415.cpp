/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * camera helper for imx415 sensor
 */

#include <cmath>

#include "cam_helper.h"

using namespace RPiController;

class CamHelperImx415 : public CamHelper
{
public:
	CamHelperImx415();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 8;
};

CamHelperImx415::CamHelperImx415()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx415::gainCode(double gain) const
{
	int code = 66.6667 * std::log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx415::gain(uint32_t gainCode) const
{
	return std::pow(10, 0.015 * gainCode);
}

unsigned int CamHelperImx415::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx415::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}

static CamHelper *create()
{
	return new CamHelperImx415();
}

static RegisterCamHelper reg("imx415", &create);
