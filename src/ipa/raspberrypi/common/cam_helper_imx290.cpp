/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * cam_helper_imx290.cpp - camera helper for imx290 sensor
 */

#include <math.h>

#include "cam_helper.hpp"

using namespace RPiController;

class CamHelperImx290 : public CamHelper
{
public:
	CamHelperImx290();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	void GetDelays(int &exposure_delay, int &gain_delay,
		       int &vblank_delay) const override;
	unsigned int HideFramesModeSwitch() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 2;
};

CamHelperImx290::CamHelperImx290()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx290::GainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx290::Gain(uint32_t gain_code) const
{
	return pow(10, 0.015 * gain_code);
}

void CamHelperImx290::GetDelays(int &exposure_delay, int &gain_delay,
				int &vblank_delay) const
{
	exposure_delay = 2;
	gain_delay = 2;
	vblank_delay = 2;
}

unsigned int CamHelperImx290::HideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}

static CamHelper *Create()
{
	return new CamHelperImx290();
}

static RegisterCamHelper reg("imx290", &Create);
