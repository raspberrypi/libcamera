/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Ideas on Board Oy.
 *
 * cam_helper_Imx335.cpp - camera information for Imx335 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx335 : public CamHelper
{
public:
	CamHelperImx335();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 9;
	static constexpr uint32_t maxGainCode = 100;
};

/*
 * Imx335 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx335::CamHelperImx335()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx335::gainCode(double gain) const
{
	/* Validate me:
	 * The value which is 10/3 times the gain is set to register.
	 * (0.3 dB step)
	 * When set to 6 dB: 6 × 10/3 = 20d; GAIN [7:0] = 14h
	 */
	uint32_t code = 10 * std::log10(gain) * 10 / 3;
	return std::min(code, maxGainCode);
}

double CamHelperImx335::gain(uint32_t gainCode) const
{
	return std::pow(10.0, gainCode / (10 * 10/3));
}

unsigned int CamHelperImx335::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}


static CamHelper *create()
{
	return new CamHelperImx335();
}

static RegisterCamHelper reg("imx335", &create);
