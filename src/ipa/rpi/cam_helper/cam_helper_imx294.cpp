/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx294.cpp - camera information for Imx294 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx294 : public CamHelper
{
public:
	CamHelperImx294();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	void getDelays(int &exposureDelay, int &gainDelay,
		       int &vblankDelay, int &hblankDelay) const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

/*
 * Imx294 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx294::CamHelperImx294()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx294::gainCode(double gain) const
{
	return static_cast<uint32_t>(2048.0 - 2048.0/gain);
}

double CamHelperImx294::gain(uint32_t gainCode) const
{
	return static_cast<double>(2048.0/(2048 - gainCode));
}

void CamHelperImx294::getDelays(int &exposureDelay, int &gainDelay,
				int &vblankDelay, int &hblankDelay) const
{
	/* The driver appears to behave as follows: */
	exposureDelay = 2;
	gainDelay = 2;
	vblankDelay = 2;
	hblankDelay = 2;
}

unsigned int CamHelperImx294::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}


static CamHelper *create()
{
	return new CamHelperImx294();
}

static RegisterCamHelper reg("imx294", &create);

