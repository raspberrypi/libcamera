/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * camera helper for imx290 sensor
 */

#include <cmath>

#include "cam_helper.h"

using namespace RPiController;

class CamHelperImx290 : public CamHelper
{
public:
	CamHelperImx290(unsigned int frameIntegrationDiff = kFrameIntegrationDiff);
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr unsigned int kFrameIntegrationDiff = 2;
};

CamHelperImx290::CamHelperImx290(unsigned int frameIntegrationDiff)
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx290::gainCode(double gain) const
{
	int code = 66.6667 * std::log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx290::gain(uint32_t gainCode) const
{
	return std::pow(10, 0.015 * gainCode);
}

unsigned int CamHelperImx290::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx290::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}

static CamHelper *create()
{
	return new CamHelperImx290();
}

static CamHelper *createImx662()
{
	/*
	 * The imx662 requires a frame integration diff of at least 4,
	 * according to the datasheet, but in practice this didn't prevent
	 * bad black level values in HCG (high conversion gain) mode.
	 * So instead, we go with 6 for "safety".
	 */
	return new CamHelperImx290(6);
}

static RegisterCamHelper reg("imx290", &create);
static RegisterCamHelper reg327("imx327", &create);
static RegisterCamHelper reg462("imx462", &create);
static RegisterCamHelper reg662("imx662", &createImx662);
