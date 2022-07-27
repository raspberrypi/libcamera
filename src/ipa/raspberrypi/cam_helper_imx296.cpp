/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * cam_helper_imx296.cpp - Camera helper for IMX296 sensor
 */

#include <algorithm>
#include <cmath>
#include <stddef.h>

#include "cam_helper.h"

using namespace RPiController;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

class CamHelperImx296 : public CamHelper
{
public:
	CamHelperImx296();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	uint32_t exposureLines(Duration exposure) const override;
	Duration exposure(uint32_t exposureLines) const override;

private:
	static constexpr uint32_t maxGainCode = 239;
	static constexpr Duration timePerLine = 550.0 / 37.125e6 * 1.0s;

	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

CamHelperImx296::CamHelperImx296()
	: CamHelper(nullptr, frameIntegrationDiff)
{
}

uint32_t CamHelperImx296::gainCode(double gain) const
{
	uint32_t code = 20 * std::log10(gain) * 10;
	return std::min(code, maxGainCode);
}

double CamHelperImx296::gain(uint32_t gainCode) const
{
	return std::pow(10.0, gainCode / 200.0);
}

uint32_t CamHelperImx296::exposureLines(Duration exposure) const
{
	return (exposure - 14.26us) / timePerLine;
}

Duration CamHelperImx296::exposure(uint32_t exposureLines) const
{
	return exposureLines * timePerLine + 14.26us;
}

static CamHelper *create()
{
	return new CamHelperImx296();
}

static RegisterCamHelper reg("imx296", &create);
