/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * cam_helper_imx296.cpp - Camera helper for IMX296 sensor
 */

#include <algorithm>
#include <cmath>
#include <stddef.h>

#include "cam_helper.hpp"

using namespace RPiController;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

class CamHelperImx296 : public CamHelper
{
public:
	CamHelperImx296();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	uint32_t ExposureLines(Duration exposure) const override;
	Duration Exposure(uint32_t exposure_lines) const override;

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

uint32_t CamHelperImx296::GainCode(double gain) const
{
	uint32_t code = 20 * std::log10(gain) * 10;
	return std::min(code, maxGainCode);
}

double CamHelperImx296::Gain(uint32_t gain_code) const
{
	return std::pow(10.0, gain_code / 200.0);
}

uint32_t CamHelperImx296::ExposureLines(Duration exposure) const
{
	return (exposure - 14.26us) / timePerLine;
}

Duration CamHelperImx296::Exposure(uint32_t exposure_lines) const
{
	return exposure_lines * timePerLine + 14.26us;
}

static CamHelper *Create()
{
	return new CamHelperImx296();
}

static RegisterCamHelper reg("imx296", &Create);
