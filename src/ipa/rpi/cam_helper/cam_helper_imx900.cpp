/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 * Copyright (C) 2026, UAB Kurokesu
 *
 * Camera helper for IMX900 sensor
 */

#include <algorithm>
#include <cmath>

#include "cam_helper.h"

using namespace RPiController;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

class CamHelperImx900 : public CamHelper
{
public:
	CamHelperImx900();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	uint32_t exposureLines(const Duration exposure, const Duration lineLength) const override;
	Duration exposure(uint32_t exposureLines, const Duration lineLength) const override;
	unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	static constexpr uint32_t minExposureLines = 1;
	static constexpr uint32_t maxGainCode = 480;
	static constexpr Duration integrationOffset = 1.55us;

	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines. Largest across all readout modes.
	 */
	static constexpr int frameIntegrationDiff = 142;
};

CamHelperImx900::CamHelperImx900()
	: CamHelper(nullptr, frameIntegrationDiff)
{
}

uint32_t CamHelperImx900::gainCode(double gain) const
{
	int code = std::lround(200 * std::log10(gain));
	return std::clamp<int>(code, 0, maxGainCode);
}

double CamHelperImx900::gain(uint32_t gainCode) const
{
	return std::pow(10.0, gainCode / 200.0);
}

uint32_t CamHelperImx900::exposureLines(const Duration exposure,
					const Duration lineLength) const
{
	return std::max<uint32_t>(minExposureLines,
				  (exposure - integrationOffset) / lineLength);
}

Duration CamHelperImx900::exposure(uint32_t exposureLines,
				   const Duration lineLength) const
{
	return std::max<uint32_t>(minExposureLines, exposureLines) * lineLength +
	       integrationOffset;
}

unsigned int CamHelperImx900::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx900::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}

static CamHelper *create()
{
	return new CamHelperImx900();
}

static RegisterCamHelper reg("imx900", &create);
