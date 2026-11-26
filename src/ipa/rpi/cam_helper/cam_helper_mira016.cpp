/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, ams-OSRAM
 * Copyright (C) 2026, Ideas on Board Oy
 *
 * Camera helper for Mira016 sensor
 */

#include <assert.h>
#include <cmath>

#include "cam_helper.h"

namespace {

using namespace RPiController;

class CamHelperMira016 : public CamHelper
{
public:
	CamHelperMira016();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t code) const override;

private:
	/*
	 * The sensor manual doesn't really specify a minimum difference between
	 * the exposure time and the frame duration, so pick a safe value of
	 * 4 frames. The mira016 driver uses the same value as the exposure
	 * margin.
	 */
	static constexpr int frameIntegrationDiff = 4;
	static constexpr unsigned int kAdcRampStep = 5700;
};

CamHelperMira016::CamHelperMira016()
	: CamHelper({}, frameIntegrationDiff)
{
}

/*
 * The driver's analogue gain code is the gain in thousandths (1000 is 1x), up
 * to 16000 in 8-bit modes, 4000 in 10-bit modes and only 1000 in 12-bit modes.
 * The driver rounds it to the nearest gain the fine gain registers give:
 *
 *   gain = 57 * 2^n / (ADC_GAIN + 21)
 *
 * n is the octave of the gain (0 to 4) and ADC_GAIN is 21 to 63. Up to 4x the
 * 8 and 10-bit modes give the same gains, so the conversion does not depend on
 * the mode, which matters because the IPA converts the gain limits of a new
 * mode before it passes that mode to the helper.
 */
uint32_t CamHelperMira016::gainCode(double gain) const
{
	return std::max<long>(std::lround(gain * 1000), 1000);
}

double CamHelperMira016::gain(uint32_t gainCode) const
{
	uint32_t code = std::clamp<uint32_t>(gainCode, 1000, 16000);
	uint32_t n = std::floor(std::log2(84.0 * static_cast<double>(code) /
					  kAdcRampStep));

	uint32_t num = kAdcRampStep << n;
	int adcGain = ((num + code / 2) / code) - 21;
	adcGain = std::clamp(adcGain, 0, 63);

	return static_cast<double>(num) / (adcGain + 21) / 1000.0;
}

CamHelper *create()
{
	return new CamHelperMira016();
}

RegisterCamHelper reg("mira016", &create);

} /* namespace */
