/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) STMicroelectronics SA 2025
 *
 * Camera information for vd56g3 sensor
 */

#include <assert.h>

#include "cam_helper.h"

using namespace RPiController;

class CamHelperVd56g3 : public CamHelper
{
public:
	CamHelperVd56g3();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 61;
};

CamHelperVd56g3::CamHelperVd56g3()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperVd56g3::gainCode(double gain) const
{
	return static_cast<uint32_t>(32.0 - 32.0 / gain);
}

double CamHelperVd56g3::gain(uint32_t gainCode) const
{
	return static_cast<double>(32.0 / (32 - gainCode));
}

static CamHelper *create()
{
	return new CamHelperVd56g3();
}

static RegisterCamHelper reg("vd56g3", &create);
