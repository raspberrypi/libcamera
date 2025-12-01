/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) STMicroelectronics SA 2025
 *
 * Camera information for vd55g1 sensor
 */

#include <assert.h>

#include "cam_helper.h"

using namespace RPiController;

class CamHelperVd55g1 : public CamHelper
{
public:
	CamHelperVd55g1();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 64;
};

CamHelperVd55g1::CamHelperVd55g1()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperVd55g1::gainCode(double gain) const
{
	return static_cast<uint32_t>(32.0 - 32.0 / gain);
}

double CamHelperVd55g1::gain(uint32_t gainCode) const
{
	return 32.0 / (32.0 - static_cast<double>(gainCode));
}

static CamHelper *create()
{
	return new CamHelperVd55g1();
}

static RegisterCamHelper reg("vd55g1", &create);
