/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, Raspberry Pi Ltd
 *
 * camera helper for imx355 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "cam_helper.h"

using namespace RPiController;

class CamHelperImx355 : public CamHelper
{
public:
	CamHelperImx355();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	unsigned int mistrustMetadataModeSwitch() const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

CamHelperImx355::CamHelperImx355()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx355::gainCode(double gain) const
{
	return (uint32_t)(1024 - 1024 / gain);
}

double CamHelperImx355::gain(uint32_t gainCode) const
{
	return 1024.0 / (1024 - gainCode);
}

unsigned int CamHelperImx355::mistrustMetadataModeSwitch() const
{
	/*
	 * For reasons unknown, we do occasionally get a bogus metadata frame
	 * at a mode switch (though not at start-up). Possibly warrants some
	 * investigation, though not a big deal.
	 */
	return 1;
}

bool CamHelperImx355::sensorEmbeddedDataPresent() const
{
	return 0;
}

static CamHelper *create()
{
	return new CamHelperImx355();
}

static RegisterCamHelper reg("imx355", &create);
