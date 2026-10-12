/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, ams-OSRAM
 *
 * Camera helper for Mira220 sensor
 */

#include <assert.h>

#include "cam_helper.h"

namespace {

using namespace RPiController;

class CamHelperMira220 : public CamHelper
{
public:
	CamHelperMira220();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 *
	 * The integration diff is expressed as
	 *    Tframe - 1928 / row_length
	 *
	 * row_length controls the line timings and varies according to the
	 * number of data lanes in use and the D-PHY data rate. Use an
	 * integration diff calculated using the value of 304, which represents
	 * the minimum row_length for a 2 data lanes configuration running at
	 * 1.5Gbps.
	 */
	static constexpr int frameIntegrationDiff = 6;
};

CamHelperMira220::CamHelperMira220()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperMira220::gainCode(double gain) const
{
	return static_cast<uint32_t>(2048.0 - 2048.0 / gain);
}

double CamHelperMira220::gain(uint32_t gainCode) const
{
	return static_cast<double>(2048.0 / (2048 - gainCode));
}

CamHelper *create()
{
	return new CamHelperMira220();
}

RegisterCamHelper reg("mira220", &create);

} /* namespace */
