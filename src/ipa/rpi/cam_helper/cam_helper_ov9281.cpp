/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * camera information for ov9281 sensor
 */

#include <assert.h>

#include "cam_helper.h"

using namespace RPiController;

class CamHelperOv9281 : public CamHelper
{
public:
	CamHelperOv9281();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 25;
};

/*
 * OV9281 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperOv9281::CamHelperOv9281()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperOv9281::gainCode(double gain) const
{
	return static_cast<uint32_t>(gain * 16.0);
}

double CamHelperOv9281::gain(uint32_t gainCode) const
{
	return static_cast<double>(gainCode) / 16.0;
}

static CamHelper *create()
{
	return new CamHelperOv9281();
}

static RegisterCamHelper reg("ov9281", &create);
