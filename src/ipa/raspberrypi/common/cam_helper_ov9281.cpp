/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * cam_helper_ov9281.cpp - camera information for ov9281 sensor
 */

#include <assert.h>

#include "cam_helper.hpp"

using namespace RPiController;

class CamHelperOv9281 : public CamHelper
{
public:
	CamHelperOv9281();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	void GetDelays(int &exposure_delay, int &gain_delay,
		       int &vblank_delay) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

/*
 * OV9281 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperOv9281::CamHelperOv9281()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperOv9281::GainCode(double gain) const
{
	return static_cast<uint32_t>(gain * 16.0);
}

double CamHelperOv9281::Gain(uint32_t gain_code) const
{
	return static_cast<double>(gain_code) / 16.0;
}

void CamHelperOv9281::GetDelays(int &exposure_delay, int &gain_delay,
				int &vblank_delay) const
{
	/* The driver appears to behave as follows: */
	exposure_delay = 2;
	gain_delay = 2;
	vblank_delay = 2;
}

static CamHelper *Create()
{
	return new CamHelperOv9281();
}

static RegisterCamHelper reg("ov9281", &Create);
