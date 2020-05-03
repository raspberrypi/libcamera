/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * cam_helper_ov5647.cpp - camera information for ov5647 sensor
 */

#include <assert.h>

#include "cam_helper.hpp"
#include "md_parser_rpi.hpp"

using namespace RPi;

class CamHelperOv5647 : public CamHelper
{
public:
	CamHelperOv5647();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	void GetDelays(int &exposure_delay, int &gain_delay) const override;
	unsigned int HideFramesModeSwitch() const override;
	unsigned int MistrustFramesStartup() const override;
	unsigned int MistrustFramesModeSwitch() const override;
};

/*
 * OV5647 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperOv5647::CamHelperOv5647()
	: CamHelper(new MdParserRPi())
{
}

uint32_t CamHelperOv5647::GainCode(double gain) const
{
	return static_cast<uint32_t>(gain * 16.0);
}

double CamHelperOv5647::Gain(uint32_t gain_code) const
{
	return static_cast<double>(gain_code) / 16.0;
}

void CamHelperOv5647::GetDelays(int &exposure_delay, int &gain_delay) const
{
	/*
	 * We run this sensor in a mode where the gain delay is bumped up to
	 * 2. It seems to be the only way to make the delays "predictable".
	 */
	exposure_delay = 2;
	gain_delay = 2;
}

unsigned int CamHelperOv5647::HideFramesModeSwitch() const
{
	/*
	 * After a mode switch, we get a couple of under-exposed frames which
	 * we don't want shown.
	 */
	return 2;
}

unsigned int CamHelperOv5647::MistrustFramesStartup() const
{
	/*
	 * First couple of frames are under-exposed and are no good for control
	 * algos.
	 */
	return 2;
}

unsigned int CamHelperOv5647::MistrustFramesModeSwitch() const
{
	/*
	 * First couple of frames are under-exposed even after a simple
	 * mode switch, and are no good for control algos.
	 */
	return 2;
}

static CamHelper *Create()
{
	return new CamHelperOv5647();
}

static RegisterCamHelper reg("ov5647", &Create);
