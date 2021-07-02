/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * cam_helper_imx477.cpp - camera helper for imx477 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "cam_helper.hpp"
#include "md_parser.hpp"

using namespace RPiController;

/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony IMX477 datasheet:
 */
constexpr uint32_t expHiReg = 0x0202;
constexpr uint32_t expLoReg = 0x0203;
constexpr uint32_t gainHiReg = 0x0204;
constexpr uint32_t gainLoReg = 0x0205;
constexpr std::initializer_list<uint32_t> registerList = { expHiReg, expLoReg, gainHiReg, gainLoReg };

class CamHelperImx477 : public CamHelper
{
public:
	CamHelperImx477();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	void GetDelays(int &exposure_delay, int &gain_delay,
		       int &vblank_delay) const override;
	bool SensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 22;

	void PopulateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;
};

CamHelperImx477::CamHelperImx477()
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
{
}

uint32_t CamHelperImx477::GainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CamHelperImx477::Gain(uint32_t gain_code) const
{
	return 1024.0 / (1024 - gain_code);
}

void CamHelperImx477::GetDelays(int &exposure_delay, int &gain_delay,
				int &vblank_delay) const
{
	exposure_delay = 2;
	gain_delay = 2;
	vblank_delay = 3;
}

bool CamHelperImx477::SensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperImx477::PopulateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	deviceStatus.shutter_speed = Exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg));
	deviceStatus.analogue_gain = Gain(registers.at(gainHiReg) * 256 + registers.at(gainLoReg));

	metadata.Set("device.status", deviceStatus);
}

static CamHelper *Create()
{
	return new CamHelperImx477();
}

static RegisterCamHelper reg("imx477", &Create);
