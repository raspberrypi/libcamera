/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx585.cpp - camera information for Imx585 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx585 : public CamHelper
{
public:
	CamHelperImx585();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	void getDelays(int &exposureDelay, int &gainDelay,
		       int &vblankDelay, int &hblankDelay) const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

/*
 * Imx585 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx585::CamHelperImx585()
	: CamHelper({}, frameIntegrationDiff)
{
}


uint32_t CamHelperImx585::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx585::gain(uint32_t gainCode) const
{
	return pow(10, 0.015 * gainCode);
}



void CamHelperImx585::getDelays(int &exposureDelay, int &gainDelay,
				int &vblankDelay, int &hblankDelay) const
{
	/* The driver appears to behave as follows: */
	exposureDelay = 2;
	gainDelay = 2;
	vblankDelay = 2;
	hblankDelay = 2;
}

unsigned int CamHelperImx585::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}


static CamHelper *create()
{
	return new CamHelperImx585();
}

static RegisterCamHelper reg("imx585", &create);


/*

void CamHelperImx585::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	DeviceStatus deviceStatus;

	if (metadata.get("device.status", deviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found from DelayedControls";
		return;
	}
	//printf(":=================prepare:=================\n");

	uint8_t registers[140];
	for (int i=0;i<200;i++){
		//printf("%02X\n",buffer[i]);
		if ( (i+1) % 3 != 0){
			registers[i - i/3] = buffer[i];
		}
	}
	
	printf("\n");
	printf("====================RESULT\n");
	for (int i=0;i<15;i++){
		printf("[%d]==%02X\n",i,registers[i]);
	}
	printf("\n");
	
	deviceStatus.lineLength = lineLengthPckToDuration(registers[0] * 256 + registers[1]);
	
	uint16_t shr = registers[10] * 256 + registers[9];
	uint16_t svr = registers[12] * 256 + registers[11];

	printf("SHR:%d, SVR:%d\n",shr,svr);
	deviceStatus.shutterSpeed = lineLengthPckToDuration(registers[0] * 256 + registers[1]);


	deviceStatus.analogueGain = gain(registers[8] * 256 + registers[7]);
	printf("Gain:%d -> %f\n",registers[8] * 256 + registers[7],gain(registers[8] * 256 + registers[7]));
	//deviceStatus.frameLength = buffer[12] * 256 + buffer[13];
	//deviceStatus.sensorTemperature = std::clamp<int8_t>(registers.at(temperatureReg), -20, 80);

	metadata.set("device.status", deviceStatus);

	LOG(IPARPI, Debug) << "Metadata updated: "
				   << deviceStatus;
}

*/