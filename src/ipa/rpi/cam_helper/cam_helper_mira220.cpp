/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * cam_helper_mira220.cpp - camera helper for mira220 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * We have observed that the mira220 embedded data stream randomly returns junk
 * register values. Do not rely on embedded data until this has been resolved.
 */
#define ENABLE_EMBEDDED_DATA 0

#include "cam_helper.h"
#if ENABLE_EMBEDDED_DATA
#include "md_parser.h"
#endif

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;
namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}
/*
 * We care about one gain register and a pair of exposure registers. Their I2C
 * addresses from the mira220 datasheet:
 */
constexpr uint32_t gainReg = 0x400A;
constexpr uint32_t expHiReg = 0x1013;
constexpr uint32_t expLoReg = 0x1012;
constexpr uint32_t frameLengthHiReg = 0x1013;
constexpr uint32_t frameLengthLoReg = 0x1012;
constexpr std::initializer_list<uint32_t> registerList [[maybe_unused]]
	= { expHiReg, expLoReg, gainReg, frameLengthHiReg, frameLengthLoReg };

class CamHelperMira220 : public CamHelper
{
public:
	CamHelperMira220();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gain_code) const override;
	// uint32_t exposureLines(const Duration exposure, const Duration lineLength) const override;
	// Duration exposure(uint32_t exposureLines, const Duration lineLength) const override;
	// unsigned int mistrustFramesModeSwitch() const override;
	// bool sensorEmbeddedDataPresent() const override;

private:
	static constexpr uint32_t minExposureLines = 1;
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
       /*
	* ROW_TIME_US = ROW_LENGTH * CLK_IN_PERIOD_NS / 1000
	* MIRA220_ROW_TIME_1600x1400_1000GBS_US=(300*26.04/1000)=7.8us
	* MIRA220_ROW_TIME_640x480_1000GBS_US=(450*26.04/1000)=11.7us
	*/
	// static constexpr Duration timePerLine_1600x1400_1000gbs = (7.8 / 1.0e6) * 1.0s;
	// static constexpr Duration timePerLine_640x480_1000gbs = (11.7 / 1.0e6) * 1.0s;
	// void populateMetadata(const MdParser::RegisterMap &registers,
			    //   Metadata &metadata) const override;
};

CamHelperMira220::CamHelperMira220()
#if ENABLE_EMBEDDED_DATA
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
#else
	: CamHelper({}, frameIntegrationDiff)
#endif
{
}

uint32_t CamHelperMira220::gainCode(double gain) const
{
	return (uint32_t)(gain);
}

double CamHelperMira220::gain(uint32_t gainCode) const
{
	return (double)(gainCode);
}

// uint32_t CamHelperMira220::exposureLines(const Duration exposure,
// 					[[maybe_unused]] const Duration lineLength) const
// {
// 	Duration timePerLine;
// 	if (mode_.width == 640) {
// 		timePerLine = timePerLine_640x480_1000gbs;
// 	} else {
// 		timePerLine = timePerLine_1600x1400_1000gbs;
// 	}
// 	return std::max<uint32_t>(minExposureLines, exposure / timePerLine);

// }


// Duration CamHelperMira220::exposure(uint32_t exposureLines,
// 				   [[maybe_unused]] const Duration lineLength) const
// {
// 	Duration timePerLine;
// 	LOG(IPARPI, Warning) << "Philippe cam_helper220.cpp exposurefun " ;

// 	if (mode_.width == 640) {
// 		timePerLine = timePerLine_640x480_1000gbs;
// 	} else {
// 		timePerLine = timePerLine_1600x1400_1000gbs;
// 	}
// 	return std::max<uint32_t>(minExposureLines, exposureLines) * timePerLine;
// }


// unsigned int CamHelperMira220::mistrustFramesModeSwitch() const
// {
// 	/*
// 	 * For reasons unknown, we do occasionally get a bogus metadata frame
// 	 * at a mode switch (though not at start-up). Possibly warrants some
// 	 * investigation, though not a big deal.
// 	 */
// 	return 1;
// }

// bool CamHelperMira220::sensorEmbeddedDataPresent() const
// {
// 	return ENABLE_EMBEDDED_DATA;
// }

// void CamHelperMira220::populateMetadata(const MdParser::RegisterMap &registers,
// 				       Metadata &metadata) const
// {
// 	DeviceStatus deviceStatus;

// 	deviceStatus.shutterSpeed = exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg), deviceStatus.lineLength);
// 	deviceStatus.analogueGain = gain(registers.at(gainReg));
// 	deviceStatus.frameLength = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);
// 	LOG(IPARPI, Warning) << "Philippe cam_helper220.cpp shutterspeed " << deviceStatus.shutterSpeed;
// 	LOG(IPARPI, Warning) << "Philippe cam_helper220.cpp anag " << deviceStatus.analogueGain;
// 	LOG(IPARPI, Warning) << "Philippe cam_helper220.cpp framelen " << deviceStatus.frameLength;

// 	metadata.set("device.status", deviceStatus);
// }

static CamHelper *create()
{
	return new CamHelperMira220();
}

static RegisterCamHelper reg("mira220", &create);

