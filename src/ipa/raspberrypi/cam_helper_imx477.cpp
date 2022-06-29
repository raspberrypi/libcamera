/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * cam_helper_imx477.cpp - camera helper for imx477 sensor
 */

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include "cam_helper.hpp"
#include "md_parser.hpp"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony IMX477 datasheet:
 */
constexpr uint32_t expHiReg = 0x0202;
constexpr uint32_t expLoReg = 0x0203;
constexpr uint32_t gainHiReg = 0x0204;
constexpr uint32_t gainLoReg = 0x0205;
constexpr uint32_t frameLengthHiReg = 0x0340;
constexpr uint32_t frameLengthLoReg = 0x0341;
constexpr uint32_t temperatureReg = 0x013a;
constexpr std::initializer_list<uint32_t> registerList =
	{ expHiReg, expLoReg, gainHiReg, gainLoReg, frameLengthHiReg, frameLengthLoReg, temperatureReg  };

class CamHelperImx477 : public CamHelper
{
public:
	CamHelperImx477();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	void Prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	uint32_t GetVBlanking(Duration &exposure, Duration minFrameDuration,
			      Duration maxFrameDuration) const override;
	void GetDelays(int &exposure_delay, int &gain_delay,
		       int &vblank_delay) const override;
	bool SensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 22;
	/* Maximum frame length allowable for long exposure calculations. */
	static constexpr int frameLengthMax = 0xffdc;
	/* Largest long exposure scale factor given as a left shift on the frame length. */
	static constexpr int longExposureShiftMax = 7;

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

void CamHelperImx477::Prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	MdParser::RegisterMap registers;
	DeviceStatus deviceStatus;

	if (metadata.Get("device.status", deviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found from DelayedControls";
		return;
	}

	parseEmbeddedData(buffer, metadata);

	/*
	 * The DeviceStatus struct is first populated with values obtained from
	 * DelayedControls. If this reports frame length is > frameLengthMax,
	 * it means we are using a long exposure mode. Since the long exposure
	 * scale factor is not returned back through embedded data, we must rely
	 * on the existing exposure lines and frame length values returned by
	 * DelayedControls.
	 *
	 * Otherwise, all values are updated with what is reported in the
	 * embedded data.
	 */
	if (deviceStatus.frame_length > frameLengthMax) {
		DeviceStatus parsedDeviceStatus;

		metadata.Get("device.status", parsedDeviceStatus);
		parsedDeviceStatus.shutter_speed = deviceStatus.shutter_speed;
		parsedDeviceStatus.frame_length = deviceStatus.frame_length;
		metadata.Set("device.status", parsedDeviceStatus);

		LOG(IPARPI, Debug) << "Metadata updated for long exposure: "
				   << parsedDeviceStatus;
	}
}

uint32_t CamHelperImx477::GetVBlanking(Duration &exposure,
				       Duration minFrameDuration,
				       Duration maxFrameDuration) const
{
	uint32_t frameLength, exposureLines;
	unsigned int shift = 0;

	frameLength = mode_.height + CamHelper::GetVBlanking(exposure, minFrameDuration,
							     maxFrameDuration);
	/*
	 * Check if the frame length calculated needs to be setup for long
	 * exposure mode. This will require us to use a long exposure scale
	 * factor provided by a shift operation in the sensor.
	 */
	while (frameLength > frameLengthMax) {
		if (++shift > longExposureShiftMax) {
			shift = longExposureShiftMax;
			frameLength = frameLengthMax;
			break;
		}
		frameLength >>= 1;
	}

	if (shift) {
		/* Account for any rounding in the scaled frame length value. */
		frameLength <<= shift;
		exposureLines = ExposureLines(exposure);
		exposureLines = std::min(exposureLines, frameLength - frameIntegrationDiff);
		exposure = Exposure(exposureLines);
	}

	return frameLength - mode_.height;
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
	deviceStatus.frame_length = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);
	deviceStatus.sensor_temperature = std::clamp<int8_t>(registers.at(temperatureReg), -20, 80);

	metadata.Set("device.status", deviceStatus);
}

static CamHelper *Create()
{
	return new CamHelperImx477();
}

static RegisterCamHelper reg("imx477", &Create);
