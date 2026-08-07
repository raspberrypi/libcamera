/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * Camera helper for IMX296 sensor
 */

#include <algorithm>
#include <cmath>
#include <string.h>
#include <stddef.h>
#include <vector>

#include "cam_helper.h"

using namespace RPiController;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

constexpr uint32_t gainReg = 0x41ba;
constexpr uint32_t shsLoReg = 0x41b4;
constexpr uint32_t shsMidReg = 0x41b5;
constexpr uint32_t shsHiReg = 0x41b6;
constexpr uint32_t vmaxLoReg = 0x41cc;
constexpr uint32_t vmaxMidReg = 0x41cd;
constexpr uint32_t vmaxHiReg = 0x41ce;
constexpr uint32_t tempReg = 0x41da;
constexpr std::initializer_list<uint32_t> registerList =
	{ gainReg, shsLoReg, shsMidReg, shsHiReg, vmaxLoReg, vmaxMidReg, vmaxHiReg, tempReg };

class CamHelperImx296 : public CamHelper
{
public:
	CamHelperImx296();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	uint32_t exposureLines(const Duration exposure, const Duration lineLength) const override;
	Duration exposure(uint32_t exposureLines, const Duration lineLength) const override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	bool sensorEmbeddedDataPresent() const override;
	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;

private:
	static constexpr uint32_t minExposureLines = 1;
	static constexpr uint32_t maxGainCode = 239;
	static constexpr Duration timePerLine = 550.0 / 37.125e6 * 1.0s;

	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;

	std::vector<uint8_t> lastEmbeddedBuffer_;
};

CamHelperImx296::CamHelperImx296()
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
{
}

uint32_t CamHelperImx296::gainCode(double gain) const
{
	uint32_t code = 20 * std::log10(gain) * 10;
	return std::min(code, maxGainCode);
}

double CamHelperImx296::gain(uint32_t gainCode) const
{
	return std::pow(10.0, gainCode / 200.0);
}

uint32_t CamHelperImx296::exposureLines(const Duration exposure,
					[[maybe_unused]] const Duration lineLength) const
{
	return std::max<uint32_t>(minExposureLines, (exposure - 14.26us) / timePerLine);
}

Duration CamHelperImx296::exposure(uint32_t exposureLines,
				   [[maybe_unused]] const Duration lineLength) const
{
	return std::max<uint32_t>(minExposureLines, exposureLines) * timePerLine + 14.26us;
}

void CamHelperImx296::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	/*
	 * The imx296 embedded data is ahead by a single frame, i.e. embedded
	 * data in frame N corresponds to the image data in frame N+1. So make
	 * a copy of the embedded data buffer and use it as normal for the next
	 * frame.
	 */
	CamHelper::prepare({ lastEmbeddedBuffer_.data(), lastEmbeddedBuffer_.size() },
			   metadata);

	if (lastEmbeddedBuffer_.size() != buffer.size())
		lastEmbeddedBuffer_.resize(buffer.size());

	memcpy(lastEmbeddedBuffer_.data(), buffer.data(), buffer.size());
}

bool CamHelperImx296::sensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperImx296::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	uint32_t shs = registers.at(shsLoReg) + (registers.at(shsMidReg) << 8) +
		       (registers.at(shsHiReg) << 16);
	uint32_t vmax = registers.at(vmaxLoReg) + (registers.at(vmaxMidReg) << 8) +
			(registers.at(vmaxHiReg) << 16);

	DeviceStatus deviceStatus;
	deviceStatus.lineLength = timePerLine;
	deviceStatus.frameLength = vmax;
	deviceStatus.exposureTime = exposure(vmax - shs, timePerLine);
	deviceStatus.analogueGain = gain(registers.at(gainReg));

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	return new CamHelperImx296();
}

static RegisterCamHelper reg("imx296", &create);
