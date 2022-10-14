/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * cam_helper_imx708.cpp - camera helper for imx708 sensor
 */

#include <cmath>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include <controller/pdaf_data.h>

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony imx708 datasheet:
 */
constexpr uint32_t expHiReg = 0x0202;
constexpr uint32_t expLoReg = 0x0203;
constexpr uint32_t gainHiReg = 0x0204;
constexpr uint32_t gainLoReg = 0x0205;
constexpr uint32_t frameLengthHiReg = 0x0340;
constexpr uint32_t frameLengthLoReg = 0x0341;
constexpr uint32_t lineLengthHiReg = 0x0342;
constexpr uint32_t lineLengthLoReg = 0x0343;
constexpr uint32_t temperatureReg = 0x013a;
constexpr std::initializer_list<uint32_t> registerList = { expHiReg, expLoReg, gainHiReg, gainLoReg, lineLengthHiReg, lineLengthLoReg, frameLengthHiReg, frameLengthLoReg, temperatureReg };

class CamHelperImx708 : public CamHelper
{
public:
	CamHelperImx708();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gain_code) const override;
	void setCameraMode(const CameraMode &mode) override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	void process(StatisticsPtr &stats, Metadata &metadata) override;
	std::pair<uint32_t, uint32_t> getBlanking(Duration &exposure, Duration minFrameDuration,
						  Duration maxFrameDuration) const override;
	void getDelays(int &exposureDelay, int &gainDelay,
		       int &vblankDelay, int &hblankDelay) const override;
	bool sensorEmbeddedDataPresent() const override;
	double getModeSensitivity(const CameraMode &mode) const override;
	unsigned int hideFramesModeSwitch() const override { return 1; } // seems to be required for HDR

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

	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;

	static bool parsePdafData(const uint8_t *ptr, size_t len, unsigned bpp,
				  PdafData &pdaf);

	bool parseAEHist(const uint8_t *ptr, size_t len, unsigned bpp);
	void putAGCStatistics(StatisticsPtr stats);

	uint32_t aeHistLinear_[128];
	uint32_t aeHistAverage_;
	bool aeHistValid_;
};

CamHelperImx708::CamHelperImx708()
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff),
	  aeHistLinear_{ 0 }, aeHistAverage_(0), aeHistValid_(false)
{
}

uint32_t CamHelperImx708::gainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CamHelperImx708::gain(uint32_t gain_code) const
{
	return 1024.0 / (1024 - gain_code);
}

void CamHelperImx708::setCameraMode(const CameraMode &mode)
{
	/*
	 * We expect the current IMX708 driver to append PDAF data to the
	 * embedded data; tell the embedded data parser to ignore it.
	 */
	CamHelper::setCameraMode(mode);
	parser_->setLineLengthBytes((mode_.width * mode_.bitdepth) >> 3);
	parser_->setNumLines(2);
}

void CamHelperImx708::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	MdParser::RegisterMap registers;
	DeviceStatus deviceStatus;

	LOG(IPARPI, Debug) << "Embedded buffer size: " << buffer.size();

	if (metadata.get("device.status", deviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found from DelayedControls";
		return;
	}

	parseEmbeddedData(buffer, metadata);

	/*
	 * Parse PDAF data, which we expect to occupy the third scanline
	 * of embedded data. As PDAF is quite sensor-specific, it's parsed here.
	 */
	size_t bytesPerLine = (mode_.width * mode_.bitdepth) >> 3;

	if (buffer.size() > 2 * bytesPerLine) {
		PdafData pdaf;
		if (parsePdafData(&buffer[2 * bytesPerLine],
				  buffer.size() - 2 * bytesPerLine,
				  mode_.bitdepth, pdaf))
			metadata.set("pdaf.data", pdaf);
	}

	/* Parse AE-HIST data where present */
	if (buffer.size() > 3 * bytesPerLine) {
		aeHistValid_ = parseAEHist(&buffer[3 * bytesPerLine],
					    buffer.size() - 3 * bytesPerLine,
					    mode_.bitdepth);
	}

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
	if (deviceStatus.frameLength > frameLengthMax) {
		DeviceStatus parsedDeviceStatus;

		metadata.get("device.status", parsedDeviceStatus);
		parsedDeviceStatus.shutterSpeed = deviceStatus.shutterSpeed;
		parsedDeviceStatus.frameLength = deviceStatus.frameLength;
		metadata.set("device.status", parsedDeviceStatus);

		LOG(IPARPI, Debug) << "Metadata updated for long exposure: "
				   << parsedDeviceStatus;
	}
}

void CamHelperImx708::process(StatisticsPtr &stats, [[maybe_unused]] Metadata &metadata)
{
	if (aeHistValid_)
		putAGCStatistics(stats);
}

std::pair<uint32_t, uint32_t> CamHelperImx708::getBlanking(Duration &exposure,
							   Duration minFrameDuration,
							   Duration maxFrameDuration) const
{
	uint32_t frameLength, exposureLines;
	unsigned int shift = 0;

	auto [vblank, hblank] = CamHelper::getBlanking(exposure, minFrameDuration,
						       maxFrameDuration);

	frameLength = mode_.height + vblank;
	Duration lineLength = hblankToLineLength(hblank);

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
		exposureLines = CamHelper::exposureLines(exposure, lineLength);
		exposureLines = std::min(exposureLines, frameLength - frameIntegrationDiff);
		exposure = CamHelper::exposure(exposureLines, lineLength);
	}

	return { frameLength - mode_.height, hblank };
}

void CamHelperImx708::getDelays(int &exposureDelay, int &gainDelay,
				int &vblankDelay, int &hblankDelay) const
{
	exposureDelay = 2;
	gainDelay = 2;
	vblankDelay = 3;
	hblankDelay = 3;
}

bool CamHelperImx708::sensorEmbeddedDataPresent() const
{
	return true;
}

double CamHelperImx708::getModeSensitivity(const CameraMode &mode) const
{
	/* In binned modes, sensitivity increases by a factor of 2 */
	return (mode.width > 2304) ? 1.0 : 2.0;
}

void CamHelperImx708::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(lineLengthHiReg) * 256 +
							  registers.at(lineLengthLoReg));
	deviceStatus.shutterSpeed = exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg),
					     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(gainHiReg) * 256 + registers.at(gainLoReg));
	deviceStatus.frameLength = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);
	deviceStatus.sensorTemperature = std::clamp<int8_t>(registers.at(temperatureReg), -20, 80);

	metadata.set("device.status", deviceStatus);
}

bool CamHelperImx708::parsePdafData(const uint8_t *ptr, size_t len,
				    unsigned bpp, PdafData &pdaf)
{
	size_t step = bpp >> 1; /* bytes per PDAF grid entry */

	if (bpp < 10 || bpp > 12 || len < 194 * step || ptr[0] != 0 || ptr[1] >= 0x40) {
		LOG(IPARPI, Error) << "PDAF data in unsupported format";
		return false;
	}

	ptr += 2 * step;
	for (unsigned i = 0; i < PDAF_DATA_ROWS; ++i) {
		for (unsigned j = 0; j < PDAF_DATA_COLS; ++j) {
			unsigned c = (ptr[0] << 3) | (ptr[1] >> 5);
			int p = (((ptr[1] & 0x0F) - (ptr[1] & 0x10)) << 6) | (ptr[2] >> 2);
			pdaf.conf[i][j] = c;
			pdaf.phase[i][j] = c ? p : 0;
			ptr += step;
		}
	}

	return true;
}

bool CamHelperImx708::parseAEHist(const uint8_t *ptr, size_t len, unsigned bpp)
{
	static const uint32_t ISP_PIPELINE_BITS = 13;
	uint64_t count = 0, sum = 0;
	size_t step = bpp >> 1; /* bytes per histogram bin */

	if (len < 144 * step)
		return false;

	/*
	 * Read the 128 bin linear histogram, which by default covers
	 * the full range of the HDR shortest exposure (small values are
	 * expected to dominate, so pixel-value resolution will be poor).
	 */
	for (unsigned i = 0; i < 128; ++i) {
		if (ptr[3] != 0x55)
			return false;
		uint32_t c = (ptr[0] << 14) + (ptr[1] << 6) + (ptr[2] >> 2);
		aeHistLinear_[i] = c >> 2; /* pixels to quads */
		if (i != 0) {
			count += c;
			sum += c *
			       (i * (1u << (ISP_PIPELINE_BITS - 7)) +
				(1u << (ISP_PIPELINE_BITS - 8)));
		}
		ptr += step;
	}

	/*
	 * Now use the first 9 bins of the log histogram (these should be
	 * subdivisions of the smallest linear bin), to get a more accurate
	 * average value. Don't assume that AEHIST1_AVERAGE is present.
	 */
	for (unsigned i = 0; i < 9; ++i) {
		if (ptr[3] != 0x55)
			return false;
		uint32_t c = (ptr[0] << 14) + (ptr[1] << 6) + (ptr[2] >> 2);
		count += c;
		sum += c *
		       ((3u << ISP_PIPELINE_BITS) >> (17 - i));
		ptr += step;
	}
	if ((unsigned)((ptr[0] << 12) + (ptr[1] << 4) + (ptr[2] >> 4)) !=
	    aeHistLinear_[1]) {
		LOG(IPARPI, Error) << "Lin/Log histogram mismatch";
		return false;
	}

	aeHistAverage_ = count ? (sum / count) : 0;

	return count != 0;
}

void CamHelperImx708::putAGCStatistics(StatisticsPtr stats)
{
	/*
	 * For HDR mode, copy sensor's AE/AGC statistics over ISP's, so the
	 * AGC algorithm sees a linear response to exposure and gain changes.
	 *
	 * Histogram: Just copy the "raw" histogram over the tone-mapped one,
	 * although they have different distributions (raw values are lower).
	 * Tuning should either ignore it, or constrain for highlights only.
	 *
	 * Average: Overwrite all regional averages with a global raw average,
	 * scaled by a fiddle-factor so that a conventional (non-HDR) y_target
	 * of e.g. 0.17 will map to a suitable level for HDR.
	 */
	memcpy(stats->hist[0].g_hist, aeHistLinear_, sizeof(stats->hist[0].g_hist));

	static const unsigned HDR_HEADROOM_FACTOR = 4;
	uint64_t v = HDR_HEADROOM_FACTOR * aeHistAverage_;
	for (int i = 0; i < AGC_REGIONS; i++) {
		struct bcm2835_isp_stats_region &r = stats->agc_stats[i];
		r.r_sum = r.b_sum = r.g_sum = r.counted * v;
	}
}

static CamHelper *create()
{
	return new CamHelperImx708();
}

static RegisterCamHelper reg("imx708", &create);
static RegisterCamHelper regWide("imx708_wide", &create);
static RegisterCamHelper regNoIr("imx708_noir", &create);
static RegisterCamHelper regWideNoIr("imx708_wide_noir", &create);
