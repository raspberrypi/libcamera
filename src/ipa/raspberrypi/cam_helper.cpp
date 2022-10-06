/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * cam_helper.cpp - helper information for different sensors
 */

#include <linux/videodev2.h>

#include <limits>
#include <map>
#include <string.h>

#include "libcamera/internal/v4l2_videodevice.h"

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

static std::map<std::string, CamHelperCreateFunc> camHelpers;

CamHelper *CamHelper::create(std::string const &camName)
{
	/*
	 * CamHelpers get registered by static RegisterCamHelper
	 * initialisers.
	 */
	for (auto &p : camHelpers) {
		if (camName.find(p.first) != std::string::npos)
			return p.second();
	}

	return nullptr;
}

CamHelper::CamHelper(std::unique_ptr<MdParser> parser, unsigned int frameIntegrationDiff)
	: parser_(std::move(parser)), frameIntegrationDiff_(frameIntegrationDiff)
{
}

CamHelper::~CamHelper()
{
}

void CamHelper::prepare(Span<const uint8_t> buffer,
			Metadata &metadata)
{
	parseEmbeddedData(buffer, metadata);
}

void CamHelper::process([[maybe_unused]] StatisticsPtr &stats,
			[[maybe_unused]] Metadata &metadata)
{
}

uint32_t CamHelper::exposureLines(const Duration exposure, const Duration lineLength) const
{
	return exposure / lineLength;
}

Duration CamHelper::exposure(uint32_t exposureLines, const Duration lineLength) const
{
	return exposureLines * lineLength;
}

std::pair<uint32_t, uint32_t> CamHelper::getBlanking(Duration &exposure,
						     Duration minFrameDuration,
						     Duration maxFrameDuration) const
{
	uint32_t frameLengthMin, frameLengthMax, vblank, hblank;
	Duration lineLength = mode_.minLineLength;

	/*
	 * minFrameDuration and maxFrameDuration are clamped by the caller
	 * based on the limits for the active sensor mode.
	 *
	 * frameLengthMax gets calculated on the smallest line length as we do
	 * not want to extend that unless absolutely necessary.
	 */
	frameLengthMin = minFrameDuration / mode_.minLineLength;
	frameLengthMax = maxFrameDuration / mode_.minLineLength;

	/*
	 * Watch out for (exposureLines + frameIntegrationDiff_) overflowing a
	 * uint32_t in the std::clamp() below when the exposure time is
	 * extremely (extremely!) long - as happens when the IPA calculates the
	 * maximum possible exposure time.
	 */
	uint32_t exposureLines = std::min(CamHelper::exposureLines(exposure, lineLength),
					  std::numeric_limits<uint32_t>::max() - frameIntegrationDiff_);
	uint32_t frameLengthLines = std::clamp(exposureLines + frameIntegrationDiff_,
					       frameLengthMin, frameLengthMax);

	/*
	 * If our frame length lines is above the maximum allowed, see if we can
	 * extend the line length to accommodate the requested frame length.
	 */
	if (frameLengthLines > mode_.maxFrameLength) {
		Duration lineLengthAdjusted = lineLength * frameLengthLines / mode_.maxFrameLength;
		lineLength = std::min(mode_.maxLineLength, lineLengthAdjusted);
		frameLengthLines = mode_.maxFrameLength;
	}

	hblank = lineLengthToHblank(lineLength);
	vblank = frameLengthLines - mode_.height;

	/*
	 * Limit the exposure to the maximum frame duration requested, and
	 * re-calculate if it has been clipped.
	 */
	exposureLines = std::min(frameLengthLines - frameIntegrationDiff_,
				 CamHelper::exposureLines(exposure, lineLength));
	exposure = CamHelper::exposure(exposureLines, lineLength);

	return { vblank, hblank };
}

Duration CamHelper::hblankToLineLength(uint32_t hblank) const
{
	return (mode_.width + hblank) * (1.0s / mode_.pixelRate);
}

uint32_t CamHelper::lineLengthToHblank(const Duration &lineLength) const
{
	return (lineLength * mode_.pixelRate / 1.0s) - mode_.width;
}

Duration CamHelper::lineLengthPckToDuration(uint32_t lineLengthPck) const
{
	return lineLengthPck * (1.0s / mode_.pixelRate);
}

void CamHelper::setCameraMode(const CameraMode &mode)
{
	mode_ = mode;
	if (parser_) {
		parser_->reset();
		parser_->setBitsPerPixel(mode.bitdepth);
		parser_->setLineLengthBytes(0); /* We use SetBufferSize. */
	}
}

void CamHelper::getDelays(int &exposureDelay, int &gainDelay,
			  int &vblankDelay, int &hblankDelay) const
{
	/*
	 * These values are correct for many sensors. Other sensors will
	 * need to over-ride this function.
	 */
	exposureDelay = 2;
	gainDelay = 1;
	vblankDelay = 2;
	hblankDelay = 2;
}

bool CamHelper::sensorEmbeddedDataPresent() const
{
	return false;
}

double CamHelper::getModeSensitivity([[maybe_unused]] const CameraMode &mode) const
{
	/*
	 * Most sensors have the same sensitivity in every mode, but this
	 * function can be overridden for those that do not. Note that it is
	 * called before mode_ is set, so it must return the sensitivity
	 * of the mode that is passed in.
	 */
	return 1.0;
}

unsigned int CamHelper::hideFramesStartup() const
{
	/*
	 * The number of frames when a camera first starts that shouldn't be
	 * displayed as they are invalid in some way.
	 */
	return 0;
}

unsigned int CamHelper::hideFramesModeSwitch() const
{
	/* After a mode switch, many sensors return valid frames immediately. */
	return 0;
}

unsigned int CamHelper::mistrustFramesStartup() const
{
	/* Many sensors return a single bad frame on start-up. */
	return 1;
}

unsigned int CamHelper::mistrustFramesModeSwitch() const
{
	/* Many sensors return valid metadata immediately. */
	return 0;
}

void CamHelper::parseEmbeddedData(Span<const uint8_t> buffer,
				  Metadata &metadata)
{
	MdParser::RegisterMap registers;
	Metadata parsedMetadata;

	if (buffer.empty())
		return;

	if (parser_->parse(buffer, registers) != MdParser::Status::OK) {
		LOG(IPARPI, Error) << "Embedded data buffer parsing failed";
		return;
	}

	populateMetadata(registers, parsedMetadata);
	metadata.merge(parsedMetadata);

	/*
	 * Overwrite the exposure/gain, line/frame length and sensor temperature values
	 * in the existing DeviceStatus with values from the parsed embedded buffer.
	 * Fetch it first in case any other fields were set meaningfully.
	 */
	DeviceStatus deviceStatus, parsedDeviceStatus;
	if (metadata.get("device.status", deviceStatus) ||
	    parsedMetadata.get("device.status", parsedDeviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found";
		return;
	}

	deviceStatus.shutterSpeed = parsedDeviceStatus.shutterSpeed;
	deviceStatus.analogueGain = parsedDeviceStatus.analogueGain;
	deviceStatus.frameLength = parsedDeviceStatus.frameLength;
	deviceStatus.lineLength = parsedDeviceStatus.lineLength;
	if (parsedDeviceStatus.sensorTemperature)
		deviceStatus.sensorTemperature = parsedDeviceStatus.sensorTemperature;

	LOG(IPARPI, Debug) << "Metadata updated - " << deviceStatus;

	metadata.set("device.status", deviceStatus);
}

void CamHelper::populateMetadata([[maybe_unused]] const MdParser::RegisterMap &registers,
				 [[maybe_unused]] Metadata &metadata) const
{
}

RegisterCamHelper::RegisterCamHelper(char const *camName,
				     CamHelperCreateFunc createFunc)
{
	camHelpers[std::string(camName)] = createFunc;
}
