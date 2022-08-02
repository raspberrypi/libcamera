/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * cam_helper.cpp - helper information for different sensors
 */

#include <linux/videodev2.h>

#include <assert.h>
#include <map>
#include <string.h>

#include "libcamera/internal/v4l2_videodevice.h"

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

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
	: parser_(std::move(parser)), initialized_(false),
	  frameIntegrationDiff_(frameIntegrationDiff)
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

uint32_t CamHelper::exposureLines(const Duration exposure) const
{
	assert(initialized_);
	return exposure / mode_.lineLength;
}

Duration CamHelper::exposure(uint32_t exposureLines) const
{
	assert(initialized_);
	return exposureLines * mode_.lineLength;
}

uint32_t CamHelper::getVBlanking(Duration &exposure,
				 Duration minFrameDuration,
				 Duration maxFrameDuration) const
{
	uint32_t frameLengthMin, frameLengthMax, vblank;
	uint32_t exposureLines = CamHelper::exposureLines(exposure);

	assert(initialized_);

	/*
	 * minFrameDuration and maxFrameDuration are clamped by the caller
	 * based on the limits for the active sensor mode.
	 */
	frameLengthMin = minFrameDuration / mode_.lineLength;
	frameLengthMax = maxFrameDuration / mode_.lineLength;

	/*
	 * Limit the exposure to the maximum frame duration requested, and
	 * re-calculate if it has been clipped.
	 */
	exposureLines = std::min(frameLengthMax - frameIntegrationDiff_, exposureLines);
	exposure = CamHelper::exposure(exposureLines);

	/* Limit the vblank to the range allowed by the frame length limits. */
	vblank = std::clamp(exposureLines + frameIntegrationDiff_,
			    frameLengthMin, frameLengthMax) - mode_.height;
	return vblank;
}

void CamHelper::setCameraMode(const CameraMode &mode)
{
	mode_ = mode;
	if (parser_) {
		parser_->reset();
		parser_->setBitsPerPixel(mode.bitdepth);
		parser_->setLineLengthBytes(0); /* We use SetBufferSize. */
	}
	initialized_ = true;
}

void CamHelper::getDelays(int &exposureDelay, int &gainDelay,
			  int &vblankDelay) const
{
	/*
	 * These values are correct for many sensors. Other sensors will
	 * need to over-ride this function.
	 */
	exposureDelay = 2;
	gainDelay = 1;
	vblankDelay = 2;
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
	 * Overwrite the exposure/gain, frame length and sensor temperature values
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
