/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * cam_helper.cpp - helper information for different sensors
 */

#include <linux/videodev2.h>

#include <assert.h>
#include <map>
#include <string.h>

#include "libcamera/internal/v4l2_videodevice.h"

#include "cam_helper.hpp"
#include "md_parser.hpp"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

static std::map<std::string, CamHelperCreateFunc> cam_helpers;

CamHelper *CamHelper::Create(std::string const &cam_name)
{
	/*
	 * CamHelpers get registered by static RegisterCamHelper
	 * initialisers.
	 */
	for (auto &p : cam_helpers) {
		if (cam_name.find(p.first) != std::string::npos)
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

void CamHelper::Prepare(Span<const uint8_t> buffer,
			Metadata &metadata)
{
	parseEmbeddedData(buffer, metadata);
}

void CamHelper::Process([[maybe_unused]] StatisticsPtr &stats,
			[[maybe_unused]] Metadata &metadata)
{
}

uint32_t CamHelper::ExposureLines(const Duration exposure) const
{
	assert(initialized_);
	return exposure / mode_.line_length;
}

Duration CamHelper::Exposure(uint32_t exposure_lines) const
{
	assert(initialized_);
	return exposure_lines * mode_.line_length;
}

uint32_t CamHelper::GetVBlanking(Duration &exposure,
				 Duration minFrameDuration,
				 Duration maxFrameDuration) const
{
	uint32_t frameLengthMin, frameLengthMax, vblank;
	uint32_t exposureLines = ExposureLines(exposure);

	assert(initialized_);

	/*
	 * minFrameDuration and maxFrameDuration are clamped by the caller
	 * based on the limits for the active sensor mode.
	 */
	frameLengthMin = minFrameDuration / mode_.line_length;
	frameLengthMax = maxFrameDuration / mode_.line_length;

	/*
	 * Limit the exposure to the maximum frame duration requested, and
	 * re-calculate if it has been clipped.
	 */
	exposureLines = std::min(frameLengthMax - frameIntegrationDiff_, exposureLines);
	exposure = Exposure(exposureLines);

	/* Limit the vblank to the range allowed by the frame length limits. */
	vblank = std::clamp(exposureLines + frameIntegrationDiff_,
			    frameLengthMin, frameLengthMax) - mode_.height;
	return vblank;
}

void CamHelper::SetCameraMode(const CameraMode &mode)
{
	mode_ = mode;
	if (parser_) {
		parser_->SetBitsPerPixel(mode.bitdepth);
		parser_->SetLineLengthBytes(0); /* We use SetBufferSize. */
	}
	initialized_ = true;
}

void CamHelper::GetDelays(int &exposure_delay, int &gain_delay,
			  int &vblank_delay) const
{
	/*
	 * These values are correct for many sensors. Other sensors will
	 * need to over-ride this function.
	 */
	exposure_delay = 2;
	gain_delay = 1;
	vblank_delay = 2;
}

bool CamHelper::SensorEmbeddedDataPresent() const
{
	return false;
}

double CamHelper::GetModeSensitivity([[maybe_unused]] const CameraMode &mode) const
{
	/*
	 * Most sensors have the same sensitivity in every mode, but this
	 * function can be overridden for those that do not. Note that it is
	 * called before mode_ is set, so it must return the sensitivity
	 * of the mode that is passed in.
	 */
	return 1.0;
}

unsigned int CamHelper::HideFramesStartup() const
{
	/*
	 * The number of frames when a camera first starts that shouldn't be
	 * displayed as they are invalid in some way.
	 */
	return 0;
}

unsigned int CamHelper::HideFramesModeSwitch() const
{
	/* After a mode switch, many sensors return valid frames immediately. */
	return 0;
}

unsigned int CamHelper::MistrustFramesStartup() const
{
	/* Many sensors return a single bad frame on start-up. */
	return 1;
}

unsigned int CamHelper::MistrustFramesModeSwitch() const
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

	if (parser_->Parse(buffer, registers) != MdParser::Status::OK) {
		LOG(IPARPI, Error) << "Embedded data buffer parsing failed";
		return;
	}

	PopulateMetadata(registers, parsedMetadata);
	metadata.Merge(parsedMetadata);

	/*
	 * Overwrite the exposure/gain, frame length and sensor temperature values
	 * in the existing DeviceStatus with values from the parsed embedded buffer.
	 * Fetch it first in case any other fields were set meaningfully.
	 */
	DeviceStatus deviceStatus, parsedDeviceStatus;
	if (metadata.Get("device.status", deviceStatus) ||
	    parsedMetadata.Get("device.status", parsedDeviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found";
		return;
	}

	deviceStatus.shutter_speed = parsedDeviceStatus.shutter_speed;
	deviceStatus.analogue_gain = parsedDeviceStatus.analogue_gain;
	deviceStatus.frame_length = parsedDeviceStatus.frame_length;
	if (parsedDeviceStatus.sensor_temperature)
		deviceStatus.sensor_temperature = parsedDeviceStatus.sensor_temperature;

	LOG(IPARPI, Debug) << "Metadata updated - " << deviceStatus;

	metadata.Set("device.status", deviceStatus);
}

void CamHelper::PopulateMetadata([[maybe_unused]] const MdParser::RegisterMap &registers,
				 [[maybe_unused]] Metadata &metadata) const
{
}

RegisterCamHelper::RegisterCamHelper(char const *cam_name,
				     CamHelperCreateFunc create_func)
{
	cam_helpers[std::string(cam_name)] = create_func;
}
