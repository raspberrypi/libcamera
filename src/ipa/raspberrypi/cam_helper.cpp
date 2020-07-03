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

using namespace RPi;

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

CamHelper::CamHelper(MdParser *parser)
	: parser_(parser), initialized_(false)
{
}

CamHelper::~CamHelper()
{
	delete parser_;
}

uint32_t CamHelper::ExposureLines(double exposure_us) const
{
	assert(initialized_);
	return exposure_us * 1000.0 / mode_.line_length;
}

double CamHelper::Exposure(uint32_t exposure_lines) const
{
	assert(initialized_);
	return exposure_lines * mode_.line_length / 1000.0;
}

void CamHelper::SetCameraMode(const CameraMode &mode)
{
	mode_ = mode;
	parser_->SetBitsPerPixel(mode.bitdepth);
	parser_->SetLineLengthBytes(0); /* We use SetBufferSize. */
	initialized_ = true;
}

void CamHelper::GetDelays(int &exposure_delay, int &gain_delay) const
{
	/*
	 * These values are correct for many sensors. Other sensors will
	 * need to over-ride this method.
	 */
	exposure_delay = 2;
	gain_delay = 1;
}

bool CamHelper::SensorEmbeddedDataPresent() const
{
	return false;
}

unsigned int CamHelper::HideFramesStartup() const
{
	/*
	 * By default, hide 6 frames completely at start-up while AGC etc. sort
	 * themselves out (converge).
	 */
	return 6;
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

RegisterCamHelper::RegisterCamHelper(char const *cam_name,
				     CamHelperCreateFunc create_func)
{
	cam_helpers[std::string(cam_name)] = create_func;
}
