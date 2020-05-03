/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * cam_helper.hpp - helper class providing camera information
 */
#pragma once

#include <string>

#include "camera_mode.h"
#include "md_parser.hpp"

#include "v4l2_videodevice.h"

namespace RPi {

// The CamHelper class provides a number of facilities that anyone trying
// trying to drive a camera will need to know, but which are not provided by
// by the standard driver framework. Specifically, it provides:
//
// A "CameraMode" structure to describe extra information about the chosen
// mode of the driver. For example, how it is cropped from the full sensor
// area, how it is scaled, whether pixels are averaged compared to the full
// resolution.
//
// The ability to convert between number of lines of exposure and actual
// exposure time, and to convert between the sensor's gain codes and actual
// gains.
//
// A method to return the number of frames of delay between updating exposure
// and analogue gain and the changes taking effect. For many sensors these
// take the values 2 and 1 respectively, but sensors that are different will
// need to over-ride the default method provided.
//
// A method to query if the sensor outputs embedded data that can be parsed.
//
// A parser to parse the metadata buffers provided by some sensors (for
// example, the imx219 does; the ov5647 doesn't). This allows us to know for
// sure the exposure and gain of the frame we're looking at. CamHelper
// provides methods for converting analogue gains to and from the sensor's
// native gain codes.
//
// Finally, a set of methods that determine how to handle the vagaries of
// different camera modules on start-up or when switching modes. Some
// modules may produce one or more frames that are not yet correctly exposed,
// or where the metadata may be suspect. We have the following methods:
// HideFramesStartup(): Tell the pipeline handler not to return this many
//     frames at start-up. This can also be used to hide initial frames
//     while the AGC and other algorithms are sorting themselves out.
// HideFramesModeSwitch(): Tell the pipeline handler not to return this
//     many frames after a mode switch (other than start-up). Some sensors
//     may produce innvalid frames after a mode switch; others may not.
// MistrustFramesStartup(): At start-up a sensor may return frames for
//    which we should not run any control algorithms (for example, metadata
//    may be invalid).
// MistrustFramesModeSwitch(): The number of frames, after a mode switch
//    (other than start-up), for which control algorithms should not run
//    (for example, metadata may be unreliable).

// Bitfield to represent the default orientation of the camera.
typedef int CamTransform;
static constexpr CamTransform CamTransform_IDENTITY = 0;
static constexpr CamTransform CamTransform_HFLIP    = 1;
static constexpr CamTransform CamTransform_VFLIP    = 2;

class CamHelper
{
public:
	static CamHelper *Create(std::string const &cam_name);
	CamHelper(MdParser *parser);
	virtual ~CamHelper();
	void SetCameraMode(const CameraMode &mode);
	MdParser &Parser() const { return *parser_; }
	uint32_t ExposureLines(double exposure_us) const;
	double Exposure(uint32_t exposure_lines) const; // in us
	virtual uint32_t GainCode(double gain) const = 0;
	virtual double Gain(uint32_t gain_code) const = 0;
	virtual void GetDelays(int &exposure_delay, int &gain_delay) const;
	virtual bool SensorEmbeddedDataPresent() const;
	virtual unsigned int HideFramesStartup() const;
	virtual unsigned int HideFramesModeSwitch() const;
	virtual unsigned int MistrustFramesStartup() const;
	virtual unsigned int MistrustFramesModeSwitch() const;
	virtual CamTransform GetOrientation() const;
protected:
	MdParser *parser_;
	CameraMode mode_;
	bool initialized_;
};

// This is for registering camera helpers with the system, so that the
// CamHelper::Create function picks them up automatically.

typedef CamHelper *(*CamHelperCreateFunc)();
struct RegisterCamHelper
{
	RegisterCamHelper(char const *cam_name,
			  CamHelperCreateFunc create_func);
};

} // namespace RPi
