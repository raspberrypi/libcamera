/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * cam_helper.h - helper class providing camera information
 */
#pragma once

#include <memory>
#include <string>
#include <utility>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include "camera_mode.h"
#include "controller/controller.h"
#include "controller/metadata.h"
#include "md_parser.h"

#include "libcamera/internal/v4l2_videodevice.h"

namespace RPiController {

/*
 * The CamHelper class provides a number of facilities that anyone trying
 * to drive a camera will need to know, but which are not provided by the
 * standard driver framework. Specifically, it provides:
 *
 * A "CameraMode" structure to describe extra information about the chosen
 * mode of the driver. For example, how it is cropped from the full sensor
 * area, how it is scaled, whether pixels are averaged compared to the full
 * resolution.
 *
 * The ability to convert between number of lines of exposure and actual
 * exposure time, and to convert between the sensor's gain codes and actual
 * gains.
 *
 * A function to return the number of frames of delay between updating exposure,
 * analogue gain and vblanking, and for the changes to take effect. For many
 * sensors these take the values 2, 1 and 2 respectively, but sensors that are
 * different will need to over-ride the default function provided.
 *
 * A function to query if the sensor outputs embedded data that can be parsed.
 *
 * A function to return the sensitivity of a given camera mode.
 *
 * A parser to parse the embedded data buffers provided by some sensors (for
 * example, the imx219 does; the ov5647 doesn't). This allows us to know for
 * sure the exposure and gain of the frame we're looking at. CamHelper
 * provides functions for converting analogue gains to and from the sensor's
 * native gain codes.
 *
 * Finally, a set of functions that determine how to handle the vagaries of
 * different camera modules on start-up or when switching modes. Some
 * modules may produce one or more frames that are not yet correctly exposed,
 * or where the metadata may be suspect. We have the following functions:
 * HideFramesStartup(): Tell the pipeline handler not to return this many
 *     frames at start-up. This can also be used to hide initial frames
 *     while the AGC and other algorithms are sorting themselves out.
 * HideFramesModeSwitch(): Tell the pipeline handler not to return this
 *     many frames after a mode switch (other than start-up). Some sensors
 *     may produce innvalid frames after a mode switch; others may not.
 * MistrustFramesStartup(): At start-up a sensor may return frames for
 *    which we should not run any control algorithms (for example, metadata
 *    may be invalid).
 * MistrustFramesModeSwitch(): The number of frames, after a mode switch
 *    (other than start-up), for which control algorithms should not run
 *    (for example, metadata may be unreliable).
 */

class CamHelper
{
public:
	static CamHelper *create(std::string const &camName);
	CamHelper(std::unique_ptr<MdParser> parser, unsigned int frameIntegrationDiff);
	virtual ~CamHelper();
	void setCameraMode(const CameraMode &mode);
	virtual void prepare(libcamera::Span<const uint8_t> buffer,
			     Metadata &metadata);
	virtual void process(StatisticsPtr &stats, Metadata &metadata);
	virtual uint32_t exposureLines(const libcamera::utils::Duration exposure,
				       const libcamera::utils::Duration lineLength) const;
	virtual libcamera::utils::Duration exposure(uint32_t exposureLines,
						    const libcamera::utils::Duration lineLength) const;
	virtual std::pair<uint32_t, uint32_t> getBlanking(libcamera::utils::Duration &exposure,
							  libcamera::utils::Duration minFrameDuration,
							  libcamera::utils::Duration maxFrameDuration) const;
	libcamera::utils::Duration hblankToLineLength(uint32_t hblank) const;
	uint32_t lineLengthToHblank(const libcamera::utils::Duration &duration) const;
	libcamera::utils::Duration lineLengthPckToDuration(uint32_t lineLengthPck) const;
	virtual uint32_t gainCode(double gain) const = 0;
	virtual double gain(uint32_t gainCode) const = 0;
	virtual void getDelays(int &exposureDelay, int &gainDelay,
			       int &vblankDelay, int &hblankDelay) const;
	virtual bool sensorEmbeddedDataPresent() const;
	virtual double getModeSensitivity(const CameraMode &mode) const;
	virtual unsigned int hideFramesStartup() const;
	virtual unsigned int hideFramesModeSwitch() const;
	virtual unsigned int mistrustFramesStartup() const;
	virtual unsigned int mistrustFramesModeSwitch() const;

protected:
	void parseEmbeddedData(libcamera::Span<const uint8_t> buffer,
			       Metadata &metadata);
	virtual void populateMetadata(const MdParser::RegisterMap &registers,
				      Metadata &metadata) const;

	std::unique_ptr<MdParser> parser_;
	CameraMode mode_;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	unsigned int frameIntegrationDiff_;
};

/*
 * This is for registering camera helpers with the system, so that the
 * CamHelper::Create function picks them up automatically.
 */

typedef CamHelper *(*CamHelperCreateFunc)();
struct RegisterCamHelper
{
	RegisterCamHelper(char const *camName,
			  CamHelperCreateFunc createFunc);
};

} /* namespace RPi */
