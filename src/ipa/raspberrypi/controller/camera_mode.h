/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2020, Raspberry Pi Ltd
 *
 * camera_mode.h - description of a particular operating mode of a sensor
 */
#pragma once

#include <libcamera/transform.h>

#include <libcamera/base/utils.h>

/*
 * Description of a "camera mode", holding enough information for control
 * algorithms to adapt their behaviour to the different modes of the camera,
 * including binning, scaling, cropping etc.
 */

struct CameraMode {
	/* bit depth of the raw camera output */
	uint32_t bitdepth;
	/* size in pixels of frames in this mode */
	uint16_t width;
	uint16_t height;
	/* size of full resolution uncropped frame ("sensor frame") */
	uint16_t sensorWidth;
	uint16_t sensorHeight;
	/* binning factor (1 = no binning, 2 = 2-pixel binning etc.) */
	uint8_t binX;
	uint8_t binY;
	/* location of top left pixel in the sensor frame */
	uint16_t cropX;
	uint16_t cropY;
	/* scaling factor (so if uncropped, width*scaleX is sensorWidth) */
	double scaleX;
	double scaleY;
	/* scaling of the noise compared to the native sensor mode */
	double noiseFactor;
	/* minimum and maximum line time */
	libcamera::utils::Duration minLineLength;
	libcamera::utils::Duration maxLineLength;
	/* any camera transform *not* reflected already in the camera tuning */
	libcamera::Transform transform;
	/* minimum and maximum frame lengths in units of lines */
	uint32_t minFrameLength;
	uint32_t maxFrameLength;
	/* sensitivity of this mode */
	double sensitivity;
	/* pixel clock rate */
	uint64_t pixelRate;
};
