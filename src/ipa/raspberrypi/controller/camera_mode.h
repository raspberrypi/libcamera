/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2020, Raspberry Pi (Trading) Limited
 *
 * camera_mode.h - description of a particular operating mode of a sensor
 */
#pragma once

// Description of a "camera mode", holding enough information for control
// algorithms to adapt their behaviour to the different modes of the camera,
// including binning, scaling, cropping etc.

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_MODE_NAME_LEN 32

struct CameraMode {
	// bit depth of the raw camera output
	uint32_t bitdepth;
	// size in pixels of frames in this mode
	uint16_t width, height;
	// size of full resolution uncropped frame ("sensor frame")
	uint16_t sensor_width, sensor_height;
	// binning factor (1 = no binning, 2 = 2-pixel binning etc.)
	uint8_t bin_x, bin_y;
	// location of top left pixel in the sensor frame
	uint16_t crop_x, crop_y;
	// scaling factor (so if uncropped, width*scale_x is sensor_width)
	double scale_x, scale_y;
	// scaling of the noise compared to the native sensor mode
	double noise_factor;
	// line time in nanoseconds
	double line_length;
};

#ifdef __cplusplus
}
#endif
