/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Database of camera sensor properties
 */

#include "libcamera/internal/camera_sensor_properties.h"

#include <map>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

/**
 * \file camera_sensor_properties.h
 * \brief Database of camera sensor properties
 *
 * The database of camera sensor properties collects static information about
 * camera sensors that is not possible or desirable to retrieve from the device
 * at run time.
 *
 * The database is indexed using the camera sensor model, as reported by the
 * properties::Model property, and for each supported sensor it contains a
 * list of properties.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraSensorProperties)

/**
 * \struct CameraSensorProperties
 * \brief Database of camera sensor properties
 *
 * \var CameraSensorProperties::unitCellSize
 * \brief The physical size of a pixel, including pixel edges, in nanometers.
 *
 * \var CameraSensorProperties::testPatternModes
 * \brief Map that associates the TestPattern control value with the indexes of
 * the corresponding sensor test pattern modes as returned by
 * V4L2_CID_TEST_PATTERN.
 *
 * \var CameraSensorProperties::sensorDelays
 * \brief Sensor control application delays
 *
 * This structure may be defined as empty if the actual sensor delays are not
 * available or have not been measured.
 */

/**
 * \struct CameraSensorProperties::SensorDelays
 * \brief Sensor control application delay values
 *
 * This structure holds delay values, expressed in number of frames, between the
 * time a control value is applied to the sensor and the time that value is
 * reflected in the output. For example "2 frames delay" means that parameters
 * set during frame N will take effect for frame N+2 (and by extension a delay
 * of 0 would mean the parameter is applied immediately to the current frame).
 *
 * \var CameraSensorProperties::SensorDelays::exposureDelay
 * \brief Number of frames between application of exposure control and effect
 *
 * \var CameraSensorProperties::SensorDelays::gainDelay
 * \brief Number of frames between application of analogue gain control and effect
 *
 * \var CameraSensorProperties::SensorDelays::vblankDelay
 * \brief Number of frames between application of vblank control and effect
 *
 * \var CameraSensorProperties::SensorDelays::hblankDelay
 * \brief Number of frames between application of hblank control and effect
 */

/**
 * \brief Retrieve the properties associated with a sensor
 * \param sensor The sensor model name as reported by properties::Model
 * \return A pointer to the CameraSensorProperties instance associated with a sensor
 * or nullptr if the sensor is not supported
 */
const CameraSensorProperties *CameraSensorProperties::get(const std::string &sensor)
{
	static const std::map<std::string, const CameraSensorProperties> sensorProps = {
		{ "ar0144", {
			.unitCellSize = { 3000, 3000 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeSolidColor, 1 },
				{ controls::draft::TestPatternModeColorBars, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
			},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "ar0521", {
			.unitCellSize = { 2200, 2200 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeSolidColor, 1 },
				{ controls::draft::TestPatternModeColorBars, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
			},
			.sensorDelays = { },
		} },
		{ "gc05a2", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "gc08a3", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 2 },
			},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "hi846", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeSolidColor, 1 },
				{ controls::draft::TestPatternModeColorBars, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
				{ controls::draft::TestPatternModePn9, 4 },
				/*
				 * No corresponding test pattern mode for:
				 * 5: "Gradient Horizontal"
				 * 6: "Gradient Vertical"
				 * 7: "Check Board"
				 * 8: "Slant Pattern"
				 * 9: "Resolution Pattern"
				 */
			},
			.sensorDelays = { },
		} },
		{ "imx214", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
				{ controls::draft::TestPatternModeSolidColor, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
				{ controls::draft::TestPatternModePn9, 4 },
			},
			.sensorDelays = { },
		} },
		{ "imx219", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
				{ controls::draft::TestPatternModeSolidColor, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
				{ controls::draft::TestPatternModePn9, 4 },
			},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 1,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "imx258", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeSolidColor, 1 },
				{ controls::draft::TestPatternModeColorBars, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
				{ controls::draft::TestPatternModePn9, 4 },
			},
			.sensorDelays = { },
		} },
		{ "imx283", {
			.unitCellSize = { 2400, 2400 },
			.testPatternModes = {},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 1,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "imx290", {
			.unitCellSize = { 2900, 2900 },
			.testPatternModes = {},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "imx296", {
			.unitCellSize = { 3450, 3450 },
			.testPatternModes = {},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "imx327", {
			.unitCellSize = { 2900, 2900 },
			.testPatternModes = {},
			.sensorDelays = { },
		} },
		{ "imx335", {
			.unitCellSize = { 2000, 2000 },
			.testPatternModes = {},
			.sensorDelays = { },
		} },
		{ "imx415", {
			.unitCellSize = { 1450, 1450 },
			.testPatternModes = {},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "imx462", {
			.unitCellSize = { 2900, 2900 },
			.testPatternModes = {},
			.sensorDelays = { },
		} },
		{ "imx477", {
			.unitCellSize = { 1550, 1550 },
			.testPatternModes = {},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 3,
				.hblankDelay = 3
			},
		} },
		{ "imx500", {
			.unitCellSize = { 1550, 1550 },
			.testPatternModes = {},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 3,
				.hblankDelay = 3
			},
		} },
		{ "imx519", {
			.unitCellSize = { 1220, 1220 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeSolidColor, 2 },
				{ controls::draft::TestPatternModePn9, 4 },
				/*
				 * The driver reports ColorBars and ColorBarsFadeToGray as well but
				 * these two patterns do not comply with MIPI CCS v1.1 (Section 10.1).
				 */
			},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 3,
				.hblankDelay = 3
			},
		} },
		{ "imx708", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
				{ controls::draft::TestPatternModeSolidColor, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
				{ controls::draft::TestPatternModePn9, 4 },
			},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 3,
				.hblankDelay = 3
			},
		} },
		{ "ov2685", {
			.unitCellSize = { 1750, 1750 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1},
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 2 },
				/*
				 * No corresponding test pattern mode for:
				 * 3: "Random Data"
				 * 4: "Black White Square"
				 * 5: "Color Square"
				 */
			},
			.sensorDelays = { },
		} },
		{ "ov2740", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1},
			},
			.sensorDelays = { },
		} },
		{ "ov4689", {
			.unitCellSize = { 2000, 2000 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1},
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 2},
				/*
				 * No corresponding test patterns in
				 * MIPI CCS specification for sensor's
				 * colorBarType2 and colorBarType3.
				 */
			},
			.sensorDelays = { },
		} },
		{ "ov5640", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
			.sensorDelays = { },
		} },
		{ "ov5647", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {},
			/*
			 * We run this sensor in a mode where the gain delay is
			 * bumped up to 2. It seems to be the only way to make
			 * the delays "predictable".
			 *
			 * \todo Verify these delays properly, as the upstream
			 * driver appears to configure _no_ delay.
			 */
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "ov5670", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
			.sensorDelays = { },
		} },
		{ "ov5675", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
			.sensorDelays = { },
		} },
		{ "ov5693", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 2 },
				/*
				 * No corresponding test pattern mode for
				 * 1: "Random data" and 3: "Colour Bars with
				 * Rolling Bar".
				 */
			},
			.sensorDelays = { },
		} },
		{ "ov64a40", {
			.unitCellSize = { 1008, 1008 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 2 },
				/*
				 * No corresponding test patter mode
				 * 3: "Vertical Color Bar Type 3",
				 * 4: "Vertical Color Bar Type 4"
				 */
			},
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "ov7251", {
			.unitCellSize = { 3000, 3000 },
			.testPatternModes =  { },
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "ov8858", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 2 },
				/*
				 * No corresponding test patter mode
				 * 3: "Vertical Color Bar Type 3",
				 * 4: "Vertical Color Bar Type 4"
				 */
			},
			.sensorDelays = { },
		} },
		{ "ov8865", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 2 },
				/*
				 * No corresponding test pattern mode for:
				 * 1: "Random data"
				 * 3: "Color bars with rolling bar"
				 * 4: "Color squares"
				 * 5: "Color squares with rolling bar"
				 */
			},
			.sensorDelays = { },
		} },
		{ "ov9281", {
			.unitCellSize = { 3000, 3000 },
			.testPatternModes =  { },
			.sensorDelays = {
				.exposureDelay = 2,
				.gainDelay = 2,
				.vblankDelay = 2,
				.hblankDelay = 2
			},
		} },
		{ "ov13858", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes =  {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
			.sensorDelays = { },
		} },
	};

	const auto it = sensorProps.find(sensor);
	if (it == sensorProps.end()) {
		LOG(CameraSensorProperties, Warning)
			<< "No static properties available for '" << sensor << "'";
		LOG(CameraSensorProperties, Warning)
			<< "Please consider updating the camera sensor properties database";
		return nullptr;
	}

	return &it->second;
}

} /* namespace libcamera */
