/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_properties.cpp - Database of camera sensor properties
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
		{ "ar0521", {
			.unitCellSize = { 2200, 2200 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeSolidColor, 1 },
				{ controls::draft::TestPatternModeColorBars, 2 },
				{ controls::draft::TestPatternModeColorBarsFadeToGray, 3 },
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
		} },
		{ "imx290", {
			.unitCellSize = { 2900, 2900 },
			.testPatternModes = {},
		} },
		{ "imx296", {
			.unitCellSize = { 3450, 3450 },
			.testPatternModes = {},
		} },
		{ "imx327", {
			.unitCellSize = { 2900, 2900 },
			.testPatternModes = {},
		} },
		{ "imx477", {
			.unitCellSize = { 1550, 1550 },
			.testPatternModes = {},
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
		} },
		{ "imx708_noir", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {},
		} },
		{ "imx708_wide", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {},
		} },
		{ "imx708_wide_noir", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {},
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
		} },
		{ "ov2740", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1},
			},
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
		} },
		{ "ov5640", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
		} },
		{ "ov5647", {
			.unitCellSize = { 1400, 1400 },
			.testPatternModes = {},
		} },
		{ "ov5670", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
		} },
		{ "ov5675", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes = {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
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
		} },
		{ "ov64a40", {
			.unitCellSize = { 1008, 1008 },
			.testPatternModes = {},
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
		} },
		{ "ov13858", {
			.unitCellSize = { 1120, 1120 },
			.testPatternModes =  {
				{ controls::draft::TestPatternModeOff, 0 },
				{ controls::draft::TestPatternModeColorBars, 1 },
			},
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
