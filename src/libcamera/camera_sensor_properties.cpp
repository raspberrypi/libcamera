/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_properties.cpp - Database of camera sensor properties
 */

#include "libcamera/internal/camera_sensor_properties.h"

#include <map>

#include "libcamera/internal/log.h"

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
		{ "imx219", {
			.unitCellSize = { 1120, 1120 },
		} },
		{ "imx258", {
			.unitCellSize = { 1120, 1120 },
		} },
		{ "ov5670", {
			.unitCellSize = { 1120, 1120 },
		} },
		{ "ov13858", {
			.unitCellSize = { 1120, 1120 },
		} },
		{ "ov5693", {
			.unitCellSize = { 1400, 1400 },
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
