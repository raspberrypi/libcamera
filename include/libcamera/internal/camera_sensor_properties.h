/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_properties.h - Database of camera sensor properties
 */
#ifndef __LIBCAMERA_SENSOR_CAMERA_SENSOR_PROPERTIES_H__
#define __LIBCAMERA_SENSOR_CAMERA_SENSOR_PROPERTIES_H__

#include <string>

#include <libcamera/geometry.h>

namespace libcamera {

struct CameraSensorProperties {
	static const CameraSensorProperties *get(const std::string &sensor);

	Size unitCellSize;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_SENSOR_CAMERA_SENSOR_PROPERTIES_H__ */
