/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_sensor.h - A camera sensor
 */
#ifndef __LIBCAMERA_CAMERA_SENSOR_H__
#define __LIBCAMERA_CAMERA_SENSOR_H__

#include <string>
#include <vector>

#include <libcamera/geometry.h>

#include "log.h"

namespace libcamera {

class MediaEntity;
class V4L2Subdevice;

struct V4L2SubdeviceFormat;

class CameraSensor : protected Loggable
{
public:
	explicit CameraSensor(const MediaEntity *entity);
	~CameraSensor();

	CameraSensor(const CameraSensor &) = delete;
	CameraSensor &operator=(const CameraSensor &) = delete;

	int init();

	const MediaEntity *entity() const { return entity_; }
	const std::vector<unsigned int> &mbusCodes() const { return mbusCodes_; }
	const std::vector<Size> &sizes() const { return sizes_; }
	const Size &resolution() const;

	V4L2SubdeviceFormat getFormat(const std::vector<unsigned int> &mbusCodes,
				      const Size &size) const;
	int setFormat(V4L2SubdeviceFormat *format);

protected:
	std::string logPrefix() const;

private:
	const MediaEntity *entity_;
	V4L2Subdevice *subdev_;

	std::vector<unsigned int> mbusCodes_;
	std::vector<Size> sizes_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CAMERA_SENSOR_H__ */
