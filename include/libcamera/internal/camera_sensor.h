/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_sensor.h - A camera sensor
 */
#ifndef __LIBCAMERA_INTERNAL_CAMERA_SENSOR_H__
#define __LIBCAMERA_INTERNAL_CAMERA_SENSOR_H__

#include <memory>
#include <string>
#include <vector>

#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

class MediaEntity;

struct CameraSensorInfo {
	std::string model;

	uint32_t bitsPerPixel;

	Size activeAreaSize;
	Rectangle analogCrop;
	Size outputSize;

	uint64_t pixelRate;
	uint32_t lineLength;
};

class CameraSensor : protected Loggable
{
public:
	explicit CameraSensor(const MediaEntity *entity);
	~CameraSensor();

	CameraSensor(const CameraSensor &) = delete;
	CameraSensor &operator=(const CameraSensor &) = delete;

	int init();

	const std::string &model() const { return model_; }
	const std::string &id() const { return id_; }
	const MediaEntity *entity() const { return entity_; }
	const std::vector<unsigned int> &mbusCodes() const { return mbusCodes_; }
	const std::vector<Size> &sizes() const { return sizes_; }
	const Size &resolution() const { return resolution_; }

	V4L2SubdeviceFormat getFormat(const std::vector<unsigned int> &mbusCodes,
				      const Size &size) const;
	int setFormat(V4L2SubdeviceFormat *format);

	const ControlInfoMap &controls() const;
	ControlList getControls(const std::vector<uint32_t> &ids);
	int setControls(ControlList *ctrls);

	const ControlList &properties() const { return properties_; }
	int sensorInfo(CameraSensorInfo *info) const;

protected:
	std::string logPrefix() const override;

private:
	int generateId();

	const MediaEntity *entity_;
	std::unique_ptr<V4L2Subdevice> subdev_;
	unsigned int pad_;

	std::string model_;
	std::string id_;

	V4L2Subdevice::Formats formats_;
	Size resolution_;
	std::vector<unsigned int> mbusCodes_;
	std::vector<Size> sizes_;

	ControlList properties_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_CAMERA_SENSOR_H__ */
