/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_sensor.h - A camera sensor
 */

#pragma once

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <libcamera/base/class.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/orientation.h>
#include <libcamera/transform.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

class CameraLens;
class MediaEntity;
class SensorConfiguration;

enum class Orientation;

struct IPACameraSensorInfo;

class CameraSensor
{
public:
	virtual ~CameraSensor();

	virtual const std::string &model() const = 0;
	virtual const std::string &id() const = 0;

	virtual const MediaEntity *entity() const = 0;

	virtual CameraLens *focusLens() = 0;

	virtual const std::vector<unsigned int> &mbusCodes() const = 0;
	virtual std::vector<Size> sizes(unsigned int mbusCode) const = 0;
	virtual Size resolution() const = 0;

	virtual V4L2SubdeviceFormat
	getFormat(const std::vector<unsigned int> &mbusCodes,
		  const Size &size) const = 0;
	virtual int setFormat(V4L2SubdeviceFormat *format,
			      Transform transform = Transform::Identity) = 0;
	virtual int tryFormat(V4L2SubdeviceFormat *format) const = 0;

	virtual int applyConfiguration(const SensorConfiguration &config,
				       Transform transform = Transform::Identity,
				       V4L2SubdeviceFormat *sensorFormat = nullptr) = 0;

	virtual V4L2Subdevice::Stream imageStream() const;
	virtual std::optional<V4L2Subdevice::Stream> embeddedDataStream() const;
	virtual V4L2SubdeviceFormat embeddedDataFormat() const;
	virtual int setEmbeddedDataEnabled(bool enable);

	virtual const ControlList &properties() const = 0;
	virtual int sensorInfo(IPACameraSensorInfo *info) const = 0;
	virtual Transform computeTransform(Orientation *orientation) const = 0;
	virtual BayerFormat::Order bayerOrder(Transform t) const = 0;

	virtual const ControlInfoMap &controls() const = 0;
	virtual ControlList getControls(const std::vector<uint32_t> &ids) = 0;
	virtual int setControls(ControlList *ctrls) = 0;

	virtual const std::vector<controls::draft::TestPatternModeEnum> &
	testPatternModes() const = 0;
	virtual int setTestPatternMode(controls::draft::TestPatternModeEnum mode) = 0;
};

class CameraSensorFactoryBase
{
public:
	CameraSensorFactoryBase(int priority);
	virtual ~CameraSensorFactoryBase() = default;

	static std::unique_ptr<CameraSensor> create(MediaEntity *entity);

	int priority() const { return priority_; }

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraSensorFactoryBase)

	static std::vector<CameraSensorFactoryBase *> &factories();

	static void registerFactory(CameraSensorFactoryBase *factory);

	virtual std::variant<std::unique_ptr<CameraSensor>, int>
	match(MediaEntity *entity) const = 0;

	int priority_;
};

template<typename _CameraSensor>
class CameraSensorFactory final : public CameraSensorFactoryBase
{
public:
	CameraSensorFactory(int priority = 0)
		: CameraSensorFactoryBase(priority)
	{
	}

private:
	std::variant<std::unique_ptr<CameraSensor>, int>
	match(MediaEntity *entity) const override
	{
		return _CameraSensor::match(entity);
	}
};

#ifndef __DOXYGEN__
#define REGISTER_CAMERA_SENSOR(sensor, ...) \
	static CameraSensorFactory<sensor> global_##sensor##Factory{ __VA_ARGS__ };
#else
#define REGISTER_CAMERA_SENSOR(sensor, priority)
#endif

} /* namespace libcamera */
