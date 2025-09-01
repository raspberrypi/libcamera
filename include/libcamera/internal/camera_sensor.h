/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * A camera sensor
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <string>
#include <variant>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/orientation.h>
#include <libcamera/transform.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera_sensor_properties.h"
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
	virtual V4L2Subdevice *device() = 0;

	virtual CameraLens *focusLens() = 0;

	virtual const std::vector<unsigned int> &mbusCodes() const = 0;
	virtual std::vector<Size> sizes(unsigned int mbusCode) const = 0;
	virtual Size resolution() const = 0;

	virtual V4L2SubdeviceFormat
	getFormat(Span<const unsigned int> mbusCodes,
		  const Size &size, const Size maxSize = Size()) const = 0;
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
	virtual ControlList getControls(Span<const uint32_t> ids) = 0;
	virtual int setControls(ControlList *ctrls) = 0;

	virtual const std::vector<controls::draft::TestPatternModeEnum> &
	testPatternModes() const = 0;
	virtual int setTestPatternMode(controls::draft::TestPatternModeEnum mode) = 0;
	virtual const CameraSensorProperties::SensorDelays &sensorDelays() = 0;
};

class CameraSensorFactoryBase
{
public:
	CameraSensorFactoryBase(const char *name, int priority);
	virtual ~CameraSensorFactoryBase() = default;

	static std::unique_ptr<CameraSensor> create(MediaEntity *entity);

	const std::string &name() const { return name_; }
	int priority() const { return priority_; }

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraSensorFactoryBase)

	static std::vector<CameraSensorFactoryBase *> &factories();

	static void registerFactory(CameraSensorFactoryBase *factory);

	virtual std::variant<std::unique_ptr<CameraSensor>, int>
	match(MediaEntity *entity) const = 0;

	std::string name_;
	int priority_;
};

template<typename _CameraSensor>
class CameraSensorFactory final : public CameraSensorFactoryBase
{
public:
	CameraSensorFactory(const char *name, int priority)
		: CameraSensorFactoryBase(name, priority)
	{
	}

private:
	std::variant<std::unique_ptr<CameraSensor>, int>
	match(MediaEntity *entity) const override
	{
		return _CameraSensor::match(entity);
	}
};

#define REGISTER_CAMERA_SENSOR(sensor, priority) \
static CameraSensorFactory<sensor> global_##sensor##Factory{ #sensor, priority };

} /* namespace libcamera */
