/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * A camera sensor
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/orientation.h>
#include <libcamera/transform.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

class CameraLens;
class MediaEntity;
class SensorConfiguration;

struct CameraSensorProperties;

enum class Orientation;

class CameraSensor : protected Loggable
{
public:
	explicit CameraSensor(const MediaEntity *entity);
	~CameraSensor();

	int init();

	const std::string &model() const { return model_; }
	const std::string &id() const { return id_; }

	const MediaEntity *entity() const { return entity_; }
	V4L2Subdevice *device() { return subdev_.get(); }

	CameraLens *focusLens() { return focusLens_.get(); }

	const std::vector<unsigned int> &mbusCodes() const { return mbusCodes_; }
	std::vector<Size> sizes(unsigned int mbusCode) const;
	Size resolution() const;

	V4L2SubdeviceFormat getFormat(const std::vector<unsigned int> &mbusCodes,
				      const Size &size) const;
	int setFormat(V4L2SubdeviceFormat *format,
		      Transform transform = Transform::Identity);
	int tryFormat(V4L2SubdeviceFormat *format) const;

	int applyConfiguration(const SensorConfiguration &config,
			       Transform transform = Transform::Identity,
			       V4L2SubdeviceFormat *sensorFormat = nullptr);

	const ControlList &properties() const { return properties_; }
	int sensorInfo(IPACameraSensorInfo *info) const;
	Transform computeTransform(Orientation *orientation) const;
	BayerFormat::Order bayerOrder(Transform t) const;

	const ControlInfoMap &controls() const;
	ControlList getControls(const std::vector<uint32_t> &ids);
	int setControls(ControlList *ctrls);

	const std::vector<controls::draft::TestPatternModeEnum> &testPatternModes() const
	{
		return testPatternModes_;
	}
	int setTestPatternMode(controls::draft::TestPatternModeEnum mode);

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY(CameraSensor)

	int generateId();
	int validateSensorDriver();
	void initVimcDefaultProperties();
	void initStaticProperties();
	void initTestPatternModes();
	int initProperties();
	int discoverAncillaryDevices();
	int applyTestPatternMode(controls::draft::TestPatternModeEnum mode);

	const MediaEntity *entity_;
	std::unique_ptr<V4L2Subdevice> subdev_;
	unsigned int pad_;

	const CameraSensorProperties *staticProps_;

	std::string model_;
	std::string id_;

	V4L2Subdevice::Formats formats_;
	std::vector<unsigned int> mbusCodes_;
	std::vector<Size> sizes_;
	std::vector<controls::draft::TestPatternModeEnum> testPatternModes_;
	controls::draft::TestPatternModeEnum testPatternMode_;

	Size pixelArraySize_;
	Rectangle activeArea_;
	const BayerFormat *bayerFormat_;
	bool supportFlips_;
	bool flipsAlterBayerOrder_;
	Orientation mountingOrientation_;

	ControlList properties_;

	std::unique_ptr<CameraLens> focusLens_;
};

} /* namespace libcamera */
