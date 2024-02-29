/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_sensor_legacy.cpp - A V4L2-backed camera sensor
 */

#include <algorithm>
#include <float.h>
#include <iomanip>
#include <limits.h>
#include <map>
#include <math.h>
#include <memory>
#include <string.h>
#include <string>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/orientation.h>
#include <libcamera/property_ids.h>
#include <libcamera/transform.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera_lens.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/sysfs.h"
#include "libcamera/internal/v4l2_subdevice.h"

namespace libcamera {

class BayerFormat;
class CameraLens;
class MediaEntity;
class SensorConfiguration;

struct CameraSensorProperties;

enum class Orientation;

LOG_DECLARE_CATEGORY(CameraSensor)

class CameraSensorLegacy : public CameraSensor, protected Loggable
{
public:
	CameraSensorLegacy(const MediaEntity *entity);
	~CameraSensorLegacy();

	static std::variant<std::unique_ptr<CameraSensor>, int>
	match(MediaEntity *entity);

	const std::string &model() const override { return model_; }
	const std::string &id() const override { return id_; }

	const MediaEntity *entity() const override { return entity_; }

	CameraLens *focusLens() override { return focusLens_.get(); }

	const std::vector<unsigned int> &mbusCodes() const override { return mbusCodes_; }
	std::vector<Size> sizes(unsigned int mbusCode) const override;
	Size resolution() const override;

	V4L2SubdeviceFormat getFormat(const std::vector<unsigned int> &mbusCodes,
				      const Size &size) const override;
	int setFormat(V4L2SubdeviceFormat *format,
		      Transform transform = Transform::Identity) override;
	int tryFormat(V4L2SubdeviceFormat *format) const override;

	int applyConfiguration(const SensorConfiguration &config,
			       Transform transform = Transform::Identity,
			       V4L2SubdeviceFormat *sensorFormat = nullptr) override;

	const ControlList &properties() const override { return properties_; }
	int sensorInfo(IPACameraSensorInfo *info) const override;
	Transform computeTransform(Orientation *orientation) const override;
	BayerFormat::Order bayerOrder(Transform t) const override;

	const ControlInfoMap &controls() const override;
	ControlList getControls(const std::vector<uint32_t> &ids) override;
	int setControls(ControlList *ctrls) override;

	const std::vector<controls::draft::TestPatternModeEnum> &
	testPatternModes() const override { return testPatternModes_; }
	int setTestPatternMode(controls::draft::TestPatternModeEnum mode) override;

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY(CameraSensorLegacy)

	int init();
	int generateId();
	int validateSensorDriver();
	void initVimcDefaultProperties();
	void initStaticProperties();
	void initTestPatternModes();
	int initProperties();
	int applyTestPatternMode(controls::draft::TestPatternModeEnum mode);
	int discoverAncillaryDevices();

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

/**
 * \class CameraSensorLegacy
 * \brief A camera sensor based on V4L2 subdevices
 *
 * The implementation is currently limited to sensors that expose a single V4L2
 * subdevice with a single pad. It will be extended to support more complex
 * devices as the needs arise.
 */

CameraSensorLegacy::CameraSensorLegacy(const MediaEntity *entity)
	: entity_(entity), pad_(UINT_MAX), staticProps_(nullptr),
	  bayerFormat_(nullptr), supportFlips_(false),
	  flipsAlterBayerOrder_(false), properties_(properties::properties)
{
}

CameraSensorLegacy::~CameraSensorLegacy() = default;

std::variant<std::unique_ptr<CameraSensor>, int>
CameraSensorLegacy::match(MediaEntity *entity)
{
	std::unique_ptr<CameraSensorLegacy> sensor =
		std::make_unique<CameraSensorLegacy>(entity);

	int ret = sensor->init();
	if (ret)
		return { ret };

	return { std::move(sensor) };
}

int CameraSensorLegacy::init()
{
	for (const MediaPad *pad : entity_->pads()) {
		if (pad->flags() & MEDIA_PAD_FL_SOURCE) {
			pad_ = pad->index();
			break;
		}
	}

	if (pad_ == UINT_MAX) {
		LOG(CameraSensor, Error)
			<< "Sensors with more than one pad are not supported";
		return -EINVAL;
	}

	switch (entity_->function()) {
	case MEDIA_ENT_F_CAM_SENSOR:
	case MEDIA_ENT_F_PROC_VIDEO_ISP:
		break;

	default:
		LOG(CameraSensor, Error)
			<< "Invalid sensor function "
			<< utils::hex(entity_->function());
		return -EINVAL;
	}

	/* Create and open the subdev. */
	subdev_ = std::make_unique<V4L2Subdevice>(entity_);
	int ret = subdev_->open();
	if (ret < 0)
		return ret;

	/*
	 * Clear any flips to be sure we get the "native" Bayer order. This is
	 * harmless for sensors where the flips don't affect the Bayer order.
	 */
	ControlList ctrls(subdev_->controls());
	if (subdev_->controls().find(V4L2_CID_HFLIP) != subdev_->controls().end())
		ctrls.set(V4L2_CID_HFLIP, 0);
	if (subdev_->controls().find(V4L2_CID_VFLIP) != subdev_->controls().end())
		ctrls.set(V4L2_CID_VFLIP, 0);
	subdev_->setControls(&ctrls);

	/* Enumerate, sort and cache media bus codes and sizes. */
	formats_ = subdev_->formats(pad_);
	if (formats_.empty()) {
		LOG(CameraSensor, Error) << "No image format found";
		return -EINVAL;
	}

	mbusCodes_ = utils::map_keys(formats_);
	std::sort(mbusCodes_.begin(), mbusCodes_.end());

	for (const auto &format : formats_) {
		const std::vector<SizeRange> &ranges = format.second;
		std::transform(ranges.begin(), ranges.end(), std::back_inserter(sizes_),
			       [](const SizeRange &range) { return range.max; });
	}

	std::sort(sizes_.begin(), sizes_.end());

	/* Remove duplicates. */
	auto last = std::unique(sizes_.begin(), sizes_.end());
	sizes_.erase(last, sizes_.end());

	/*
	 * VIMC is a bit special, as it does not yet support all the mandatory
	 * requirements regular sensors have to respect.
	 *
	 * Do not validate the driver if it's VIMC and initialize the sensor
	 * properties with static information.
	 *
	 * \todo Remove the special case once the VIMC driver has been
	 * updated in all test platforms.
	 */
	if (entity_->device()->driver() == "vimc") {
		initVimcDefaultProperties();

		ret = initProperties();
		if (ret)
			return ret;

		return discoverAncillaryDevices();
	}

	/* Get the color filter array pattern (only for RAW sensors). */
	for (unsigned int mbusCode : mbusCodes_) {
		const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(mbusCode);
		if (bayerFormat.isValid()) {
			bayerFormat_ = &bayerFormat;
			break;
		}
	}

	ret = validateSensorDriver();
	if (ret)
		return ret;

	ret = initProperties();
	if (ret)
		return ret;

	ret = discoverAncillaryDevices();
	if (ret)
		return ret;

	/*
	 * Set HBLANK to the minimum to start with a well-defined line length,
	 * allowing IPA modules that do not modify HBLANK to use the sensor
	 * minimum line length in their calculations.
	 */
	const struct v4l2_query_ext_ctrl *hblankInfo = subdev_->controlInfo(V4L2_CID_HBLANK);
	if (hblankInfo && !(hblankInfo->flags & V4L2_CTRL_FLAG_READ_ONLY)) {
		ControlList ctrl(subdev_->controls());

		ctrl.set(V4L2_CID_HBLANK, static_cast<int32_t>(hblankInfo->minimum));
		ret = subdev_->setControls(&ctrl);
		if (ret)
			return ret;
	}

	return applyTestPatternMode(controls::draft::TestPatternModeEnum::TestPatternModeOff);
}

int CameraSensorLegacy::generateId()
{
	const std::string devPath = subdev_->devicePath();

	/* Try to get ID from firmware description. */
	id_ = sysfs::firmwareNodePath(devPath);
	if (!id_.empty())
		return 0;

	/*
	 * Virtual sensors not described in firmware
	 *
	 * Verify it's a platform device and construct ID from the device path
	 * and model of sensor.
	 */
	if (devPath.find("/sys/devices/platform/", 0) == 0) {
		id_ = devPath.substr(strlen("/sys/devices/")) + " " + model();
		return 0;
	}

	LOG(CameraSensor, Error) << "Can't generate sensor ID";
	return -EINVAL;
}

int CameraSensorLegacy::validateSensorDriver()
{
	int err = 0;

	/*
	 * Optional controls are used to register optional sensor properties. If
	 * not present, some values will be defaulted.
	 */
	static constexpr uint32_t optionalControls[] = {
		V4L2_CID_CAMERA_SENSOR_ROTATION,
	};

	const ControlIdMap &controls = subdev_->controls().idmap();
	for (uint32_t ctrl : optionalControls) {
		if (!controls.count(ctrl))
			LOG(CameraSensor, Debug)
				<< "Optional V4L2 control " << utils::hex(ctrl)
				<< " not supported";
	}

	/*
	 * Recommended controls are similar to optional controls, but will
	 * become mandatory in the near future. Be loud if they're missing.
	 */
	static constexpr uint32_t recommendedControls[] = {
		V4L2_CID_CAMERA_ORIENTATION,
	};

	for (uint32_t ctrl : recommendedControls) {
		if (!controls.count(ctrl)) {
			LOG(CameraSensor, Warning)
				<< "Recommended V4L2 control " << utils::hex(ctrl)
				<< " not supported";
			err = -EINVAL;
		}
	}

	/*
	 * Verify if sensor supports horizontal/vertical flips
	 *
	 * \todo Handle horizontal and vertical flips independently.
	 */
	const struct v4l2_query_ext_ctrl *hflipInfo = subdev_->controlInfo(V4L2_CID_HFLIP);
	const struct v4l2_query_ext_ctrl *vflipInfo = subdev_->controlInfo(V4L2_CID_VFLIP);
	if (hflipInfo && !(hflipInfo->flags & V4L2_CTRL_FLAG_READ_ONLY) &&
	    vflipInfo && !(vflipInfo->flags & V4L2_CTRL_FLAG_READ_ONLY)) {
		supportFlips_ = true;

		if (hflipInfo->flags & V4L2_CTRL_FLAG_MODIFY_LAYOUT ||
		    vflipInfo->flags & V4L2_CTRL_FLAG_MODIFY_LAYOUT)
			flipsAlterBayerOrder_ = true;
	}

	if (!supportFlips_)
		LOG(CameraSensor, Debug)
			<< "Camera sensor does not support horizontal/vertical flip";

	/*
	 * Make sure the required selection targets are supported.
	 *
	 * Failures in reading any of the targets are not deemed to be fatal,
	 * but some properties and features, like constructing a
	 * IPACameraSensorInfo for the IPA module, won't be supported.
	 *
	 * \todo Make support for selection targets mandatory as soon as all
	 * test platforms have been updated.
	 */
	Rectangle rect;
	int ret = subdev_->getSelection(pad_, V4L2_SEL_TGT_CROP_BOUNDS, &rect);
	if (ret) {
		/*
		 * Default the pixel array size to the largest size supported
		 * by the sensor. The sizes_ vector is sorted in ascending
		 * order, the largest size is thus the last element.
		 */
		pixelArraySize_ = sizes_.back();

		LOG(CameraSensor, Warning)
			<< "The PixelArraySize property has been defaulted to "
			<< pixelArraySize_;
		err = -EINVAL;
	} else {
		pixelArraySize_ = rect.size();
	}

	ret = subdev_->getSelection(pad_, V4L2_SEL_TGT_CROP_DEFAULT, &activeArea_);
	if (ret) {
		activeArea_ = Rectangle(pixelArraySize_);
		LOG(CameraSensor, Warning)
			<< "The PixelArrayActiveAreas property has been defaulted to "
			<< activeArea_;
		err = -EINVAL;
	}

	ret = subdev_->getSelection(pad_, V4L2_SEL_TGT_CROP, &rect);
	if (ret) {
		LOG(CameraSensor, Warning)
			<< "Failed to retrieve the sensor crop rectangle";
		err = -EINVAL;
	}

	if (err) {
		LOG(CameraSensor, Warning)
			<< "The sensor kernel driver needs to be fixed";
		LOG(CameraSensor, Warning)
			<< "See Documentation/sensor_driver_requirements.rst in the libcamera sources for more information";
	}

	if (!bayerFormat_)
		return 0;

	/*
	 * For raw sensors, make sure the sensor driver supports the controls
	 * required by the CameraSensor class.
	 */
	static constexpr uint32_t mandatoryControls[] = {
		V4L2_CID_ANALOGUE_GAIN,
		V4L2_CID_EXPOSURE,
		V4L2_CID_HBLANK,
		V4L2_CID_PIXEL_RATE,
		V4L2_CID_VBLANK,
	};

	err = 0;
	for (uint32_t ctrl : mandatoryControls) {
		if (!controls.count(ctrl)) {
			LOG(CameraSensor, Error)
				<< "Mandatory V4L2 control " << utils::hex(ctrl)
				<< " not available";
			err = -EINVAL;
		}
	}

	if (err) {
		LOG(CameraSensor, Error)
			<< "The sensor kernel driver needs to be fixed";
		LOG(CameraSensor, Error)
			<< "See Documentation/sensor_driver_requirements.rst in the libcamera sources for more information";
		return err;
	}

	return 0;
}

void CameraSensorLegacy::initVimcDefaultProperties()
{
	/* Use the largest supported size. */
	pixelArraySize_ = sizes_.back();
	activeArea_ = Rectangle(pixelArraySize_);
}

void CameraSensorLegacy::initStaticProperties()
{
	staticProps_ = CameraSensorProperties::get(model_);
	if (!staticProps_)
		return;

	/* Register the properties retrieved from the sensor database. */
	properties_.set(properties::UnitCellSize, staticProps_->unitCellSize);

	initTestPatternModes();
}

void CameraSensorLegacy::initTestPatternModes()
{
	const auto &v4l2TestPattern = controls().find(V4L2_CID_TEST_PATTERN);
	if (v4l2TestPattern == controls().end()) {
		LOG(CameraSensor, Debug) << "V4L2_CID_TEST_PATTERN is not supported";
		return;
	}

	const auto &testPatternModes = staticProps_->testPatternModes;
	if (testPatternModes.empty()) {
		/*
		 * The camera sensor supports test patterns but we don't know
		 * how to map them so this should be fixed.
		 */
		LOG(CameraSensor, Debug) << "No static test pattern map for \'"
					 << model() << "\'";
		return;
	}

	/*
	 * Create a map that associates the V4L2 control index to the test
	 * pattern mode by reversing the testPatternModes map provided by the
	 * camera sensor properties. This makes it easier to verify if the
	 * control index is supported in the below for loop that creates the
	 * list of supported test patterns.
	 */
	std::map<int32_t, controls::draft::TestPatternModeEnum> indexToTestPatternMode;
	for (const auto &it : testPatternModes)
		indexToTestPatternMode[it.second] = it.first;

	for (const ControlValue &value : v4l2TestPattern->second.values()) {
		const int32_t index = value.get<int32_t>();

		const auto it = indexToTestPatternMode.find(index);
		if (it == indexToTestPatternMode.end()) {
			LOG(CameraSensor, Debug)
				<< "Test pattern mode " << index << " ignored";
			continue;
		}

		testPatternModes_.push_back(it->second);
	}
}

int CameraSensorLegacy::initProperties()
{
	model_ = subdev_->model();
	properties_.set(properties::Model, utils::toAscii(model_));

	/* Generate a unique ID for the sensor. */
	int ret = generateId();
	if (ret)
		return ret;

	/* Initialize the static properties from the sensor database. */
	initStaticProperties();

	/* Retrieve and register properties from the kernel interface. */
	const ControlInfoMap &controls = subdev_->controls();

	const auto &orientation = controls.find(V4L2_CID_CAMERA_ORIENTATION);
	if (orientation != controls.end()) {
		int32_t v4l2Orientation = orientation->second.def().get<int32_t>();
		int32_t propertyValue;

		switch (v4l2Orientation) {
		default:
			LOG(CameraSensor, Warning)
				<< "Unsupported camera location "
				<< v4l2Orientation << ", setting to External";
			[[fallthrough]];
		case V4L2_CAMERA_ORIENTATION_EXTERNAL:
			propertyValue = properties::CameraLocationExternal;
			break;
		case V4L2_CAMERA_ORIENTATION_FRONT:
			propertyValue = properties::CameraLocationFront;
			break;
		case V4L2_CAMERA_ORIENTATION_BACK:
			propertyValue = properties::CameraLocationBack;
			break;
		}
		properties_.set(properties::Location, propertyValue);
	} else {
		LOG(CameraSensor, Warning) << "Failed to retrieve the camera location";
	}

	const auto &rotationControl = controls.find(V4L2_CID_CAMERA_SENSOR_ROTATION);
	if (rotationControl != controls.end()) {
		int32_t propertyValue = rotationControl->second.def().get<int32_t>();

		/*
		 * Cache the Transform associated with the camera mounting
		 * rotation for later use in computeTransform().
		 */
		bool success;
		mountingOrientation_ = orientationFromRotation(propertyValue, &success);
		if (!success) {
			LOG(CameraSensor, Warning)
				<< "Invalid rotation of " << propertyValue
				<< " degrees - ignoring";
			mountingOrientation_ = Orientation::Rotate0;
		}

		properties_.set(properties::Rotation, propertyValue);
	} else {
		LOG(CameraSensor, Warning)
			<< "Rotation control not available, default to 0 degrees";
		properties_.set(properties::Rotation, 0);
		mountingOrientation_ = Orientation::Rotate0;
	}

	properties_.set(properties::PixelArraySize, pixelArraySize_);
	properties_.set(properties::PixelArrayActiveAreas, { activeArea_ });

	/* Color filter array pattern, register only for RAW sensors. */
	if (bayerFormat_) {
		int32_t cfa;
		switch (bayerFormat_->order) {
		case BayerFormat::BGGR:
			cfa = properties::draft::BGGR;
			break;
		case BayerFormat::GBRG:
			cfa = properties::draft::GBRG;
			break;
		case BayerFormat::GRBG:
			cfa = properties::draft::GRBG;
			break;
		case BayerFormat::RGGB:
			cfa = properties::draft::RGGB;
			break;
		case BayerFormat::MONO:
			cfa = properties::draft::MONO;
			break;
		}

		properties_.set(properties::draft::ColorFilterArrangement, cfa);
	}

	return 0;
}

int CameraSensorLegacy::discoverAncillaryDevices()
{
	int ret;

	for (MediaEntity *ancillary : entity_->ancillaryEntities()) {
		switch (ancillary->function()) {
		case MEDIA_ENT_F_LENS:
			focusLens_ = std::make_unique<CameraLens>(ancillary);
			ret = focusLens_->init();
			if (ret) {
				LOG(CameraSensor, Error)
					<< "Lens initialisation failed, lens disabled";
				focusLens_.reset();
			}
			break;

		default:
			LOG(CameraSensor, Warning)
				<< "Unsupported ancillary entity function "
				<< ancillary->function();
			break;
		}
	}

	return 0;
}

std::vector<Size> CameraSensorLegacy::sizes(unsigned int mbusCode) const
{
	std::vector<Size> sizes;

	const auto &format = formats_.find(mbusCode);
	if (format == formats_.end())
		return sizes;

	const std::vector<SizeRange> &ranges = format->second;
	std::transform(ranges.begin(), ranges.end(), std::back_inserter(sizes),
		       [](const SizeRange &range) { return range.max; });

	std::sort(sizes.begin(), sizes.end());

	return sizes;
}

Size CameraSensorLegacy::resolution() const
{
	return std::min(sizes_.back(), activeArea_.size());
}

V4L2SubdeviceFormat
CameraSensorLegacy::getFormat(const std::vector<unsigned int> &mbusCodes,
			      const Size &size) const
{
	unsigned int desiredArea = size.width * size.height;
	unsigned int bestArea = UINT_MAX;
	float desiredRatio = static_cast<float>(size.width) / size.height;
	float bestRatio = FLT_MAX;
	const Size *bestSize = nullptr;
	uint32_t bestCode = 0;

	for (unsigned int code : mbusCodes) {
		const auto formats = formats_.find(code);
		if (formats == formats_.end())
			continue;

		for (const SizeRange &range : formats->second) {
			const Size &sz = range.max;

			if (sz.width < size.width || sz.height < size.height)
				continue;

			float ratio = static_cast<float>(sz.width) / sz.height;
			float ratioDiff = fabsf(ratio - desiredRatio);
			unsigned int area = sz.width * sz.height;
			unsigned int areaDiff = area - desiredArea;

			if (ratioDiff > bestRatio)
				continue;

			if (ratioDiff < bestRatio || areaDiff < bestArea) {
				bestRatio = ratioDiff;
				bestArea = areaDiff;
				bestSize = &sz;
				bestCode = code;
			}
		}
	}

	if (!bestSize) {
		LOG(CameraSensor, Debug) << "No supported format or size found";
		return {};
	}

	V4L2SubdeviceFormat format{
		.code = bestCode,
		.size = *bestSize,
		.colorSpace = ColorSpace::Raw,
	};

	return format;
}

int CameraSensorLegacy::setFormat(V4L2SubdeviceFormat *format, Transform transform)
{
	/* Configure flips if the sensor supports that. */
	if (supportFlips_) {
		ControlList flipCtrls(subdev_->controls());

		flipCtrls.set(V4L2_CID_HFLIP,
			      static_cast<int32_t>(!!(transform & Transform::HFlip)));
		flipCtrls.set(V4L2_CID_VFLIP,
			      static_cast<int32_t>(!!(transform & Transform::VFlip)));

		int ret = subdev_->setControls(&flipCtrls);
		if (ret)
			return ret;
	}

	/* Apply format on the subdev. */
	int ret = subdev_->setFormat(pad_, format);
	if (ret)
		return ret;

	subdev_->updateControlInfo();
	return 0;
}

int CameraSensorLegacy::tryFormat(V4L2SubdeviceFormat *format) const
{
	return subdev_->setFormat(pad_, format,
				  V4L2Subdevice::Whence::TryFormat);
}

int CameraSensorLegacy::applyConfiguration(const SensorConfiguration &config,
					   Transform transform,
					   V4L2SubdeviceFormat *sensorFormat)
{
	if (!config.isValid()) {
		LOG(CameraSensor, Error) << "Invalid sensor configuration";
		return -EINVAL;
	}

	std::vector<unsigned int> filteredCodes;
	std::copy_if(mbusCodes_.begin(), mbusCodes_.end(),
		     std::back_inserter(filteredCodes),
		     [&config](unsigned int mbusCode) {
			     BayerFormat bayer = BayerFormat::fromMbusCode(mbusCode);
			     if (bayer.bitDepth == config.bitDepth)
				     return true;
			     return false;
		     });
	if (filteredCodes.empty()) {
		LOG(CameraSensor, Error)
			<< "Cannot find any format with bit depth "
			<< config.bitDepth;
		return -EINVAL;
	}

	/*
	 * Compute the sensor's data frame size by applying the cropping
	 * rectangle, subsampling and output crop to the sensor's pixel array
	 * size.
	 *
	 * \todo The actual size computation is for now ignored and only the
	 * output size is considered. This implies that resolutions obtained
	 * with two different cropping/subsampling will look identical and
	 * only the first found one will be considered.
	 */
	V4L2SubdeviceFormat subdevFormat = {};
	for (unsigned int code : filteredCodes) {
		for (const Size &size : sizes(code)) {
			if (size.width != config.outputSize.width ||
			    size.height != config.outputSize.height)
				continue;

			subdevFormat.code = code;
			subdevFormat.size = size;
			break;
		}
	}
	if (!subdevFormat.code) {
		LOG(CameraSensor, Error) << "Invalid output size in sensor configuration";
		return -EINVAL;
	}

	int ret = setFormat(&subdevFormat, transform);
	if (ret)
		return ret;

	/*
	 * Return to the caller the format actually applied to the sensor.
	 * This is relevant if transform has changed the bayer pattern order.
	 */
	if (sensorFormat)
		*sensorFormat = subdevFormat;

	/* \todo Handle AnalogCrop. Most sensors do not support set_selection */
	/* \todo Handle scaling in the digital domain. */

	return 0;
}

int CameraSensorLegacy::sensorInfo(IPACameraSensorInfo *info) const
{
	if (!bayerFormat_)
		return -EINVAL;

	info->model = model();

	/*
	 * The active area size is a static property, while the crop
	 * rectangle needs to be re-read as it depends on the sensor
	 * configuration.
	 */
	info->activeAreaSize = { activeArea_.width, activeArea_.height };

	/*
	 * \todo Support for retreiving the crop rectangle is scheduled to
	 * become mandatory. For the time being use the default value if it has
	 * been initialized at sensor driver validation time.
	 */
	int ret = subdev_->getSelection(pad_, V4L2_SEL_TGT_CROP, &info->analogCrop);
	if (ret) {
		info->analogCrop = activeArea_;
		LOG(CameraSensor, Warning)
			<< "The analogue crop rectangle has been defaulted to the active area size";
	}

	/*
	 * IPACameraSensorInfo::analogCrop::x and IPACameraSensorInfo::analogCrop::y
	 * are defined relatively to the active pixel area, while V4L2's
	 * TGT_CROP target is defined in respect to the full pixel array.
	 *
	 * Compensate it by subtracting the active area offset.
	 */
	info->analogCrop.x -= activeArea_.x;
	info->analogCrop.y -= activeArea_.y;

	/* The bit depth and image size depend on the currently applied format. */
	V4L2SubdeviceFormat format{};
	ret = subdev_->getFormat(pad_, &format);
	if (ret)
		return ret;
	info->bitsPerPixel = MediaBusFormatInfo::info(format.code).bitsPerPixel;
	info->outputSize = format.size;

	std::optional<int32_t> cfa = properties_.get(properties::draft::ColorFilterArrangement);
	info->cfaPattern = cfa ? *cfa : properties::draft::RGB;

	/*
	 * Retrieve the pixel rate, line length and minimum/maximum frame
	 * duration through V4L2 controls. Support for the V4L2_CID_PIXEL_RATE,
	 * V4L2_CID_HBLANK and V4L2_CID_VBLANK controls is mandatory.
	 */
	ControlList ctrls = subdev_->getControls({ V4L2_CID_PIXEL_RATE,
						   V4L2_CID_HBLANK,
						   V4L2_CID_VBLANK });
	if (ctrls.empty()) {
		LOG(CameraSensor, Error)
			<< "Failed to retrieve camera info controls";
		return -EINVAL;
	}

	info->pixelRate = ctrls.get(V4L2_CID_PIXEL_RATE).get<int64_t>();

	const ControlInfo hblank = ctrls.infoMap()->at(V4L2_CID_HBLANK);
	info->minLineLength = info->outputSize.width + hblank.min().get<int32_t>();
	info->maxLineLength = info->outputSize.width + hblank.max().get<int32_t>();

	const ControlInfo vblank = ctrls.infoMap()->at(V4L2_CID_VBLANK);
	info->minFrameLength = info->outputSize.height + vblank.min().get<int32_t>();
	info->maxFrameLength = info->outputSize.height + vblank.max().get<int32_t>();

	return 0;
}

Transform CameraSensorLegacy::computeTransform(Orientation *orientation) const
{
	/*
	 * If we cannot do any flips we cannot change the native camera mounting
	 * orientation.
	 */
	if (!supportFlips_) {
		*orientation = mountingOrientation_;
		return Transform::Identity;
	}

	/*
	 * Now compute the required transform to obtain 'orientation' starting
	 * from the mounting rotation.
	 *
	 * As a note:
	 * 	orientation / mountingOrientation_ = transform
	 * 	mountingOrientation_ * transform = orientation
	 */
	Transform transform = *orientation / mountingOrientation_;

	/*
	 * If transform contains any Transpose we cannot do it, so adjust
	 * 'orientation' to report the image native orientation and return Identity.
	 */
	if (!!(transform & Transform::Transpose)) {
		*orientation = mountingOrientation_;
		return Transform::Identity;
	}

	return transform;
}

BayerFormat::Order CameraSensorLegacy::bayerOrder(Transform t) const
{
	/* Return a defined by meaningless value for non-Bayer sensors. */
	if (!bayerFormat_)
		return BayerFormat::Order::BGGR;

	if (!flipsAlterBayerOrder_)
		return bayerFormat_->order;

	/*
	 * Apply the transform to the native (i.e. untransformed) Bayer order,
	 * using the rest of the Bayer format supplied by the caller.
	 */
	return bayerFormat_->transform(t).order;
}

const ControlInfoMap &CameraSensorLegacy::controls() const
{
	return subdev_->controls();
}

ControlList CameraSensorLegacy::getControls(const std::vector<uint32_t> &ids)
{
	return subdev_->getControls(ids);
}

int CameraSensorLegacy::setControls(ControlList *ctrls)
{
	return subdev_->setControls(ctrls);
}

int CameraSensorLegacy::setTestPatternMode(controls::draft::TestPatternModeEnum mode)
{
	if (testPatternMode_ == mode)
		return 0;

	if (testPatternModes_.empty()) {
		LOG(CameraSensor, Error)
			<< "Camera sensor does not support test pattern modes.";
		return -EINVAL;
	}

	return applyTestPatternMode(mode);
}

int CameraSensorLegacy::applyTestPatternMode(controls::draft::TestPatternModeEnum mode)
{
	if (testPatternModes_.empty())
		return 0;

	auto it = std::find(testPatternModes_.begin(), testPatternModes_.end(),
			    mode);
	if (it == testPatternModes_.end()) {
		LOG(CameraSensor, Error) << "Unsupported test pattern mode "
					 << mode;
		return -EINVAL;
	}

	LOG(CameraSensor, Debug) << "Apply test pattern mode " << mode;

	int32_t index = staticProps_->testPatternModes.at(mode);
	ControlList ctrls{ controls() };
	ctrls.set(V4L2_CID_TEST_PATTERN, index);

	int ret = setControls(&ctrls);
	if (ret)
		return ret;

	testPatternMode_ = mode;

	return 0;
}

std::string CameraSensorLegacy::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

REGISTER_CAMERA_SENSOR(CameraSensorLegacy, -100)

} /* namespace libcamera */
