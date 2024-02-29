/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy.
 *
 * camera_sensor_raw.cpp - A raw camera sensor using the V4L2 streams API
 */

#include <algorithm>
#include <float.h>
#include <iomanip>
#include <limits.h>
#include <map>
#include <math.h>
#include <memory>
#include <optional>
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

class CameraSensorRaw : public CameraSensor, protected Loggable
{
public:
	CameraSensorRaw(const MediaEntity *entity);
	~CameraSensorRaw();

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

	V4L2Subdevice::Stream imageStream() const override;
	std::optional<V4L2Subdevice::Stream> embeddedDataStream() const override;
	V4L2SubdeviceFormat embeddedDataFormat() const override;
	int setEmbeddedDataEnabled(bool enable) override;

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
	LIBCAMERA_DISABLE_COPY(CameraSensorRaw)

	std::optional<int> init();
	int initProperties();
	void initStaticProperties();
	void initTestPatternModes();
	int applyTestPatternMode(controls::draft::TestPatternModeEnum mode);

	const MediaEntity *entity_;
	std::unique_ptr<V4L2Subdevice> subdev_;

	struct Streams {
		V4L2Subdevice::Stream sink;
		V4L2Subdevice::Stream source;
	};

	struct {
		Streams image;
		std::optional<Streams> edata;
	} streams_;

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
	BayerFormat::Order cfaPattern_;
	bool supportFlips_;
	bool flipsAlterBayerOrder_;
	Orientation mountingOrientation_;

	ControlList properties_;

	std::unique_ptr<CameraLens> focusLens_;
};

/**
 * \class CameraSensorRaw
 * \brief A camera sensor based on V4L2 subdevices
 *
 * This class supports single-subdev sensors with a single source pad and one
 * or two internal sink pads (for the image and embedded data streams).
 */

CameraSensorRaw::CameraSensorRaw(const MediaEntity *entity)
	: entity_(entity), staticProps_(nullptr), supportFlips_(false),
	  flipsAlterBayerOrder_(false), properties_(properties::properties)
{
}

CameraSensorRaw::~CameraSensorRaw() = default;

std::variant<std::unique_ptr<CameraSensor>, int>
CameraSensorRaw::match(MediaEntity *entity)
{
	/* Check the entity type. */
	if (entity->type() != MediaEntity::Type::V4L2Subdevice ||
	    entity->function() != MEDIA_ENT_F_CAM_SENSOR) {
		libcamera::LOG(CameraSensor, Debug)
			<< entity->name() << ": unsupported entity type ("
			<< utils::to_underlying(entity->type())
			<< ") or function (" << utils::hex(entity->function()) << ")";
		return { 0 };
	}

	/* Count and check the number of pads. */
	static constexpr uint32_t kPadFlagsMask = MEDIA_PAD_FL_SINK
						| MEDIA_PAD_FL_SOURCE
						| MEDIA_PAD_FL_INTERNAL;
	unsigned int numSinks = 0;
	unsigned int numSources = 0;

	for (const MediaPad *pad : entity->pads()) {
		switch (pad->flags() & kPadFlagsMask) {
		case MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_INTERNAL:
			numSinks++;
			break;

		case MEDIA_PAD_FL_SOURCE:
			numSources++;
			break;

		default:
			libcamera::LOG(CameraSensor, Debug)
				<< entity->name() << ": unsupported pad " << pad->index()
				<< " type " << utils::hex(pad->flags());
			return { 0 };
		}
	}

	if (numSinks < 1 || numSinks > 2 || numSources != 1) {
		libcamera::LOG(CameraSensor, Debug)
			<< entity->name() << ": unsupported number of sinks ("
			<< numSinks << ") or sources (" << numSources << ")";
		return { 0 };
	}

	/*
	 * The entity matches. Create the camera sensor and initialize it. The
	 * init() function will perform further match checks.
	 */
	std::unique_ptr<CameraSensorRaw> sensor =
		std::make_unique<CameraSensorRaw>(entity);

	std::optional<int> err = sensor->init();
	if (err)
		return { *err };

	return { std::move(sensor) };
}

std::optional<int> CameraSensorRaw::init()
{
	/* Create and open the subdev. */
	subdev_ = std::make_unique<V4L2Subdevice>(entity_);
	int ret = subdev_->open();
	if (ret)
		return { ret };

	/*
	 * 1. Identify the pads.
	 */

	/*
	 * First locate the source pad. The match() function guarantees there
	 * is one and only one source pad.
	 */
	unsigned int sourcePad = UINT_MAX;

	for (const MediaPad *pad : entity_->pads()) {
		if (pad->flags() & MEDIA_PAD_FL_SOURCE) {
			sourcePad = pad->index();
			break;
		}
	}

	/*
	 * Iterate over the routes to identify the streams on the source pad,
	 * and the internal sink pads.
	 */
	V4L2Subdevice::Routing routing = {};
	ret = subdev_->getRouting(&routing, V4L2Subdevice::TryFormat);
	if (ret)
		return { ret };

	bool imageStreamFound = false;

	for (const V4L2Subdevice::Route &route : routing) {
		if (route.source.pad != sourcePad) {
			LOG(CameraSensor, Error) << "Invalid route " << route;
			return { -EINVAL };
		}

		/* Identify the stream type based on the supported formats. */
		V4L2Subdevice::Formats formats = subdev_->formats(route.source);

		std::optional<MediaBusFormatInfo::Type> type;

		for (const auto &[code, sizes] : formats) {
			const MediaBusFormatInfo &info =
				MediaBusFormatInfo::info(code);
			if (info.isValid()) {
				type = info.type;
				break;
			}
		}

		if (!type) {
			LOG(CameraSensor, Warning)
				<< "No known format on pad " << route.source;
			continue;
		}

		switch (*type) {
		case MediaBusFormatInfo::Type::Image:
			if (imageStreamFound) {
				LOG(CameraSensor, Error)
					<< "Multiple internal image streams ("
					<< streams_.image.sink << " and "
					<< route.sink << ")";
				return { -EINVAL };
			}

			imageStreamFound = true;
			streams_.image.sink = route.sink;
			streams_.image.source = route.source;
			break;

		case MediaBusFormatInfo::Type::Metadata:
			/*
			 * Skip metadata streams that are not sensor embedded
			 * data. The source stream reports a generic metadata
			 * format, check the sink stream for the exact format.
			 */
			formats = subdev_->formats(route.sink);
			if (formats.size() != 1)
				continue;

			if (MediaBusFormatInfo::info(formats.cbegin()->first).type !=
			    MediaBusFormatInfo::Type::EmbeddedData)
				continue;

			if (streams_.edata) {
				LOG(CameraSensor, Error)
					<< "Multiple internal embedded data streams ("
					<< streams_.edata->sink << " and "
					<< route.sink << ")";
				return { -EINVAL };
			}

			streams_.edata = { route.sink, route.source };
			break;

		default:
			break;
		}
	}

	if (!imageStreamFound) {
		LOG(CameraSensor, Error) << "No image stream found";
		return { -EINVAL };
	}

	LOG(CameraSensor, Debug)
		<< "Found image stream " << streams_.image.sink
		<< " -> " << streams_.image.source;

	if (streams_.edata)
		LOG(CameraSensor, Debug)
			<< "Found embedded data stream " << streams_.edata->sink
			<< " -> " << streams_.edata->source;

	/*
	 * 2. Enumerate and cache the media bus codes, sizes and colour filter
	 * array order for the image stream.
	 */

	/*
	 * Get the native sensor CFA pattern. It is simpler to retrieve it from
	 * the internal image sink pad as it is guaranteed to expose a single
	 * format, and is not affected by flips.
	 */
	V4L2Subdevice::Formats formats = subdev_->formats(streams_.image.sink);
	if (formats.size() != 1) {
		LOG(CameraSensor, Error)
			<< "Image pad has " << formats.size()
			<< " formats, expected 1";
		return { -EINVAL };
	}

	uint32_t nativeFormat = formats.cbegin()->first;
	const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(nativeFormat);
	if (!bayerFormat.isValid()) {
		LOG(CameraSensor, Error)
			<< "Invalid native format " << nativeFormat;
		return { 0 };
	}

	cfaPattern_ = bayerFormat.order;

	/*
	 * Retrieve and cache the media bus codes and sizes on the source image
	 * stream.
	 */
	formats_ = subdev_->formats(streams_.image.source);
	if (formats_.empty()) {
		LOG(CameraSensor, Error) << "No image format found";
		return { -EINVAL };
	}

	/* Populate and sort the media bus codes and the sizes. */
	for (const auto &[code, ranges] : formats_) {
		/* Drop non-raw formats (in case we have a hybrid sensor). */
		const MediaBusFormatInfo &info = MediaBusFormatInfo::info(code);
		if (info.colourEncoding != PixelFormatInfo::ColourEncodingRAW)
			continue;

		mbusCodes_.push_back(code);
		std::transform(ranges.begin(), ranges.end(), std::back_inserter(sizes_),
			       [](const SizeRange &range) { return range.max; });
	}

	if (mbusCodes_.empty()) {
		LOG(CameraSensor, Debug) << "No raw image formats found";
		return { 0 };
	}

	std::sort(mbusCodes_.begin(), mbusCodes_.end());
	std::sort(sizes_.begin(), sizes_.end());

	/*
	 * Remove duplicate sizes. There are no duplicate media bus codes as
	 * they are the keys in the formats map.
	 */
	auto last = std::unique(sizes_.begin(), sizes_.end());
	sizes_.erase(last, sizes_.end());

	/*
	 * 3. Query selection rectangles. Retrieve properties, and verify that
	 * all the expected selection rectangles are supported.
	 */

	Rectangle rect;
	ret = subdev_->getSelection(streams_.image.sink, V4L2_SEL_TGT_CROP_BOUNDS,
				    &rect);
	if (ret) {
		LOG(CameraSensor, Error) << "No pixel array crop bounds";
		return { ret };
	}

	pixelArraySize_ = rect.size();

	ret = subdev_->getSelection(streams_.image.sink, V4L2_SEL_TGT_CROP_DEFAULT,
				    &activeArea_);
	if (ret) {
		LOG(CameraSensor, Error) << "No pixel array crop default";
		return { ret };
	}

	ret = subdev_->getSelection(streams_.image.sink, V4L2_SEL_TGT_CROP,
				    &rect);
	if (ret) {
		LOG(CameraSensor, Error) << "No pixel array crop rectangle";
		return { ret };
	}

	/*
	 * 4. Verify that all required controls are present.
	 */

	const ControlIdMap &controls = subdev_->controls().idmap();

	static constexpr uint32_t mandatoryControls[] = {
		V4L2_CID_ANALOGUE_GAIN,
		V4L2_CID_CAMERA_ORIENTATION,
		V4L2_CID_EXPOSURE,
		V4L2_CID_HBLANK,
		V4L2_CID_PIXEL_RATE,
		V4L2_CID_VBLANK,
	};

	ret = 0;

	for (uint32_t ctrl : mandatoryControls) {
		if (!controls.count(ctrl)) {
			LOG(CameraSensor, Error)
				<< "Mandatory V4L2 control " << utils::hex(ctrl)
				<< " not available";
			ret = -EINVAL;
		}
	}

	if (ret) {
		LOG(CameraSensor, Error)
			<< "The sensor kernel driver needs to be fixed";
		LOG(CameraSensor, Error)
			<< "See Documentation/sensor_driver_requirements.rst in the libcamera sources for more information";
		return { ret };
	}

	/*
	 * Verify if sensor supports horizontal/vertical flips
	 *
	 * \todo Handle horizontal and vertical flips independently.
	 */
	const struct v4l2_query_ext_ctrl *hflipInfo = subdev_->controlInfo(V4L2_CID_HFLIP);
	const struct v4l2_query_ext_ctrl *vflipInfo = subdev_->controlInfo(V4L2_CID_VFLIP);
	if (hflipInfo && !(hflipInfo->flags & V4L2_CTRL_FLAG_READ_ONLY) &&
	    vflipInfo && !(vflipInfo->flags & V4L2_CTRL_FLAG_READ_ONLY))
		supportFlips_ = true;

	if (!supportFlips_)
		LOG(CameraSensor, Debug)
			<< "Camera sensor does not support horizontal/vertical flip";

	/*
	 * 5. Discover ancillary devices.
	 *
	 * \todo This code may be shared by different V4L2 sensor classes.
	 */
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

	/*
	 * 6. Initialize properties.
	 */

	ret = initProperties();
	if (ret)
		return { ret };

	/*
	 * 7. Initialize controls.
	 */

	/*
	 * Set HBLANK to the minimum to start with a well-defined line length,
	 * allowing IPA modules that do not modify HBLANK to use the sensor
	 * minimum line length in their calculations.
	 *
	 * At present, there is no way of knowing if a control is read-only.
	 * As a workaround, assume that if the minimum and maximum values of
	 * the V4L2_CID_HBLANK control are the same, it implies the control
	 * is read-only.
	 *
	 * \todo The control API ought to have a flag to specify if a control
	 * is read-only which could be used below.
	 */
	const ControlInfoMap &ctrls = subdev_->controls();
	if (ctrls.find(V4L2_CID_HBLANK) != ctrls.end()) {
		const ControlInfo hblank = ctrls.at(V4L2_CID_HBLANK);
		const int32_t hblankMin = hblank.min().get<int32_t>();
		const int32_t hblankMax = hblank.max().get<int32_t>();

		if (hblankMin != hblankMax) {
			ControlList ctrl(subdev_->controls());

			ctrl.set(V4L2_CID_HBLANK, hblankMin);
			ret = subdev_->setControls(&ctrl);
			if (ret)
				return { ret };
		}
	}

	ret = applyTestPatternMode(controls::draft::TestPatternModeEnum::TestPatternModeOff);
	if (ret)
		return { ret };

	return {};
}

int CameraSensorRaw::initProperties()
{
	model_ = subdev_->model();
	properties_.set(properties::Model, utils::toAscii(model_));

	/* Generate a unique ID for the sensor. */
	id_ = sysfs::firmwareNodePath(subdev_->devicePath());
	if (id_.empty()) {
		LOG(CameraSensor, Error) << "Can't generate sensor ID";
		return -EINVAL;
	}

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

	/* Color filter array pattern. */
	uint32_t cfa;

	switch (cfaPattern_) {
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

	return 0;
}

void CameraSensorRaw::initStaticProperties()
{
	staticProps_ = CameraSensorProperties::get(model_);
	if (!staticProps_)
		return;

	/* Register the properties retrieved from the sensor database. */
	properties_.set(properties::UnitCellSize, staticProps_->unitCellSize);

	initTestPatternModes();
}

void CameraSensorRaw::initTestPatternModes()
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

std::vector<Size> CameraSensorRaw::sizes(unsigned int mbusCode) const
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

Size CameraSensorRaw::resolution() const
{
	return std::min(sizes_.back(), activeArea_.size());
}

V4L2SubdeviceFormat
CameraSensorRaw::getFormat(const std::vector<unsigned int> &mbusCodes,
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

int CameraSensorRaw::setFormat(V4L2SubdeviceFormat *format, Transform transform)
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
	int ret = subdev_->setFormat(streams_.image.source, format);
	if (ret)
		return ret;

	subdev_->updateControlInfo();
	return 0;
}

int CameraSensorRaw::tryFormat(V4L2SubdeviceFormat *format) const
{
	return subdev_->setFormat(streams_.image.source, format,
				  V4L2Subdevice::Whence::TryFormat);
}

int CameraSensorRaw::applyConfiguration(const SensorConfiguration &config,
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

V4L2Subdevice::Stream CameraSensorRaw::imageStream() const
{
	return streams_.image.source;
}

std::optional<V4L2Subdevice::Stream> CameraSensorRaw::embeddedDataStream() const
{
	if (!streams_.edata)
		return {};

	return { streams_.edata->source };
}

V4L2SubdeviceFormat CameraSensorRaw::embeddedDataFormat() const
{
	if (!streams_.edata)
		return {};

	V4L2SubdeviceFormat format;
	int ret = subdev_->getFormat(streams_.edata->source, &format);
	if (ret)
		return {};

	return format;
}

int CameraSensorRaw::setEmbeddedDataEnabled(bool enable)
{
	if (!streams_.edata)
		return enable ? -ENOSTR : 0;

	V4L2Subdevice::Routing routing{ 2 };

	routing[0].sink = streams_.image.sink;
	routing[0].source = streams_.image.source;
	routing[0].flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;

	routing[1].sink = streams_.edata->sink;
	routing[1].source = streams_.edata->source;
	routing[1].flags = enable ? V4L2_SUBDEV_ROUTE_FL_ACTIVE : 0;

	int ret = subdev_->setRouting(&routing);
	if (ret)
		return ret;

	/*
	 * Check if the embedded data stream has been enabled or disabled
	 * correctly. Assume at least one route will match the embedded data
	 * source stream, as there would be something seriously wrong
	 * otherwise.
	 */
	bool enabled = false;

	for (const V4L2Subdevice::Route &route : routing) {
		if (route.source != streams_.edata->source)
			continue;

		enabled = route.flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE;
		break;
	}

	if (enabled != enable)
		return enabled ? -EISCONN : -ENOSTR;

	return 0;
}

int CameraSensorRaw::sensorInfo(IPACameraSensorInfo *info) const
{
	info->model = model();

	/*
	 * The active area size is a static property, while the crop
	 * rectangle needs to be re-read as it depends on the sensor
	 * configuration.
	 */
	info->activeAreaSize = { activeArea_.width, activeArea_.height };

	int ret = subdev_->getSelection(streams_.image.sink, V4L2_SEL_TGT_CROP,
					&info->analogCrop);
	if (ret)
		return ret;

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
	ret = subdev_->getFormat(streams_.image.source, &format);
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

Transform CameraSensorRaw::computeTransform(Orientation *orientation) const
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

BayerFormat::Order CameraSensorRaw::bayerOrder(Transform t) const
{
	if (!flipsAlterBayerOrder_)
		return cfaPattern_;

	/*
	 * Apply the transform to the native (i.e. untransformed) Bayer order,
	 * using the rest of the Bayer format supplied by the caller.
	 */
	BayerFormat format{ cfaPattern_, 8, BayerFormat::Packing::None };
	return format.transform(t).order;
}

const ControlInfoMap &CameraSensorRaw::controls() const
{
	return subdev_->controls();
}

ControlList CameraSensorRaw::getControls(const std::vector<uint32_t> &ids)
{
	return subdev_->getControls(ids);
}

int CameraSensorRaw::setControls(ControlList *ctrls)
{
	return subdev_->setControls(ctrls);
}

int CameraSensorRaw::setTestPatternMode(controls::draft::TestPatternModeEnum mode)
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

int CameraSensorRaw::applyTestPatternMode(controls::draft::TestPatternModeEnum mode)
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

std::string CameraSensorRaw::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

REGISTER_CAMERA_SENSOR(CameraSensorRaw)

} /* namespace libcamera */
