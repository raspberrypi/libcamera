/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_sensor.cpp - A camera sensor
 */

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"

#include <algorithm>
#include <float.h>
#include <iomanip>
#include <limits.h>
#include <math.h>
#include <regex>
#include <string.h>

#include <libcamera/property_ids.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/sysfs.h"
#include "libcamera/internal/utils.h"

/**
 * \file camera_sensor.h
 * \brief A camera sensor
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraSensor)

/**
 * \struct CameraSensorInfo
 * \brief Report the image sensor characteristics
 *
 * The structure reports image sensor characteristics used by IPA modules to
 * tune their algorithms based on the image sensor model currently in use and
 * its configuration.
 *
 * The reported information describes the sensor's intrinsics characteristics,
 * such as its pixel array size and the sensor model name, as well as
 * information relative to the currently configured mode, such as the produced
 * image size and the bit depth of the requested image format.
 *
 * Instances of this structure are meant to be assembled by the CameraSensor
 * class by inspecting the sensor static properties as well as information
 * relative to the current configuration.
 */

/**
 * \var CameraSensorInfo::model
 * \brief The image sensor model name
 *
 * The sensor model name is a free-formed string that uniquely identifies the
 * sensor model.
 */

/**
 * \var CameraSensorInfo::bitsPerPixel
 * \brief The number of bits per pixel of the image format produced by the
 * image sensor
 */

/**
 * \var CameraSensorInfo::activeAreaSize
 * \brief The size of the pixel array active area of the sensor
 */

/**
 * \var CameraSensorInfo::analogCrop
 * \brief The portion of the pixel array active area which is read-out and
 * processed
 *
 * The analog crop rectangle top-left corner is defined as the displacement
 * from the top-left corner of the pixel array active area. The rectangle
 * horizontal and vertical sizes define the portion of the pixel array which
 * is read-out and provided to the sensor's internal processing pipeline, before
 * any pixel sub-sampling method, such as pixel binning, skipping and averaging
 * take place.
 */

/**
 * \var CameraSensorInfo::outputSize
 * \brief The size of the images produced by the camera sensor
 *
 * The output image size defines the horizontal and vertical sizes of the images
 * produced by the image sensor. The output image size is defined as the end
 * result of the sensor's internal image processing pipeline stages, applied on
 * the pixel array portion defined by the analog crop rectangle. Each image
 * processing stage that performs pixel sub-sampling techniques, such as pixel
 * binning or skipping, or perform any additional digital scaling concur in the
 * definition of the output image size.
 */

/**
 * \var CameraSensorInfo::pixelRate
 * \brief The number of pixels produced in a second
 *
 * To obtain the read-out time in seconds of a full line:
 *
 * \verbatim
	lineDuration(s) = lineLength(pixels) / pixelRate(pixels per second)
   \endverbatim
 */

/**
 * \var CameraSensorInfo::lineLength
 * \brief Total line length in pixels
 *
 * The total line length in pixel clock periods, including blanking.
 */

/**
 * \var CameraSensorInfo::minFrameLength
 * \brief The minimum allowable frame length in units of lines
 *
 * The sensor frame length comprises of active output lines and blanking lines
 * in a frame. The minimum frame length value dictates the minimum allowable
 * frame duration of the sensor mode.
 *
 * To obtain the minimum frame duration:
 *
 * \verbatim
	frameDuration(s) = minFrameLength(lines) * lineLength(pixels) / pixelRate(pixels per second)
   \endverbatim
 */

/**
 * \var CameraSensorInfo::maxFrameLength
 * \brief The maximum allowable frame length in units of lines
 *
 * The sensor frame length comprises of active output lines and blanking lines
 * in a frame. The maximum frame length value dictates the maximum allowable
 * frame duration of the sensor mode.
 *
 * To obtain the maximum frame duration:
 *
 * \verbatim
	frameDuration(s) = maxFrameLength(lines) * lineLength(pixels) / pixelRate(pixels per second)
   \endverbatim
 */

/**
 * \class CameraSensor
 * \brief A camera sensor based on V4L2 subdevices
 *
 * The CameraSensor class eases handling of sensors for pipeline handlers by
 * hiding the details of the V4L2 subdevice kernel API and caching sensor
 * information.
 *
 * The implementation is currently limited to sensors that expose a single V4L2
 * subdevice with a single pad. It will be extended to support more complex
 * devices as the needs arise.
 */

/**
 * \brief Construct a CameraSensor
 * \param[in] entity The media entity backing the camera sensor
 *
 * Once constructed the instance must be initialized with init().
 */
CameraSensor::CameraSensor(const MediaEntity *entity)
	: entity_(entity), pad_(UINT_MAX), bayerFormat_(nullptr),
	  properties_(properties::properties)
{
}

/**
 * \brief Destroy a CameraSensor
 */
CameraSensor::~CameraSensor()
{
}

/**
 * \brief Initialize the camera sensor instance
 *
 * This method performs the initialisation steps of the CameraSensor that may
 * fail. It shall be called once and only once after constructing the instance.
 *
 * \return 0 on success or a negative error code otherwise
 */
int CameraSensor::init()
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
		return initProperties();
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

	return 0;
}

int CameraSensor::validateSensorDriver()
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
	 * Make sure the required selection targets are supported.
	 *
	 * Failures in reading any of the targets are not deemed to be fatal,
	 * but some properties and features, like constructing a
	 * CameraSensorInfo for the IPA module, won't be supported.
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
			<< pixelArraySize_.toString();
		err = -EINVAL;
	} else {
		pixelArraySize_ = rect.size();
	}

	ret = subdev_->getSelection(pad_, V4L2_SEL_TGT_CROP_DEFAULT, &activeArea_);
	if (ret) {
		activeArea_ = Rectangle(pixelArraySize_);
		LOG(CameraSensor, Warning)
			<< "The PixelArrayActiveAreas property has been defaulted to "
			<< activeArea_.toString();
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

/*
 * \brief Initialize properties that cannot be intialized by the
 * regular initProperties() function for VIMC
 */
void CameraSensor::initVimcDefaultProperties()
{
	/* Use the largest supported size. */
	pixelArraySize_ = sizes_.back();
	activeArea_ = Rectangle(pixelArraySize_);
}

int CameraSensor::initProperties()
{
	/*
	 * Extract the camera sensor model name from the media entity name.
	 *
	 * There is no standardized naming scheme for sensor entities in the
	 * Linux kernel at the moment.
	 *
	 * - The most common rule, used by I2C sensors, associates the model
	 *   name with the I2C bus number and address (e.g. 'imx219 0-0010').
	 *
	 * - When the sensor exposes multiple subdevs, the model name is
	 *   usually followed by a function name, as in the smiapp driver (e.g.
	 *   'jt8ew9 pixel_array 0-0010').
	 *
	 * - The vimc driver names its sensors 'Sensor A' and 'Sensor B'.
	 *
	 * Other schemes probably exist. As a best effort heuristic, use the
	 * part of the entity name before the first space if the name contains
	 * an I2C address, and use the full entity name otherwise.
	 */
	std::string entityName = entity_->name();
	std::regex i2cRegex{ " [0-9]+-[0-9a-f]{4}" };
	std::smatch match;

	if (std::regex_search(entityName, match, i2cRegex))
		model_ = entityName.substr(0, entityName.find(' '));
	else
		model_ = entityName;

	properties_.set(properties::Model, utils::toAscii(model_));

	/* Generate a unique ID for the sensor. */
	int ret = generateId();
	if (ret)
		return ret;

	/* Retrieve and store the camera sensor properties. */
	const ControlInfoMap &controls = subdev_->controls();
	int32_t propertyValue;

	const auto &orientation = controls.find(V4L2_CID_CAMERA_ORIENTATION);
	if (orientation != controls.end()) {
		int32_t v4l2Orientation = orientation->second.def().get<int32_t>();

		switch (v4l2Orientation) {
		default:
			LOG(CameraSensor, Warning)
				<< "Unsupported camera location "
				<< v4l2Orientation << ", setting to External";
			/* Fall-through */
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
	} else {
		LOG(CameraSensor, Warning)
			<< "Failed to retrieve the camera location, setting to External";
		propertyValue = properties::CameraLocationExternal;
	}
	properties_.set(properties::Location, propertyValue);

	const auto &rotationControl = controls.find(V4L2_CID_CAMERA_SENSOR_ROTATION);
	if (rotationControl != controls.end()) {
		propertyValue = rotationControl->second.def().get<int32_t>();
		properties_.set(properties::Rotation, propertyValue);
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
		}

		properties_.set(properties::draft::ColorFilterArrangement, cfa);
	}

	return 0;
}

/**
 * \fn CameraSensor::model()
 * \brief Retrieve the sensor model name
 *
 * The sensor model name is a free-formed string that uniquely identifies the
 * sensor model.
 *
 * \return The sensor model name
 */

/**
 * \fn CameraSensor::id()
 * \brief Retrieve the sensor ID
 *
 * The sensor ID is a free-form string that uniquely identifies the sensor in
 * the system. The ID satisfies the requirements to be used as a camera ID.
 *
 * \return The sensor ID
 */

/**
 * \fn CameraSensor::entity()
 * \brief Retrieve the sensor media entity
 * \return The sensor media entity
 */

/**
 * \fn CameraSensor::mbusCodes()
 * \brief Retrieve the media bus codes supported by the camera sensor
 * \return The supported media bus codes sorted in increasing order
 */

/**
 * \fn CameraSensor::sizes()
 * \brief Retrieve the frame sizes supported by the camera sensor
 *
 * The reported sizes span all media bus codes supported by the camera sensor.
 * Not all sizes may be supported by all media bus codes.
 *
 * \return The supported frame sizes sorted in increasing order
 */

/**
 * \brief Retrieve the camera sensor resolution
 *
 * The camera sensor resolution is the active pixel area size, clamped to the
 * maximum frame size the sensor can produce if it is smaller than the active
 * pixel area.
 *
 * \todo Consider if it desirable to distinguish between the maximum resolution
 * the sensor can produce (also including upscaled ones) and the actual pixel
 * array size by splitting this method in two.
 *
 * \return The camera sensor resolution in pixels
 */
Size CameraSensor::resolution() const
{
	return std::min(sizes_.back(), activeArea_.size());
}

/**
 * \brief Retrieve the best sensor format for a desired output
 * \param[in] mbusCodes The list of acceptable media bus codes
 * \param[in] size The desired size
 *
 * Media bus codes are selected from \a mbusCodes, which lists all acceptable
 * codes in decreasing order of preference. Media bus codes supported by the
 * sensor but not listed in \a mbusCodes are ignored. If none of the desired
 * codes is supported, it returns an error.
 *
 * \a size indicates the desired size at the output of the sensor. This method
 * selects the best media bus code and size supported by the sensor according
 * to the following criteria.
 *
 * - The desired \a size shall fit in the sensor output size to avoid the need
 *   to up-scale.
 * - The sensor output size shall match the desired aspect ratio to avoid the
 *   need to crop the field of view.
 * - The sensor output size shall be as small as possible to lower the required
 *   bandwidth.
 * - The desired \a size shall be supported by one of the media bus code listed
 *   in \a mbusCodes.
 *
 * When multiple media bus codes can produce the same size, the code at the
 * lowest position in \a mbusCodes is selected.
 *
 * The use of this method is optional, as the above criteria may not match the
 * needs of all pipeline handlers. Pipeline handlers may implement custom
 * sensor format selection when needed.
 *
 * The returned sensor output format is guaranteed to be acceptable by the
 * setFormat() method without any modification.
 *
 * \return The best sensor output format matching the desired media bus codes
 * and size on success, or an empty format otherwise.
 */
V4L2SubdeviceFormat CameraSensor::getFormat(const std::vector<unsigned int> &mbusCodes,
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
		.mbus_code = bestCode,
		.size = *bestSize,
	};

	return format;
}

/**
 * \brief Set the sensor output format
 * \param[in] format The desired sensor output format
 * \return 0 on success or a negative error code otherwise
 */
int CameraSensor::setFormat(V4L2SubdeviceFormat *format)
{
	return subdev_->setFormat(pad_, format);
}

/**
 * \brief Retrieve the supported V4L2 controls and their information
 * \return A map of the V4L2 controls supported by the sensor
 */
const ControlInfoMap &CameraSensor::controls() const
{
	return subdev_->controls();
}

/**
 * \brief Read V4L2 controls from the sensor
 * \param[in] ids The list of controls to read, specified by their ID
 *
 * This method reads the value of all controls contained in \a ids, and returns
 * their values as a ControlList. The control identifiers are defined by the
 * V4L2 specification (V4L2_CID_*).
 *
 * If any control in \a ids is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), or if any other error occurs
 * during validation of the requested controls, no control is read and this
 * method returns an empty control list.
 *
 * \sa V4L2Device::getControls()
 *
 * \return The control values in a ControlList on success, or an empty list on
 * error
 */
ControlList CameraSensor::getControls(const std::vector<uint32_t> &ids)
{
	return subdev_->getControls(ids);
}

/**
 * \brief Write V4L2 controls to the sensor
 * \param[in] ctrls The list of controls to write
 *
 * This method writes the value of all controls contained in \a ctrls, and
 * stores the values actually applied to the device in the corresponding \a
 * ctrls entry. The control identifiers are defined by the V4L2 specification
 * (V4L2_CID_*).
 *
 * If any control in \a ctrls is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), is read-only, or if any other
 * error occurs during validation of the requested controls, no control is
 * written and this method returns -EINVAL.
 *
 * If an error occurs while writing the controls, the index of the first
 * control that couldn't be written is returned. All controls below that index
 * are written and their values are updated in \a ctrls, while all other
 * controls are not written and their values are not changed.
 *
 * \sa V4L2Device::setControls()
 *
 * \return 0 on success or an error code otherwise
 * \retval -EINVAL One of the control is not supported or not accessible
 * \retval i The index of the control that failed
 */
int CameraSensor::setControls(ControlList *ctrls)
{
	return subdev_->setControls(ctrls);
}

/**
 * \fn CameraSensor::device()
 * \brief Retrieve the camera sensor device
 * \todo Remove this function by integrating DelayedControl with CameraSensor
 * \return The camera sensor device
 */

/**
 * \fn CameraSensor::properties()
 * \brief Retrieve the camera sensor properties
 * \return The list of camera sensor properties
 */

/**
 * \brief Assemble and return the camera sensor info
 * \param[out] info The camera sensor info
 *
 * This function fills \a info with information that describes the camera sensor
 * and its current configuration. The information combines static data (such as
 * the the sensor model or active pixel array size) and data specific to the
 * current sensor configuration (such as the line length and pixel rate).
 *
 * Sensor information is only available for raw sensors. When called for a YUV
 * sensor, this function returns -EINVAL.
 *
 * \return 0 on success, a negative error code otherwise
 */
int CameraSensor::sensorInfo(CameraSensorInfo *info) const
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
		LOG(CameraSensor, Error)
			<< "The analogue crop rectangle has been defaulted to the active area size";
	}

	/*
	 * CameraSensorInfo::analogCrop::x and CameraSensorInfo::analogCrop::y
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
	info->bitsPerPixel = format.bitsPerPixel();
	info->outputSize = format.size;

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

	int32_t hblank = ctrls.get(V4L2_CID_HBLANK).get<int32_t>();
	info->lineLength = info->outputSize.width + hblank;
	info->pixelRate = ctrls.get(V4L2_CID_PIXEL_RATE).get<int64_t>();

	const ControlInfo vblank = ctrls.infoMap()->at(V4L2_CID_VBLANK);
	info->minFrameLength = info->outputSize.height + vblank.min().get<int32_t>();
	info->maxFrameLength = info->outputSize.height + vblank.max().get<int32_t>();

	return 0;
}

std::string CameraSensor::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

int CameraSensor::generateId()
{
	const std::string devPath = subdev_->devicePath();

	/* Try to get ID from firmware description. */
	id_ = sysfs::firmwareNodePath(devPath);
	if (!id_.empty())
		return 0;

	/*
	 * Virtual sensors not described in firmware
	 *
	 * Verify it's a platform device and construct ID from the deive path
	 * and model of sensor.
	 */
	if (devPath.find("/sys/devices/platform/", 0) == 0) {
		id_ = devPath.substr(strlen("/sys/devices/")) + " " + model();
		return 0;
	}

	LOG(CameraSensor, Error) << "Can't generate sensor ID";
	return -EINVAL;
}

} /* namespace libcamera */
