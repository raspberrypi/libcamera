/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_sensor.cpp - A camera sensor
 */

#include "camera_sensor.h"

#include <algorithm>
#include <float.h>
#include <iomanip>
#include <limits.h>
#include <math.h>

#include "formats.h"
#include "v4l2_subdevice.h"

/**
 * \file camera_sensor.h
 * \brief A camera sensor
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraSensor);

/**
 * \class CameraSensor
 * \brief A camera sensor based on V4L2 subdevices
 *
 * The CameraSensor class eases handling of sensors for pipeline handlers by
 * hiding the details of the V4L2 subdevice kernel API and caching sensor
 * information.
 *
 * The implementation is currently limited to sensors that expose a single V4L2
 * subdevice with a single pad, and support the same frame sizes for all
 * supported media bus codes. It will be extended to support more complex
 * devices as the needs arise.
 */

/**
 * \brief Construct a CameraSensor
 * \param[in] entity The media entity backing the camera sensor
 *
 * Once constructed the instance must be initialized with init().
 */
CameraSensor::CameraSensor(const MediaEntity *entity)
	: entity_(entity)
{
	subdev_ = new V4L2Subdevice(entity);
}

/**
 * \brief Destroy a CameraSensor
 */
CameraSensor::~CameraSensor()
{
	delete subdev_;
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
	int ret;

	if (entity_->pads().size() != 1) {
		LOG(CameraSensor, Error)
			<< "Sensors with more than one pad are not supported";
		return -EINVAL;
	}

	if (entity_->function() != MEDIA_ENT_F_CAM_SENSOR) {
		LOG(CameraSensor, Error)
			<< "Invalid sensor function 0x"
			<< std::hex << std::setfill('0') << std::setw(8)
			<< entity_->function();
		return -EINVAL;
	}

	ret = subdev_->open();
	if (ret < 0)
		return ret;

	/* Enumerate and cache media bus codes and sizes. */
	const ImageFormats formats = subdev_->formats(0);
	if (formats.isEmpty()) {
		LOG(CameraSensor, Error) << "No image format found";
		return -EINVAL;
	}

	mbusCodes_ = formats.formats();

	/*
	 * Extract the supported sizes from the first format as we only support
	 * sensors that offer the same frame sizes for all media bus codes.
	 * Verify this assumption and reject the sensor if it isn't true.
	 */
	const std::vector<SizeRange> &sizes = formats.sizes(mbusCodes_[0]);
	std::transform(sizes.begin(), sizes.end(), std::back_inserter(sizes_),
		       [](const SizeRange &range) { return range.max; });

	for (unsigned int code : mbusCodes_) {
		if (formats.sizes(code) != sizes) {
			LOG(CameraSensor, Error)
				<< "Frame sizes differ between media bus codes";
			return -EINVAL;
		}
	}

	/* Sort the media bus codes and sizes. */
	std::sort(mbusCodes_.begin(), mbusCodes_.end());
	std::sort(sizes_.begin(), sizes_.end());

	return 0;
}

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
 * \return The supported frame sizes sorted in increasing order
 */

/**
 * \brief Retrieve the camera sensor resolution
 * \return The camera sensor resolution in pixels
 */
const Size &CameraSensor::resolution() const
{
	/*
	 * The sizes_ vector is sorted in ascending order, the resolution is
	 * thus the last element of the vector.
	 */
	return sizes_.back();
}

/**
 * \brief Retrieve the best sensor format for a desired output
 * \param[in] mbusCodes The list of acceptable media bus codes
 * \param[in] size The desired size
 *
 * Media bus codes are selected from \a mbusCodes, which lists all acceptable
 * codes in decreasing order of preference. This method selects the first code
 * from the list that is supported by the sensor. If none of the desired codes
 * is supported, it returns an error.
 *
 * \a size indicates the desired size at the output of the sensor. This method
 * selects the best size supported by the sensor according to the following
 * criteria.
 *
 * - The desired \a size shall fit in the sensor output size to avoid the need
 *   to up-scale.
 * - The sensor output size shall match the desired aspect ratio to avoid the
 *   need to crop the field of view.
 * - The sensor output size shall be as small as possible to lower the required
 *   bandwidth.
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
	V4L2SubdeviceFormat format{};

	for (unsigned int code : mbusCodes) {
		if (std::any_of(mbusCodes_.begin(), mbusCodes_.end(),
				[code](unsigned int c) { return c == code; })) {
			format.mbus_code = code;
			break;
		}
	}

	if (!format.mbus_code) {
		LOG(CameraSensor, Debug) << "No supported format found";
		return format;
	}

	unsigned int desiredArea = size.width * size.height;
	unsigned int bestArea = UINT_MAX;
	float desiredRatio = static_cast<float>(size.width) / size.height;
	float bestRatio = FLT_MAX;
	const Size *bestSize = nullptr;

	for (const Size &sz : sizes_) {
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
		}
	}

	if (!bestSize) {
		LOG(CameraSensor, Debug) << "No supported size found";
		return format;
	}

	format.size = *bestSize;

	return format;
}

/**
 * \brief Set the sensor output format
 * \param[in] format The desired sensor output format
 * \return 0 on success or a negative error code otherwise
 */
int CameraSensor::setFormat(V4L2SubdeviceFormat *format)
{
	return subdev_->setFormat(0, format);
}

/**
 * \brief Retrieve the supported V4L2 controls and their information
 * \return A map of the V4L2 controls supported by the sensor
 */
const V4L2ControlInfoMap &CameraSensor::controls() const
{
	return subdev_->controls();
}

/**
 * \brief Read controls from the sensor
 * \param[inout] ctrls The list of controls to read
 *
 * This method reads the value of all controls contained in \a ctrls, and stores
 * their values in the corresponding \a ctrls entry.
 *
 * If any control in \a ctrls is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), is a compound control, or if any
 * other error occurs during validation of the requested controls, no control is
 * read and this method returns -EINVAL.
 *
 * If an error occurs while reading the controls, the index of the first control
 * that couldn't be read is returned. The value of all controls below that index
 * are updated in \a ctrls, while the value of all the other controls are not
 * changed.
 *
 * \sa V4L2Device::getControls()
 *
 * \return 0 on success or an error code otherwise
 * \retval -EINVAL One of the control is not supported or not accessible
 * \retval i The index of the control that failed
 */
int CameraSensor::getControls(ControlList *ctrls)
{
	return subdev_->getControls(ctrls);
}

/**
 * \brief Write controls to the sensor
 * \param[in] ctrls The list of controls to write
 *
 * This method writes the value of all controls contained in \a ctrls, and
 * stores the values actually applied to the device in the corresponding
 * \a ctrls entry.
 *
 * If any control in \a ctrls is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), is read-only, is a
 * compound control, or if any other error occurs during validation of
 * the requested controls, no control is written and this method returns
 * -EINVAL.
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

std::string CameraSensor::logPrefix() const
{
	return "'" + subdev_->entity()->name() + "'";
}

} /* namespace libcamera */
