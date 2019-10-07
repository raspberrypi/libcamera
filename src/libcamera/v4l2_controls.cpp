/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_controls.cpp - V4L2 Controls Support
 */

#include "v4l2_controls.h"

#include <string.h>

/**
 * \file v4l2_controls.h
 * \brief Support for V4L2 Controls using the V4L2 Extended Controls APIs
 *
 * The V4L2 Control API allows application to inspect and modify sets of
 * configurable parameters on a video device or subdevice. The nature of the
 * parameters an application can modify using the control framework depends on
 * what the driver implements support for, and on the characteristics of the
 * underlying hardware platform. Generally controls are used to modify user
 * visible settings, such as the image brightness and exposure time, or
 * non-standard parameters which cannot be controlled through the V4L2 format
 * negotiation API.
 *
 * Controls are identified by a numerical ID, defined by the V4L2 kernel headers
 * and have an associated type. Each control has a value, which is the data that
 * can be modified with V4L2Device::setControls() or retrieved with
 * V4L2Device::getControls().
 *
 * The control's type along with the control's flags define the type of the
 * control's value content. Controls can transport a single data value stored in
 * variable inside the control, or they might as well deal with more complex
 * data types, such as arrays of matrices, stored in a contiguous memory
 * locations associated with the control and called 'the payload'. Such controls
 * are called 'compound controls' and are currently not supported by the
 * libcamera V4L2 control framework.
 *
 * libcamera implements support for controls using the V4L2 Extended Control
 * API, which allows future handling of controls with payloads of arbitrary
 * sizes.
 *
 * The libcamera V4L2 Controls framework operates on lists of controls, wrapped
 * by the ControlList class, to match the V4L2 extended controls API. The
 * interface to set and get control is implemented by the V4L2Device class, and
 * this file only provides the data type definitions.
 *
 * \todo Add support for compound controls
 */

namespace libcamera {

namespace {

std::string v4l2_ctrl_name(const struct v4l2_query_ext_ctrl &ctrl)
{
	size_t len = strnlen(ctrl.name, sizeof(ctrl.name));
	return std::string(static_cast<const char *>(ctrl.name), len);
}

ControlType v4l2_ctrl_type(const struct v4l2_query_ext_ctrl &ctrl)
{
	switch (ctrl.type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
		return ControlTypeBool;

	case V4L2_CTRL_TYPE_INTEGER:
		return ControlTypeInteger32;

	case V4L2_CTRL_TYPE_INTEGER64:
		return ControlTypeInteger64;

	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_BUTTON:
	case V4L2_CTRL_TYPE_BITMASK:
	case V4L2_CTRL_TYPE_INTEGER_MENU:
		/*
		 * More precise types may be needed, for now use a 32-bit
		 * integer type.
		 */
		return ControlTypeInteger32;

	default:
		return ControlTypeNone;
	}
}

} /* namespace */

/**
 * \class V4L2ControlId
 * \brief V4L2 control static metadata
 *
 * The V4L2ControlId class is a specialisation of the ControlId for V4L2
 * controls.
 */

/**
 * \brief Construct a V4L2ControlId from a struct v4l2_query_ext_ctrl
 * \param[in] ctrl The struct v4l2_query_ext_ctrl as returned by the kernel
 */
V4L2ControlId::V4L2ControlId(const struct v4l2_query_ext_ctrl &ctrl)
	: ControlId(ctrl.id, v4l2_ctrl_name(ctrl), v4l2_ctrl_type(ctrl))
{
}

/**
 * \class V4L2ControlInfo
 * \brief Information on a V4L2 control
 *
 * The V4L2ControlInfo class represents all the information related to a V4L2
 * control, such as its ID, its type, its user-readable name and the expected
 * size of its value data.
 *
 * V4L2ControlInfo instances are created by inspecting the fieldS of a struct
 * v4l2_query_ext_ctrl structure, after it has been filled by the device driver
 * as a consequence of a VIDIOC_QUERY_EXT_CTRL ioctl call.
 *
 * This class does not contain the control value, but only static information on
 * the control, which shall be cached by the caller at initialisation time or
 * the first time the control information is accessed.
 */

/**
 * \brief Construct a V4L2ControlInfo from a struct v4l2_query_ext_ctrl
 * \param[in] ctrl The struct v4l2_query_ext_ctrl as returned by the kernel
 */
V4L2ControlInfo::V4L2ControlInfo(const struct v4l2_query_ext_ctrl &ctrl)
	: id_(ctrl)
{
	size_ = ctrl.elem_size * ctrl.elems;

	if (ctrl.type == V4L2_CTRL_TYPE_INTEGER64)
		range_ = ControlRange(static_cast<int64_t>(ctrl.minimum),
				      static_cast<int64_t>(ctrl.maximum));
	else
		range_ = ControlRange(static_cast<int32_t>(ctrl.minimum),
				      static_cast<int32_t>(ctrl.maximum));
}

/**
 * \fn V4L2ControlInfo::id()
 * \brief Retrieve the control ID
 * \return The V4L2 control ID
 */

/**
 * \fn V4L2ControlInfo::size()
 * \brief Retrieve the control value data size (in bytes)
 * \return The V4L2 control value data size
 */

/**
 * \fn V4L2ControlInfo::range()
 * \brief Retrieve the control value range
 * \return The V4L2 control value range
 */

/**
 * \class V4L2ControlInfoMap
 * \brief A map of control ID to V4L2ControlInfo
 */

/**
 * \brief Move assignment operator from plain map
 * \param[in] info The control info map
 *
 * Populate the map by replacing its contents with those of \a info using move
 * semantics. Upon return the \a info map will be empty.
 *
 * This is the only supported way to populate a V4L2ControlInfoMap.
 *
 * \return The populated V4L2ControlInfoMap
 */
V4L2ControlInfoMap &V4L2ControlInfoMap::operator=(std::map<unsigned int, V4L2ControlInfo> &&info)
{
	std::map<unsigned int, V4L2ControlInfo>::operator=(std::move(info));

	idmap_.clear();
	for (const auto &ctrl : *this)
		idmap_[ctrl.first] = &ctrl.second.id();

	return *this;
}

/**
 * \fn const ControlIdMap &V4L2ControlInfoMap::idmap() const
 * \brief Retrieve the ControlId map
 *
 * Constructing ControlList instances for V4L2 controls requires a ControlIdMap
 * for the V4L2 device that the control list targets. This helper method
 * returns a suitable idmap for that purpose.
 *
 * \return The ControlId map
 */

/**
 * \class V4L2ControlList
 * \brief A list of controls for a V4L2 device
 *
 * This class specialises the ControList class for use with V4L2 devices. It
 * offers a convenience API to create a ControlList from a V4L2ControlInfoMap.
 *
 * V4L2ControlList allows easy construction of a ControlList containing V4L2
 * controls for a device. It can be used to construct the list of controls
 * passed to the V4L2Device::getControls() and V4L2Device::setControls()
 * methods. The class should however not be used in place of ControlList in
 * APIs.
 */

/**
 * \fn V4L2ControlList::V4L2ControlList(const V4L2ControlInfoMap &info)
 * \brief Construct a V4L2ControlList associated with a V4L2 device
 * \param[in] info The V4L2 device control info map
 */

} /* namespace libcamera */
