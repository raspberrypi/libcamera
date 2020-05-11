/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_controls.cpp - V4L2 Controls Support
 */

#include "libcamera/internal/v4l2_controls.h"

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
	case V4L2_CTRL_TYPE_U8:
		return ControlTypeByte;

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
 * \brief Convenience specialisation of ControlInfo for V4L2 controls
 *
 * The V4L2ControlInfo class is a specialisation of the ControlInfo for V4L2
 * controls. It offers a convenience constructor from a struct
 * v4l2_query_ext_ctrl, and is otherwise equivalent to the ControlInfo class.
 */

/**
 * \brief Construct a V4L2ControlInfo from a struct v4l2_query_ext_ctrl
 * \param[in] ctrl The struct v4l2_query_ext_ctrl as returned by the kernel
 */
V4L2ControlInfo::V4L2ControlInfo(const struct v4l2_query_ext_ctrl &ctrl)
{
	switch (ctrl.type) {
	case V4L2_CTRL_TYPE_U8:
		ControlInfo::operator=(ControlInfo(static_cast<uint8_t>(ctrl.minimum),
						   static_cast<uint8_t>(ctrl.maximum),
						   static_cast<uint8_t>(ctrl.default_value)));
		break;

	case V4L2_CTRL_TYPE_BOOLEAN:
		ControlInfo::operator=(ControlInfo(static_cast<bool>(ctrl.minimum),
						   static_cast<bool>(ctrl.maximum),
						   static_cast<bool>(ctrl.default_value)));
		break;

	case V4L2_CTRL_TYPE_INTEGER64:
		ControlInfo::operator=(ControlInfo(static_cast<int64_t>(ctrl.minimum),
						   static_cast<int64_t>(ctrl.maximum),
						   static_cast<int64_t>(ctrl.default_value)));
		break;

	default:
		ControlInfo::operator=(ControlInfo(static_cast<int32_t>(ctrl.minimum),
						   static_cast<int32_t>(ctrl.maximum),
						   static_cast<int32_t>(ctrl.default_value)));
		break;
	}
}

} /* namespace libcamera */
