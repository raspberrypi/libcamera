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
 * by the V4L2ControlList class, to match the V4L2 extended controls API. The
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
	type_ = ctrl.type;
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
 * \fn V4L2ControlInfo::type()
 * \brief Retrieve the control type as defined by V4L2_CTRL_TYPE_*
 * \return The V4L2 control type
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
 * \typedef V4L2ControlInfoMap
 * \brief A map of control ID to V4L2ControlInfo
 */

/**
 * \class V4L2Control
 * \brief A V4L2 control value
 *
 * The V4L2Control class represent the value of a V4L2 control. The class
 * stores values that have been read from or will be applied to a V4L2 device.
 *
 * The value stored in the class instances does not reflect what is actually
 * applied to the hardware but is a pure software cache optionally initialized
 * at control creation time and modified by a control read or write operation.
 *
 * The write and read controls the V4L2Control class instances are not meant
 * to be directly used but are instead intended to be grouped in
 * V4L2ControlList instances, which are then passed as parameters to
 * V4L2Device::setControls() and V4L2Device::getControls() operations.
 */

/**
 * \fn V4L2Control::V4L2Control
 * \brief Construct a V4L2 control with \a id and \a value
 * \param id The V4L2 control ID
 * \param value The control value
 */

/**
 * \fn V4L2Control::value() const
 * \brief Retrieve the value of the control
 *
 * This method is a const version of V4L2Control::value(), returning a const
 * reference to the value.
 *
 * \return The V4L2 control value
 */

/**
 * \fn V4L2Control::value()
 * \brief Retrieve the value of the control
 *
 * This method returns the cached control value, initially set by
 * V4L2ControlList::add() and then updated when the controls are read or
 * written with V4L2Device::getControls() and V4L2Device::setControls().
 *
 * \return The V4L2 control value
 */

/**
 * \fn V4L2Control::id()
 * \brief Retrieve the control ID this instance refers to
 * \return The V4L2Control ID
 */

/**
 * \class V4L2ControlList
 * \brief Container of V4L2Control instances
 *
 * The V4L2ControlList class works as a container for a list of V4L2Control
 * instances. The class provides operations to add a new control to the list,
 * get back a control value, and reset the list of controls it contains.
 *
 * In order to set and get controls, user of the libcamera V4L2 control
 * framework should operate on instances of the V4L2ControlList class, and use
 * them as argument for the V4L2Device::setControls() and
 * V4L2Device::getControls() operations, which write and read a list of
 * controls to or from a V4L2 device (a video device or a subdevice).
 *
 * Controls are added to a V4L2ControlList instance with the add() method, with
 * or without a value.
 *
 * To write controls to a device, the controls of interest shall be added with
 * an initial value by calling V4L2ControlList::add(unsigned int id, int64_t
 * value) to prepare for a write operation. Once the values of all controls of
 * interest have been added, the V4L2ControlList instance is passed to the
 * V4L2Device::setControls(), which sets the controls on the device.
 *
 * To read controls from a device, the desired controls are added with
 * V4L2ControlList::add(unsigned int id) to prepare for a read operation. The
 * V4L2ControlList instance is then passed to V4L2Device::getControls(), which
 * reads the controls from the device and updates the values stored in
 * V4L2ControlList.
 *
 * V4L2ControlList instances can be reset to remove all controls they contain
 * and prepare to be re-used for a new control write/read sequence.
 */

/**
 * \typedef V4L2ControlList::iterator
 * \brief Iterator on the V4L2 controls contained in the instance
 */

/**
 * \typedef V4L2ControlList::const_iterator
 * \brief Const iterator on the V4L2 controls contained in the instance
 */

/**
 * \fn iterator V4L2ControlList::begin()
 * \brief Retrieve an iterator to the first V4L2Control in the instance
 * \return An iterator to the first V4L2 control
 */

/**
 * \fn const_iterator V4L2ControlList::begin() const
 * \brief Retrieve a constant iterator to the first V4L2Control in the instance
 * \return A constant iterator to the first V4L2 control
 */

/**
 * \fn iterator V4L2ControlList::end()
 * \brief Retrieve an iterator pointing to the past-the-end V4L2Control in the
 * instance
 * \return An iterator to the element following the last V4L2 control in the
 * instance
 */

/**
 * \fn const_iterator V4L2ControlList::end() const
 * \brief Retrieve a constant iterator pointing to the past-the-end V4L2Control
 * in the instance
 * \return A constant iterator to the element following the last V4L2 control
 * in the instance
 */

/**
 * \fn V4L2ControlList::empty()
 * \brief Verify if the instance does not contain any control
 * \return True if the instance does not contain any control, false otherwise
 */

/**
 * \fn V4L2ControlList::size()
 * \brief Retrieve the number on controls in the instance
 * \return The number of V4L2Control stored in the instance
 */

/**
 * \fn V4L2ControlList::clear()
 * \brief Remove all controls in the instance
 */

/**
 * \brief Add control with \a id and optional \a value to the instance
 * \param id The V4L2 control ID (V4L2_CID_*)
 * \param value The V4L2 control value
 *
 * This method adds a new V4L2 control to the V4L2ControlList. The newly
 * inserted control shall not already be present in the control lists, otherwise
 * this method, and further use of the control list lead to undefined behaviour.
 */
void V4L2ControlList::add(unsigned int id, int64_t value)
{
	controls_.emplace_back(id, value);
}

/**
 * \brief Retrieve the control at \a index
 * \param[in] index The control index
 *
 * Controls are stored in a V4L2ControlList in order of insertion and this
 * method retrieves the control at \a index.
 *
 * \return A pointer to the V4L2Control at \a index or nullptr if the
 * index is larger than the number of controls
 */
V4L2Control *V4L2ControlList::getByIndex(unsigned int index)
{
	if (index >= controls_.size())
		return nullptr;

	return &controls_[index];
}

/**
 * \brief Retrieve the control with \a id
 * \param[in] id The V4L2 control ID (V4L2_CID_xx)
 * \return A pointer to the V4L2Control with \a id or nullptr if the
 * control ID is not part of this instance.
 */
V4L2Control *V4L2ControlList::operator[](unsigned int id)
{
	for (V4L2Control &ctrl : controls_) {
		if (ctrl.id() == id)
			return &ctrl;
	}

	return nullptr;
}

} /* namespace libcamera */
