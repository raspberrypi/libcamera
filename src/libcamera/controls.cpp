/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * controls.cpp - Control handling
 */

#include <libcamera/controls.h>

#include <sstream>
#include <string>

#include "log.h"
#include "utils.h"

/**
 * \file controls.h
 * \brief Describes control framework and controls supported by a camera
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Controls)

/**
 * \enum ControlValueType
 * \brief Define the data type of value represented by a ControlValue
 * \var ControlValueNone
 * Identifies an unset control value
 * \var ControlValueBool
 * Identifies controls storing a boolean value
 * \var ControlValueInteger
 * Identifies controls storing an integer value
 * \var ControlValueInteger64
 * Identifies controls storing a 64-bit integer value
 */

/**
 * \class ControlValue
 * \brief Abstract type representing the value of a control
 */

/**
 * \brief Construct an empty ControlValue.
 */
ControlValue::ControlValue()
	: type_(ControlValueNone)
{
}

/**
 * \brief Construct a Boolean ControlValue
 * \param[in] value Boolean value to store
 */
ControlValue::ControlValue(bool value)
	: type_(ControlValueBool), bool_(value)
{
}

/**
 * \brief Construct an integer ControlValue
 * \param[in] value Integer value to store
 */
ControlValue::ControlValue(int value)
	: type_(ControlValueInteger), integer_(value)
{
}

/**
 * \brief Construct a 64 bit integer ControlValue
 * \param[in] value Integer value to store
 */
ControlValue::ControlValue(int64_t value)
	: type_(ControlValueInteger64), integer64_(value)
{
}

/**
 * \fn ControlValue::type()
 * \brief Retrieve the data type of the value
 * \return The value data type
 */

/**
 * \fn ControlValue::isNone()
 * \brief Determine if the value is not initialised
 * \return True if the value type is ControlValueNone, false otherwise
 */

/**
 * \brief Set the value with a boolean
 * \param[in] value Boolean value to store
 */
void ControlValue::set(bool value)
{
	type_ = ControlValueBool;
	bool_ = value;
}

/**
 * \brief Set the value with an integer
 * \param[in] value Integer value to store
 */
void ControlValue::set(int value)
{
	type_ = ControlValueInteger;
	integer_ = value;
}

/**
 * \brief Set the value with a 64 bit integer
 * \param[in] value 64 bit integer value to store
 */
void ControlValue::set(int64_t value)
{
	type_ = ControlValueInteger64;
	integer64_ = value;
}

/**
 * \brief Get the boolean value
 *
 * The value type must be Boolean.
 *
 * \return The boolean value
 */
bool ControlValue::getBool() const
{
	ASSERT(type_ == ControlValueBool);

	return bool_;
}

/**
 * \brief Get the integer value
 *
 * The value type must be Integer or Integer64.
 *
 * \return The integer value
 */
int ControlValue::getInt() const
{
	ASSERT(type_ == ControlValueInteger || type_ == ControlValueInteger64);

	return integer_;
}

/**
 * \brief Get the 64-bit integer value
 *
 * The value type must be Integer or Integer64.
 *
 * \return The 64-bit integer value
 */
int64_t ControlValue::getInt64() const
{
	ASSERT(type_ == ControlValueInteger || type_ == ControlValueInteger64);

	return integer64_;
}

/**
 * \brief Assemble and return a string describing the value
 * \return A string describing the ControlValue
 */
std::string ControlValue::toString() const
{
	switch (type_) {
	case ControlValueNone:
		return "<None>";
	case ControlValueBool:
		return bool_ ? "True" : "False";
	case ControlValueInteger:
		return std::to_string(integer_);
	case ControlValueInteger64:
		return std::to_string(integer64_);
	}

	return "<ValueType Error>";
}

/**
 * \enum ControlId
 * \brief Numerical control ID
 */

/**
 * \struct ControlIdentifier
 * \brief Describe a ControlId with control specific constant meta-data
 *
 * Defines a Control with a unique ID, a name, and a type.
 * This structure is used as static part of the auto-generated control
 * definitions, which are generated from the ControlId documentation.
 *
 * \var ControlIdentifier::id
 * The unique ID for a control
 * \var ControlIdentifier::name
 * The string representation of the control
 * \var ControlIdentifier::type
 * The ValueType required to represent the control value
 */

/*
 * The controlTypes are automatically generated to produce a control_types.cpp
 * output. This file is not for public use, and so no suitable header exists
 * for this sole usage of the controlTypes reference. As such the extern is
 * only defined here for use during the ControlInfo constructor and should not
 * be referenced directly elsewhere.
 */
extern const std::unordered_map<ControlId, ControlIdentifier> controlTypes;

/**
 * \class ControlInfo
 * \brief Describe the information and capabilities of a Control
 *
 * The ControlInfo represents control specific meta-data which is constant on a
 * per camera basis. ControlInfo classes are constructed by pipeline handlers
 * to expose the controls they support and the metadata needed to utilise those
 * controls.
 */

/**
 * \brief Construct a ControlInfo with minimum and maximum range parameters
 * \param[in] id The control ID
 * \param[in] min The control minimum value
 * \param[in] max The control maximum value
 */
ControlInfo::ControlInfo(ControlId id, const ControlValue &min,
			 const ControlValue &max)
	: min_(min), max_(max)
{
	auto iter = controlTypes.find(id);
	if (iter == controlTypes.end()) {
		LOG(Controls, Fatal) << "Attempt to create invalid ControlInfo";
		return;
	}

	ident_ = &iter->second;
}

/**
 * \fn ControlInfo::id()
 * \brief Retrieve the control ID
 * \return The control ID
 */

/**
 * \fn ControlInfo::name()
 * \brief Retrieve the control name string
 * \return The control name string
 */

/**
 * \fn ControlInfo::type()
 * \brief Retrieve the control data type
 * \return The control data type
 */

/**
 * \fn ControlInfo::min()
 * \brief Retrieve the minimum value of the control
 * \return A ControlValue with the minimum value for the control
 */

/**
 * \fn ControlInfo::max()
 * \brief Retrieve the maximum value of the control
 * \return A ControlValue with the maximum value for the control
 */

/**
 * \brief Provide a string representation of the ControlInfo
 */
std::string ControlInfo::toString() const
{
	std::stringstream ss;

	ss << name() << "[" << min_.toString() << ".." << max_.toString() << "]";

	return ss.str();
}

/**
 * \brief Compare control information for equality
 * \param[in] lhs Left-hand side control information
 * \param[in] rhs Right-hand side control information
 *
 * Control information is compared based on the ID only, as a camera may not
 * have two separate controls with the same ID.
 *
 * \return True if \a lhs and \a rhs are equal, false otherwise
 */
bool operator==(const ControlInfo &lhs, const ControlInfo &rhs)
{
	return lhs.id() == rhs.id();
}

/**
 * \brief Compare control ID and information for equality
 * \param[in] lhs Left-hand side control identifier
 * \param[in] rhs Right-hand side control information
 *
 * Control information is compared based on the ID only, as a camera may not
 * have two separate controls with the same ID.
 *
 * \return True if \a lhs and \a rhs are equal, false otherwise
 */
bool operator==(const ControlId &lhs, const ControlInfo &rhs)
{
	return lhs == rhs.id();
}

/**
 * \brief Compare control information and ID for equality
 * \param[in] lhs Left-hand side control information
 * \param[in] rhs Right-hand side control identifier
 *
 * Control information is compared based on the ID only, as a camera may not
 * have two separate controls with the same ID.
 *
 * \return True if \a lhs and \a rhs are equal, false otherwise
 */
bool operator==(const ControlInfo &lhs, const ControlId &rhs)
{
	return lhs.id() == rhs;
}

/**
 * \typedef ControlInfoMap
 * \brief A map of ControlId to ControlInfo
 */

/**
 * \class ControlList
 * \brief Associate a list of ControlId with their values for a camera
 *
 * A ControlList wraps a map of ControlId to ControlValue and provide
 * additional validation against the control information exposed by a Camera.
 *
 * A list is only valid for as long as the camera it refers to is valid. After
 * that calling any method of the ControlList class other than its destructor
 * will cause undefined behaviour.
 */

/**
 * \brief Construct a ControlList with a reference to the Camera it applies on
 * \param[in] camera The camera
 */
ControlList::ControlList(Camera *camera)
	: camera_(camera)
{
}

/**
 * \typedef ControlList::iterator
 * \brief Iterator for the controls contained within the list
 */

/**
 * \typedef ControlList::const_iterator
 * \brief Const iterator for the controls contained within the list
 */

/**
 * \fn iterator ControlList::begin()
 * \brief Retrieve an iterator to the first Control in the list
 * \return An iterator to the first Control in the list
 */

/**
 * \fn const_iterator ControlList::begin() const
 * \brief Retrieve a const_iterator to the first Control in the list
 * \return A const_iterator to the first Control in the list
 */

/**
 * \fn iterator ControlList::end()
 * \brief Retrieve an iterator pointing to the past-the-end control in the list
 * \return An iterator to the element following the last control in the list
 */

/**
 * \fn const_iterator ControlList::end() const
 * \brief Retrieve a const iterator pointing to the past-the-end control in the
 * list
 * \return A const iterator to the element following the last control in the
 * list
 */

/**
 * \brief Check if the list contains a control with the specified \a info
 * \param[in] info The control info
 * \return True if the list contains a matching control, false otherwise
 */
bool ControlList::contains(const ControlInfo *info) const
{
	return controls_.find(info) != controls_.end();
}

/**
 * \fn ControlList::empty()
 * \brief Identify if the list is empty
 * \return True if the list does not contain any control, false otherwise
 */

/**
 * \fn ControlList::size()
 * \brief Retrieve the number of controls in the list
 * \return The number of Control entries stored in the list
 */

/**
 * \fn ControlList::clear()
 * \brief Removes all controls from the list
 */

/**
 * \fn ControlList::operator[](const ControlInfo *info)
 * \brief Access or insert the control specified by \a info
 * \param[in] info The control info
 *
 * This method returns a reference to the control identified by \a info,
 * inserting it in the list if the info is not already present.
 *
 * \return A reference to the value of the control identified by \a info
 */

/**
 * \brief Update the list with a union of itself and \a other
 * \param other The other list
 *
 * Update the control list to include all values from the \a other list.
 * Elements in the list whose control IDs are contained in \a other are updated
 * with the value from \a other. Elements in the \a other list that have no
 * corresponding element in the list are added to the list with their value.
 *
 * The behaviour is undefined if the two lists refer to different Camera
 * instances.
 */
void ControlList::update(const ControlList &other)
{
	if (other.camera_ != camera_) {
		LOG(Controls, Error)
			<< "Can't update ControlList from a different camera";
		return;
	}

	for (auto it : other) {
		const ControlInfo *info = it.first;
		const ControlValue &value = it.second;

		controls_[info] = value;
	}
}

} /* namespace libcamera */
