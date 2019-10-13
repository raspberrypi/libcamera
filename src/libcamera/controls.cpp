/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * controls.cpp - Control handling
 */

#include <libcamera/controls.h>

#include <iomanip>
#include <sstream>
#include <string>

#include "control_validator.h"
#include "log.h"
#include "utils.h"

/**
 * \file controls.h
 * \brief Framework to manage controls related to an object
 *
 * A control is a mean to govern or influence the operation of an object, and in
 * particular of a camera. Every control is defined by a unique numerical ID, a
 * name string and the data type of the value it stores. The libcamera API
 * defines a set of standard controls in the libcamera::controls namespace, as
 * a set of instances of the Control class.
 *
 * The main way for applications to interact with controls is through the
 * ControlList stored in the Request class:
 *
 * \code{.cpp}
 * Request *req = ...;
 * ControlList &controls = req->controls();
 * controls->set(controls::AwbEnable, false);
 * controls->set(controls::ManualExposure, 1000);
 *
 * ...
 *
 * int32_t exposure = controls->get(controls::ManualExposure);
 * \endcode
 *
 * The ControlList::get() and ControlList::set() methods automatically deduce
 * the data type based on the control.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Controls)

/**
 * \enum ControlType
 * \brief Define the data type of a Control
 * \var ControlTypeNone
 * Invalid type, for empty values
 * \var ControlTypeBool
 * The control stores a boolean value
 * \var ControlTypeInteger32
 * The control stores a 32-bit integer value
 * \var ControlTypeInteger64
 * The control stores a 64-bit integer value
 */

/**
 * \class ControlValue
 * \brief Abstract type representing the value of a control
 */

/**
 * \brief Construct an empty ControlValue.
 */
ControlValue::ControlValue()
	: type_(ControlTypeNone)
{
}

/**
 * \brief Construct a Boolean ControlValue
 * \param[in] value Boolean value to store
 */
ControlValue::ControlValue(bool value)
	: type_(ControlTypeBool), bool_(value)
{
}

/**
 * \brief Construct an integer ControlValue
 * \param[in] value Integer value to store
 */
ControlValue::ControlValue(int32_t value)
	: type_(ControlTypeInteger32), integer32_(value)
{
}

/**
 * \brief Construct a 64 bit integer ControlValue
 * \param[in] value Integer value to store
 */
ControlValue::ControlValue(int64_t value)
	: type_(ControlTypeInteger64), integer64_(value)
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
 * \return True if the value type is ControlTypeNone, false otherwise
 */

/**
 * \fn template<typename T> const T &ControlValue::get() const
 * \brief Get the control value
 *
 * The control value type shall match the type T, otherwise the behaviour is
 * undefined.
 *
 * \return The control value
 */

/**
 * \fn template<typename T> void ControlValue::set(const T &value)
 * \brief Set the control value to \a value
 * \param[in] value The control value
 */

#ifndef __DOXYGEN__
template<>
const bool &ControlValue::get<bool>() const
{
	ASSERT(type_ == ControlTypeBool);

	return bool_;
}

template<>
const int32_t &ControlValue::get<int32_t>() const
{
	ASSERT(type_ == ControlTypeInteger32 || type_ == ControlTypeInteger64);

	return integer32_;
}

template<>
const int64_t &ControlValue::get<int64_t>() const
{
	ASSERT(type_ == ControlTypeInteger32 || type_ == ControlTypeInteger64);

	return integer64_;
}

template<>
void ControlValue::set<bool>(const bool &value)
{
	type_ = ControlTypeBool;
	bool_ = value;
}

template<>
void ControlValue::set<int32_t>(const int32_t &value)
{
	type_ = ControlTypeInteger32;
	integer32_ = value;
}

template<>
void ControlValue::set<int64_t>(const int64_t &value)
{
	type_ = ControlTypeInteger64;
	integer64_ = value;
}
#endif /* __DOXYGEN__ */

/**
 * \brief Assemble and return a string describing the value
 * \return A string describing the ControlValue
 */
std::string ControlValue::toString() const
{
	switch (type_) {
	case ControlTypeNone:
		return "<None>";
	case ControlTypeBool:
		return bool_ ? "True" : "False";
	case ControlTypeInteger32:
		return std::to_string(integer32_);
	case ControlTypeInteger64:
		return std::to_string(integer64_);
	}

	return "<ValueType Error>";
}

/**
 * \brief Compare ControlValue instances for equality
 * \return True if the values have identical types and values, false otherwise
 */
bool ControlValue::operator==(const ControlValue &other) const
{
	if (type_ != other.type_)
		return false;

	switch (type_) {
	case ControlTypeBool:
		return bool_ == other.bool_;
	case ControlTypeInteger32:
		return integer32_ == other.integer32_;
	case ControlTypeInteger64:
		return integer64_ == other.integer64_;
	default:
		return false;
	}
}

/**
 * \fn bool ControlValue::operator!=()
 * \brief Compare ControlValue instances for non equality
 * \return False if the values have identical types and values, true otherwise
 */

/**
 * \class ControlId
 * \brief Control static metadata
 *
 * The ControlId class stores a control ID, name and data type. It provides
 * unique identification of a control, but without support for compile-time
 * type deduction that the derived template Control class supports. See the
 * Control class for more information.
 */

/**
 * \fn ControlId::ControlId(unsigned int id, const std::string &name, ControlType type)
 * \brief Construct a ControlId instance
 * \param[in] id The control numerical ID
 * \param[in] name The control name
 * \param[in] type The control data type
 */

/**
 * \fn unsigned int ControlId::id() const
 * \brief Retrieve the control numerical ID
 * \return The control numerical ID
 */

/**
 * \fn const char *ControlId::name() const
 * \brief Retrieve the control name
 * \return The control name
 */

/**
 * \fn ControlType ControlId::type() const
 * \brief Retrieve the control data type
 * \return The control data type
 */

/**
 * \fn bool operator==(const ControlId &lhs, const ControlId &rhs)
 * \brief Compare two ControlId instances for equality
 * \param[in] lhs Left-hand side ControlId
 * \param[in] rhs Right-hand side ControlId
 *
 * ControlId instances are compared based on the numerical ControlId::id()
 * only, as an object may not have two separate controls with the same
 * numerical ID.
 *
 * \return True if \a lhs and \a rhs have equal control IDs, false otherwise
 */

/**
 * \class Control
 * \brief Describe a control and its intrinsic properties
 *
 * The Control class models a control exposed by an object. Its template type
 * name T refers to the control data type, and allows methods that operate on
 * control values to be defined as template methods using the same type T for
 * the control value. See for instance how the ControlList::get() method
 * returns a value corresponding to the type of the requested control.
 *
 * While this class is the main mean to refer to a control, the control
 * identifying information are stored in the non-template base ControlId class.
 * This allows code that operates on a set of controls of different types to
 * reference those controls through a ControlId instead of a Control. For
 * instance, the list of controls supported by a camera is exposed as ControlId
 * instead of Control.
 *
 * Controls of any type can be defined through template specialisation, but
 * libcamera only supports the bool, int32_t and int64_t types natively (this
 * includes types that are equivalent to the supported types, such as int and
 * long int).
 *
 * Controls IDs shall be unique. While nothing prevents multiple instances of
 * the Control class to be created with the same ID for the same object, doing
 * so may cause undefined behaviour.
 */

/**
 * \fn Control::Control(unsigned int id, const char *name)
 * \brief Construct a Control instance
 * \param[in] id The control numerical ID
 * \param[in] name The control name
 *
 * The control data type is automatically deduced from the template type T.
 */

/**
 * \typedef Control::type
 * \brief The Control template type T
 */

#ifndef __DOXYGEN__
template<>
Control<void>::Control(unsigned int id, const char *name)
	: ControlId(id, name, ControlTypeNone)
{
}

template<>
Control<bool>::Control(unsigned int id, const char *name)
	: ControlId(id, name, ControlTypeBool)
{
}

template<>
Control<int32_t>::Control(unsigned int id, const char *name)
	: ControlId(id, name, ControlTypeInteger32)
{
}

template<>
Control<int64_t>::Control(unsigned int id, const char *name)
	: ControlId(id, name, ControlTypeInteger64)
{
}
#endif /* __DOXYGEN__ */

/**
 * \class ControlRange
 * \brief Describe the limits of valid values for a Control
 *
 * The ControlRange expresses the constraints on valid values for a control.
 * The constraints depend on the object the control applies to, and are
 * constant for the lifetime of that object. They are typically constructed by
 * pipeline handlers to describe the controls they support.
 */

/**
 * \brief Construct a ControlRange with minimum and maximum range parameters
 * \param[in] min The control minimum value
 * \param[in] max The control maximum value
 */
ControlRange::ControlRange(const ControlValue &min,
			   const ControlValue &max)
	: min_(min), max_(max)
{
}

/**
 * \fn ControlRange::min()
 * \brief Retrieve the minimum value of the control
 * \return A ControlValue with the minimum value for the control
 */

/**
 * \fn ControlRange::max()
 * \brief Retrieve the maximum value of the control
 * \return A ControlValue with the maximum value for the control
 */

/**
 * \brief Provide a string representation of the ControlRange
 */
std::string ControlRange::toString() const
{
	std::stringstream ss;

	ss << "[" << min_.toString() << ".." << max_.toString() << "]";

	return ss.str();
}

/**
 * \typedef ControlIdMap
 * \brief A map of numerical control ID to ControlId
 *
 * The map is used by ControlList instances to access controls by numerical
 * IDs. A global map of all libcamera controls is provided by
 * controls::controls.
 */

/**
 * \typedef ControlInfoMap
 * \brief A map of ControlId to ControlRange
 */

/**
 * \class ControlList
 * \brief Associate a list of ControlId with their values for an object
 *
 * The ControlList class stores values of controls exposed by an object. The
 * lists returned by the Request::controls() and Request::metadata() methods
 * refer to the camera that the request belongs to.
 *
 * Control lists are constructed with a map of all the controls supported by
 * their object, and an optional ControlValidator to further validate the
 * controls.
 */

/**
 * \brief Construct a ControlList with an optional control validator
 * \param[in] idmap The ControlId map for the control list target object
 * \param[in] validator The validator (may be null)
 *
 * For ControlList containing libcamera controls, a global map of all libcamera
 * controls is provided by controls::controls and can be used as the \a idmap
 * argument.
 */
ControlList::ControlList(const ControlIdMap &idmap, ControlValidator *validator)
	: validator_(validator), idmap_(&idmap)
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
 * \brief Check if the list contains a control with the specified \a id
 * \param[in] id The control ID
 *
 * \return True if the list contains a matching control, false otherwise
 */
bool ControlList::contains(const ControlId &id) const
{
	return controls_.find(&id) != controls_.end();
}

/**
 * \brief Check if the list contains a control with the specified \a id
 * \param[in] id The control numerical ID
 *
 * \return True if the list contains a matching control, false otherwise
 */
bool ControlList::contains(unsigned int id) const
{
	const auto iter = idmap_->find(id);
	if (iter == idmap_->end())
		return false;

	return contains(*iter->second);
}

/**
 * \fn template<typename T> const T &ControlList::get(const Control<T> &ctrl) const
 * \brief Get the value of control \a ctrl
 * \param[in] ctrl The control
 *
 * The behaviour is undefined if the control \a ctrl is not present in the
 * list. Use ControlList::contains() to test for the presence of a control in
 * the list before retrieving its value.
 *
 * The control value type shall match the type T, otherwise the behaviour is
 * undefined.
 *
 * \return The control value
 */

/**
 * \fn template<typename T> void ControlList::set(const Control<T> &ctrl, const T &value)
 * \brief Set the control \a ctrl value to \a value
 * \param[in] ctrl The control
 * \param[in] value The control value
 *
 * This method sets the value of a control in the control list. If the control
 * is already present in the list, its value is updated, otherwise it is added
 * to the list.
 *
 * The behaviour is undefined if the control \a ctrl is not supported by the
 * object that the list refers to.
 */

/**
 * \brief Get the value of control \a id
 * \param[in] id The control numerical ID
 *
 * The behaviour is undefined if the control \a id is not present in the list.
 * Use ControlList::contains() to test for the presence of a control in the
 * list before retrieving its value.
 *
 * \return The control value
 */
const ControlValue &ControlList::get(unsigned int id) const
{
	static ControlValue zero;

	const auto ctrl = idmap_->find(id);
	if (ctrl == idmap_->end()) {
		LOG(Controls, Error)
			<< "Control " << utils::hex(id)
			<< " is not supported";
		return zero;
	}

	const ControlValue *val = find(*ctrl->second);
	if (!val)
		return zero;

	return *val;
}

/**
 * \brief Set the value of control \a id to \a value
 * \param[in] id The control ID
 * \param[in] value The control value
 *
 * This method sets the value of a control in the control list. If the control
 * is already present in the list, its value is updated, otherwise it is added
 * to the list.
 *
 * The behaviour is undefined if the control \a id is not supported by the
 * object that the list refers to.
 */
void ControlList::set(unsigned int id, const ControlValue &value)
{
	const auto ctrl = idmap_->find(id);
	if (ctrl == idmap_->end()) {
		LOG(Controls, Error)
			<< "Control 0x" << utils::hex(id)
			<< " is not supported";
		return;
	}

	ControlValue *val = find(*ctrl->second);
	if (!val)
		return;

	*val = value;
}

const ControlValue *ControlList::find(const ControlId &id) const
{
	const auto iter = controls_.find(&id);
	if (iter == controls_.end()) {
		LOG(Controls, Error)
			<< "Control " << id.name() << " not found";

		return nullptr;
	}

	return &iter->second;
}

ControlValue *ControlList::find(const ControlId &id)
{
	if (validator_ && !validator_->validate(id)) {
		LOG(Controls, Error)
			<< "Control " << id.name()
			<< " is not valid for " << validator_->name();
		return nullptr;
	}

	return &controls_[&id];
}

} /* namespace libcamera */
