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
#include <string.h>

#include "libcamera/internal/control_validator.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/utils.h"

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

namespace {

static constexpr size_t ControlValueSize[] = {
	[ControlTypeNone]		= 0,
	[ControlTypeBool]		= sizeof(bool),
	[ControlTypeByte]		= sizeof(uint8_t),
	[ControlTypeInteger32]		= sizeof(int32_t),
	[ControlTypeInteger64]		= sizeof(int64_t),
	[ControlTypeFloat]		= sizeof(float),
	[ControlTypeString]		= sizeof(char),
	[ControlTypeRectangle]		= sizeof(Rectangle),
	[ControlTypeSize]		= sizeof(Size),
};

} /* namespace */

/**
 * \enum ControlType
 * \brief Define the data type of a Control
 * \var ControlTypeNone
 * Invalid type, for empty values
 * \var ControlTypeBool
 * The control stores a boolean value
 * \var ControlTypeByte
 * The control stores a byte value as an unsigned 8-bit integer
 * \var ControlTypeInteger32
 * The control stores a 32-bit integer value
 * \var ControlTypeInteger64
 * The control stores a 64-bit integer value
 * \var ControlTypeFloat
 * The control stores a 32-bit floating point value
 * \var ControlTypeString
 * The control stores a string value as an array of char
 */

/**
 * \class ControlValue
 * \brief Abstract type representing the value of a control
 */

/** \todo Revisit the ControlValue layout when stabilizing the ABI */
static_assert(sizeof(ControlValue) == 16, "Invalid size of ControlValue class");

/**
 * \brief Construct an empty ControlValue.
 */
ControlValue::ControlValue()
	: type_(ControlTypeNone), isArray_(false), numElements_(0)
{
}

/**
 * \fn template<typename T> T ControlValue::ControlValue(const T &value)
 * \brief Construct a ControlValue of type T
 * \param[in] value Initial value
 *
 * This function constructs a new instance of ControlValue and stores the \a
 * value inside it. If the type \a T is equivalent to Span<R>, the instance
 * stores an array of values of type \a R. Otherwise the instance stores a
 * single value of type \a T. The numElements() and type() are updated to
 * reflect the stored value.
 */

void ControlValue::release()
{
	std::size_t size = numElements_ * ControlValueSize[type_];

	if (size > sizeof(value_)) {
		delete[] reinterpret_cast<uint8_t *>(storage_);
		storage_ = nullptr;
	}
}

ControlValue::~ControlValue()
{
	release();
}

/**
 * \brief Construct a ControlValue with the content of \a other
 * \param[in] other The ControlValue to copy content from
 */
ControlValue::ControlValue(const ControlValue &other)
	: type_(ControlTypeNone), numElements_(0)
{
	*this = other;
}

/**
 * \brief Replace the content of the ControlValue with a copy of the content
 * of \a other
 * \param[in] other The ControlValue to copy content from
 * \return The ControlValue with its content replaced with the one of \a other
 */
ControlValue &ControlValue::operator=(const ControlValue &other)
{
	set(other.type_, other.isArray_, other.data().data(),
	    other.numElements_, ControlValueSize[other.type_]);
	return *this;
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
 * \fn ControlValue::isArray()
 * \brief Determine if the value stores an array
 * \return True if the value stores an array, false otherwise
 */

/**
 * \fn ControlValue::numElements()
 * \brief Retrieve the number of elements stored in the ControlValue
 *
 * For instances storing an array, this function returns the number of elements
 * in the array. For instances storing a string, it returns the length of the
 * string, not counting the terminating '\0'. Otherwise, it returns 1.
 *
 * \return The number of elements stored in the ControlValue
 */

/**
 * \brief Retrieve the raw data of a control value
 * \return The raw data of the control value as a span of uint8_t
 */
Span<const uint8_t> ControlValue::data() const
{
	std::size_t size = numElements_ * ControlValueSize[type_];
	const uint8_t *data = size > sizeof(value_)
			    ? reinterpret_cast<const uint8_t *>(storage_)
			    : reinterpret_cast<const uint8_t *>(&value_);
	return { data, size };
}

/**
 * \copydoc ControlValue::data() const
 */
Span<uint8_t> ControlValue::data()
{
	Span<const uint8_t> data = const_cast<const ControlValue *>(this)->data();
	return { const_cast<uint8_t *>(data.data()), data.size() };
}

/**
 * \brief Assemble and return a string describing the value
 * \return A string describing the ControlValue
 */
std::string ControlValue::toString() const
{
	if (type_ == ControlTypeNone)
		return "<ValueType Error>";

	const uint8_t *data = ControlValue::data().data();

	if (type_ == ControlTypeString)
		return std::string(reinterpret_cast<const char *>(data),
				   numElements_);

	std::string str(isArray_ ? "[ " : "");

	for (unsigned int i = 0; i < numElements_; ++i) {
		switch (type_) {
		case ControlTypeBool: {
			const bool *value = reinterpret_cast<const bool *>(data);
			str += *value ? "true" : "false";
			break;
		}
		case ControlTypeByte: {
			const uint8_t *value = reinterpret_cast<const uint8_t *>(data);
			str += std::to_string(*value);
			break;
		}
		case ControlTypeInteger32: {
			const int32_t *value = reinterpret_cast<const int32_t *>(data);
			str += std::to_string(*value);
			break;
		}
		case ControlTypeInteger64: {
			const int64_t *value = reinterpret_cast<const int64_t *>(data);
			str += std::to_string(*value);
			break;
		}
		case ControlTypeFloat: {
			const float *value = reinterpret_cast<const float *>(data);
			str += std::to_string(*value);
			break;
		}
		case ControlTypeRectangle: {
			const Rectangle *value = reinterpret_cast<const Rectangle *>(data);
			str += value->toString();
			break;
		}
		case ControlTypeSize: {
			const Size *value = reinterpret_cast<const Size *>(data);
			str += value->toString();
			break;
		}
		case ControlTypeNone:
		case ControlTypeString:
			break;
		}

		if (i + 1 != numElements_)
			str += ", ";

		data += ControlValueSize[type_];
	}

	if (isArray_)
		str += " ]";

	return str;
}

/**
 * \brief Compare ControlValue instances for equality
 * \return True if the values have identical types and values, false otherwise
 */
bool ControlValue::operator==(const ControlValue &other) const
{
	if (type_ != other.type_)
		return false;

	if (numElements_ != other.numElements())
		return false;

	if (isArray_ != other.isArray_)
		return false;

	return memcmp(data().data(), other.data().data(), data().size()) == 0;
}

/**
 * \fn bool ControlValue::operator!=()
 * \brief Compare ControlValue instances for non equality
 * \return False if the values have identical types and values, true otherwise
 */

/**
 * \fn template<typename T> T ControlValue::get() const
 * \brief Get the control value
 *
 * This function returns the contained value as an instance of \a T. If the
 * ControlValue instance stores a single value, the type \a T shall match the
 * stored value type(). If the instance stores an array of values, the type
 * \a T should be equal to Span<const R>, and the type \a R shall match the
 * stored value type(). The behaviour is undefined otherwise.
 *
 * Note that a ControlValue instance that stores a non-array value is not
 * equivalent to an instance that stores an array value containing a single
 * element. The latter shall be accessed through a Span<const R> type, while
 * the former shall be accessed through a type \a T corresponding to type().
 *
 * \return The control value
 */

/**
 * \fn template<typename T> void ControlValue::set(const T &value)
 * \brief Set the control value to \a value
 * \param[in] value The control value
 *
 * This function stores the \a value in the instance. If the type \a T is
 * equivalent to Span<R>, the instance stores an array of values of type \a R.
 * Otherwise the instance stores a single value of type \a T. The numElements()
 * and type() are updated to reflect the stored value.
 *
 * The entire content of \a value is copied to the instance, no reference to \a
 * value or to the data it references is retained. This may be an expensive
 * operation for Span<> values that refer to large arrays.
 */

void ControlValue::set(ControlType type, bool isArray, const void *data,
		       std::size_t numElements, std::size_t elementSize)
{
	ASSERT(elementSize == ControlValueSize[type]);

	reserve(type, isArray, numElements);

	Span<uint8_t> storage = ControlValue::data();
	memcpy(storage.data(), data, storage.size());
}

/**
 * \brief Set the control type and reserve memory
 * \param[in] type The control type
 * \param[in] isArray True to make the value an array
 * \param[in] numElements The number of elements
 *
 * This function sets the type of the control value to \a type, and reserves
 * memory to store the control value. If \a isArray is true, the instance
 * becomes an array control and storage for \a numElements is reserved.
 * Otherwise the instance becomes a simple control, numElements is ignored, and
 * storage for the single element is reserved.
 */
void ControlValue::reserve(ControlType type, bool isArray, std::size_t numElements)
{
	if (!isArray)
		numElements = 1;

	std::size_t oldSize = numElements_ * ControlValueSize[type_];
	std::size_t newSize = numElements * ControlValueSize[type];

	if (oldSize != newSize)
		release();

	type_ = type;
	isArray_ = isArray;
	numElements_ = numElements;

	if (oldSize == newSize)
		return;

	if (newSize > sizeof(value_))
		storage_ = reinterpret_cast<void *>(new uint8_t[newSize]);
}

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
 * \fn bool operator==(unsigned int lhs, const ControlId &rhs)
 * \brief Compare a ControlId with a control numerical ID
 * \param[in] lhs Left-hand side numerical ID
 * \param[in] rhs Right-hand side ControlId
 *
 * \return True if \a lhs is equal to \a rhs.id(), false otherwise
 */

/**
 * \fn bool operator==(const ControlId &lhs, unsigned int rhs)
 * \brief Compare a ControlId with a control numerical ID
 * \param[in] lhs Left-hand side ControlId
 * \param[in] rhs Right-hand side numerical ID
 *
 * \return True if \a lhs.id() is equal to \a rhs, false otherwise
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
 * libcamera only supports the bool, uint8_t, int32_t, int64_t and float types
 * natively (this includes types that are equivalent to the supported types,
 * such as int and long int).
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

/**
 * \class ControlInfo
 * \brief Describe the limits of valid values for a Control
 *
 * The ControlInfo expresses the constraints on valid values for a control.
 * The constraints depend on the object the control applies to, and are
 * constant for the lifetime of that object. They are typically constructed by
 * pipeline handlers to describe the controls they support.
 */

/**
 * \brief Construct a ControlInfo with minimum and maximum range parameters
 * \param[in] min The control minimum value
 * \param[in] max The control maximum value
 * \param[in] def The control default value
 */
ControlInfo::ControlInfo(const ControlValue &min,
			 const ControlValue &max,
			 const ControlValue &def)
	: min_(min), max_(max), def_(def)
{
}

/**
 * \fn ControlInfo::min()
 * \brief Retrieve the minimum value of the control
 *
 * For string controls, this is the minimum length of the string, not counting
 * the terminating '\0'. For all other control types, this is the minimum value
 * of each element.
 *
 * \return A ControlValue with the minimum value for the control
 */

/**
 * \fn ControlInfo::max()
 * \brief Retrieve the maximum value of the control
 *
 * For string controls, this is the maximum length of the string, not counting
 * the terminating '\0'. For all other control types, this is the maximum value
 * of each element.
 *
 * \return A ControlValue with the maximum value for the control
 */

/**
 * \fn ControlInfo::def()
 * \brief Retrieve the default value of the control
 * \return A ControlValue with the default value for the control
 */

/**
 * \brief Provide a string representation of the ControlInfo
 */
std::string ControlInfo::toString() const
{
	std::stringstream ss;

	ss << "[" << min_.toString() << ".." << max_.toString() << "]";

	return ss.str();
}

/**
 * \fn bool ControlInfo::operator==()
 * \brief Compare ControlInfo instances for equality
 * \return True if the constraints have identical min and max, false otherwise
 */

/**
 * \fn bool ControlInfo::operator!=()
 * \brief Compare ControlInfo instances for non equality
 * \return True if the constraints have different min and max, false otherwise
 */

/**
 * \typedef ControlIdMap
 * \brief A map of numerical control ID to ControlId
 *
 * The map is used by ControlList instances to access controls by numerical
 * IDs. A global map of all libcamera controls is provided by
 * controls::controls.
 */

/**
 * \class ControlInfoMap
 * \brief A map of ControlId to ControlInfo
 *
 * The ControlInfoMap class describes controls supported by an object as an
 * unsorted map of ControlId pointers to ControlInfo instances. Unlike the
 * standard std::unsorted_map<> class, it is designed the be immutable once
 * constructed, and thus only exposes the read accessors of the
 * std::unsorted_map<> base class.
 *
 * In addition to the features of the standard unsorted map, this class also
 * provides access to the mapped elements using numerical ID keys. It maintains
 * an internal map of numerical ID to ControlId for this purpose, and exposes it
 * through the idmap() method to help construction of ControlList instances.
 */

/**
 * \typedef ControlInfoMap::Map
 * \brief The base std::unsorted_map<> container
 */

/**
 * \fn ControlInfoMap::ControlInfoMap(const ControlInfoMap &other)
 * \brief Copy constructor, construct a ControlInfoMap from a copy of \a other
 * \param[in] other The other ControlInfoMap
 */

/**
 * \brief Construct a ControlInfoMap from an initializer list
 * \param[in] init The initializer list
 */
ControlInfoMap::ControlInfoMap(std::initializer_list<Map::value_type> init)
	: Map(init)
{
	generateIdmap();
}

/**
 * \brief Construct a ControlInfoMap from a plain map
 * \param[in] info The control info plain map
 *
 * Construct a new ControlInfoMap and populate its contents with those of
 * \a info using move semantics. Upon return the \a info map will be empty.
 */
ControlInfoMap::ControlInfoMap(Map &&info)
	: Map(std::move(info))
{
	generateIdmap();
}

/**
 * \fn ControlInfoMap &ControlInfoMap::operator=(const ControlInfoMap &other)
 * \brief Copy assignment operator, replace the contents with a copy of \a other
 * \param[in] other The other ControlInfoMap
 * \return A reference to the ControlInfoMap
 */

/**
 * \brief Replace the contents with those from the initializer list
 * \param[in] init The initializer list
 * \return A reference to the ControlInfoMap
 */
ControlInfoMap &ControlInfoMap::operator=(std::initializer_list<Map::value_type> init)
{
	Map::operator=(init);
	generateIdmap();
	return *this;
}

/**
 * \brief Move assignment operator from a plain map
 * \param[in] info The control info plain map
 *
 * Populate the map by replacing its contents with those of \a info using move
 * semantics. Upon return the \a info map will be empty.
 *
 * \return A reference to the populated ControlInfoMap
 */
ControlInfoMap &ControlInfoMap::operator=(Map &&info)
{
	Map::operator=(std::move(info));
	generateIdmap();
	return *this;
}

/**
 * \brief Access specified element by numerical ID
 * \param[in] id The numerical ID
 * \return A reference to the element whose ID is equal to \a id
 */
ControlInfoMap::mapped_type &ControlInfoMap::at(unsigned int id)
{
	return at(idmap_.at(id));
}

/**
 * \brief Access specified element by numerical ID
 * \param[in] id The numerical ID
 * \return A const reference to the element whose ID is equal to \a id
 */
const ControlInfoMap::mapped_type &ControlInfoMap::at(unsigned int id) const
{
	return at(idmap_.at(id));
}

/**
 * \brief Count the number of elements matching a numerical ID
 * \param[in] id The numerical ID
 * \return The number of elements matching the numerical \a id
 */
ControlInfoMap::size_type ControlInfoMap::count(unsigned int id) const
{
	/*
	 * The ControlInfoMap and its idmap have a 1:1 mapping between their
	 * entries, we can thus just count the matching entries in idmap to
	 * avoid an additional lookup.
	 */
	return idmap_.count(id);
}

/**
 * \brief Find the element matching a numerical ID
 * \param[in] id The numerical ID
 * \return An iterator pointing to the element matching the numerical \a id, or
 * end() if no such element exists
 */
ControlInfoMap::iterator ControlInfoMap::find(unsigned int id)
{
	auto iter = idmap_.find(id);
	if (iter == idmap_.end())
		return end();

	return find(iter->second);
}

/**
 * \brief Find the element matching a numerical ID
 * \param[in] id The numerical ID
 * \return A const iterator pointing to the element matching the numerical
 * \a id, or end() if no such element exists
 */
ControlInfoMap::const_iterator ControlInfoMap::find(unsigned int id) const
{
	auto iter = idmap_.find(id);
	if (iter == idmap_.end())
		return end();

	return find(iter->second);
}

/**
 * \fn const ControlIdMap &ControlInfoMap::idmap() const
 * \brief Retrieve the ControlId map
 *
 * Constructing ControlList instances for V4L2 controls requires a ControlIdMap
 * for the V4L2 device that the control list targets. This helper method
 * returns a suitable idmap for that purpose.
 *
 * \return The ControlId map
 */

void ControlInfoMap::generateIdmap()
{
	idmap_.clear();

	for (const auto &ctrl : *this) {
		/*
		 * For string controls, min and max define the valid
		 * range for the string size, not for the individual
		 * values.
		 */
		ControlType rangeType = ctrl.first->type() == ControlTypeString
				      ? ControlTypeInteger32 : ctrl.first->type();
		const ControlInfo &info = ctrl.second;

		if (info.min().type() != rangeType) {
			LOG(Controls, Error)
				<< "Control " << utils::hex(ctrl.first->id())
				<< " type and info type mismatch";
			idmap_.clear();
			clear();
			return;
		}

		idmap_[ctrl.first->id()] = ctrl.first;
	}
}

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
 * \brief Construct a ControlList not associated with any object
 *
 * This constructor is meant to support ControlList serialization and shall not
 * be used directly by application.
 */
ControlList::ControlList()
	: validator_(nullptr), idmap_(nullptr), infoMap_(nullptr)
{
}

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
	: validator_(validator), idmap_(&idmap), infoMap_(nullptr)
{
}

/**
 * \brief Construct a ControlList with the idmap of a control info map
 * \param[in] infoMap The ControlInfoMap for the control list target object
 * \param[in] validator The validator (may be null)
 */
ControlList::ControlList(const ControlInfoMap &infoMap, ControlValidator *validator)
	: validator_(validator), idmap_(&infoMap.idmap()), infoMap_(&infoMap)
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
	return controls_.find(id.id()) != controls_.end();
}

/**
 * \brief Check if the list contains a control with the specified \a id
 * \param[in] id The control numerical ID
 *
 * \return True if the list contains a matching control, false otherwise
 */
bool ControlList::contains(unsigned int id) const
{
	return controls_.find(id) != controls_.end();
}

/**
 * \fn template<typename T> T ControlList::get(const Control<T> &ctrl) const
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
 * \fn template<typename T, typename V> void ControlList::set(const Control<T> &ctrl, const V &value)
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
 * \fn template<typename T, typename V> \
 * void ControlList::set(const Control<T> &ctrl, const std::initializer_list<V> &value)
 * \copydoc ControlList::set(const Control<T> &ctrl, const V &value)
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
	static const ControlValue zero;

	const ControlValue *val = find(id);
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
	ControlValue *val = find(id);
	if (!val)
		return;

	*val = value;
}

/**
 * \fn ControlList::infoMap()
 * \brief Retrieve the ControlInfoMap used to construct the ControlList
 *
 * \return The ControlInfoMap used to construct the ControlList. ControlList
 * instances constructed with ControlList() or
 * ControlList(const ControlIdMap &idmap, ControlValidator *validator) have no
 * associated ControlInfoMap, nullptr is returned in that case.
 */

const ControlValue *ControlList::find(unsigned int id) const
{
	const auto iter = controls_.find(id);
	if (iter == controls_.end()) {
		LOG(Controls, Error)
			<< "Control " << utils::hex(id) << " not found";

		return nullptr;
	}

	return &iter->second;
}

ControlValue *ControlList::find(unsigned int id)
{
	if (validator_ && !validator_->validate(id)) {
		LOG(Controls, Error)
			<< "Control " << utils::hex(id)
			<< " is not valid for " << validator_->name();
		return nullptr;
	}

	return &controls_[id];
}

} /* namespace libcamera */
