/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 * Copyright (C) 2025, Ideas on Board.
 *
 * libcamera YAML object
 */

#include "libcamera/internal/yaml_object.h"

#include <charconv>
#include <errno.h>
#include <string>
#include <vector>

#include <libcamera/base/utils.h>

/**
 * \file yaml_object.h
 * \brief YAML objects
 */

namespace libcamera {

namespace {

/* Empty static YamlObject as a safe result for invalid operations */
static const YamlObject empty;

} /* namespace */

/**
 * \class YamlObject
 * \brief A class representing the tree structure of the YAML content
 *
 * The YamlObject class represents the tree structure of YAML content. A
 * YamlObject can be empty, a dictionary or list of YamlObjects, or a value if a
 * tree leaf.
 */

YamlObject::YamlObject()
	: type_(Type::Empty)
{
}

YamlObject::~YamlObject() = default;

/**
 * \fn YamlObject::isValue()
 * \brief Return whether the YamlObject is a value
 *
 * \return True if the YamlObject is a value, false otherwise
 */

/**
 * \fn YamlObject::isList()
 * \brief Return whether the YamlObject is a list
 *
 * \return True if the YamlObject is a list, false otherwise
 */

/**
 * \fn YamlObject::isDictionary()
 * \brief Return whether the YamlObject is a dictionary
 *
 * \return True if the YamlObject is a dictionary, false otherwise
 */

/**
 * \fn YamlObject::isEmpty()
 * \brief Return whether the YamlObject is an empty
 *
 * \return True if the YamlObject is empty, false otherwise
 */

/**
 * \fn YamlObject::operator bool()
 * \brief Return whether the YamlObject is a non-empty
 *
 * \return False if the YamlObject is empty, true otherwise
 */

/**
 * \brief Retrieve the number of elements in a dictionary or list YamlObject
 *
 * This function retrieves the size of the YamlObject, defined as the number of
 * child elements it contains. Only YamlObject instances of Dictionary or List
 * types have a size, calling this function on other types of instances is
 * invalid and results in undefined behaviour.
 *
 * \return The size of the YamlObject
 */
std::size_t YamlObject::size() const
{
	switch (type_) {
	case Type::Dictionary:
	case Type::List:
		return list_.size();
	default:
		return 0;
	}
}

/**
 * \fn template<typename T> YamlObject::get<T>() const
 * \brief Parse the YamlObject as a \a T value
 * \tparam T Type of the value
 *
 * This function parses the value of the YamlObject as a \a T object, and
 * returns the value. If parsing fails (usually because the YamlObject doesn't
 * store a \a T value), std::nullopt is returned.
 *
 * If the type \a T is an std::vector, the YamlObject will be parsed as a list
 * of values.
 *
 * \return The YamlObject value, or std::nullopt if parsing failed
 */

/**
 * \fn template<typename T, typename U> YamlObject::get<T>(U &&defaultValue) const
 * \brief Parse the YamlObject as a \a T value
 * \tparam T Type of the value
 * \tparam U Type of the default value
 * \param[in] defaultValue The default value when failing to parse
 *
 * This function parses the value of the YamlObject as a \a T object, and
 * returns the value. If parsing fails (usually because the YamlObject doesn't
 * store a \a T value), the \a defaultValue is returned. Type \a U must be
 * convertible to type \a T.
 *
 * Unlike the get() function, this overload does not support std::vector for the
 * type \a T.
 *
 * \return The YamlObject value, or \a defaultValue if parsing failed
 */

/**
 * \fn template<typename T> YamlObject::set<T>(T &&value)
 * \brief Set the value of a YamlObject
 * \tparam T Type of the value
 * \param[in] value The value
 *
 * This function sets the value stored in a YamlObject to \a value. The value is
 * converted to a string in an implementation-specific way that guarantees that
 * subsequent calls to get<T>() will return the same value.
 *
 * \todo Implement the conversion guarantee for floating point types
 *
 * Values can only be set on YamlObject of Type::Value type or empty YamlObject.
 * Attempting to set a value on an object of type Type::Dict or Type::List does
 * not modify the YamlObject.
 */

#ifndef __DOXYGEN__

template<>
std::optional<bool>
YamlObject::Accessor<bool>::get(const YamlObject &obj) const
{
	if (obj.type_ != Type::Value)
		return std::nullopt;

	if (obj.value_ == "true")
		return true;
	else if (obj.value_ == "false")
		return false;

	return std::nullopt;
}

template<>
void YamlObject::Accessor<bool>::set(YamlObject &obj, bool value)
{
	if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
		return;

	obj.type_ = Type::Value;
	obj.value_ = value ? "true" : "false";
}

template<typename T>
struct YamlObject::Accessor<T, std::enable_if_t<
	std::is_same_v<int8_t, T> ||
	std::is_same_v<uint8_t, T> ||
	std::is_same_v<int16_t, T> ||
	std::is_same_v<uint16_t, T> ||
	std::is_same_v<int32_t, T> ||
	std::is_same_v<uint32_t, T>>>
{
	std::optional<T> get(const YamlObject &obj) const
	{
		if (obj.type_ != Type::Value)
			return std::nullopt;

		const std::string &str = obj.value_;
		T value = {};

		auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(),
						 value);
		if (ptr != str.data() + str.size() || ec != std::errc())
			return std::nullopt;

		return value;
	}

	void set(YamlObject &obj, T value)
	{
		if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
			return;

		obj.type_ = Type::Value;
		obj.value_ = std::to_string(value);
	}
};

template struct YamlObject::Accessor<int8_t>;
template struct YamlObject::Accessor<uint8_t>;
template struct YamlObject::Accessor<int16_t>;
template struct YamlObject::Accessor<uint16_t>;
template struct YamlObject::Accessor<int32_t>;
template struct YamlObject::Accessor<uint32_t>;

template<>
std::optional<float>
YamlObject::Accessor<float>::get(const YamlObject &obj) const
{
	return obj.get<double>();
}

template<>
void YamlObject::Accessor<float>::set(YamlObject &obj, float value)
{
	obj.set<double>(value);
}

template<>
std::optional<double>
YamlObject::Accessor<double>::get(const YamlObject &obj) const
{
	if (obj.type_ != Type::Value)
		return std::nullopt;

	if (obj.value_.empty())
		return std::nullopt;

	char *end;

	errno = 0;
	double value = utils::strtod(obj.value_.c_str(), &end);

	if ('\0' != *end || errno == ERANGE)
		return std::nullopt;

	return value;
}

template<>
void YamlObject::Accessor<double>::set(YamlObject &obj, double value)
{
	if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
		return;

	obj.type_ = Type::Value;
	obj.value_ = std::to_string(value);
}

template<>
std::optional<std::string>
YamlObject::Accessor<std::string>::get(const YamlObject &obj) const
{
	if (obj.type_ != Type::Value)
		return std::nullopt;

	return obj.value_;
}

template<>
void YamlObject::Accessor<std::string>::set(YamlObject &obj, std::string value)
{
	if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
		return;

	obj.type_ = Type::Value;
	obj.value_ = std::move(value);
}

template<typename T>
struct YamlObject::Accessor<std::vector<T>> {
	std::optional<std::vector<T>> get(const YamlObject &obj) const
	{
		if (obj.type_ != Type::List)
			return std::nullopt;

		std::vector<T> values;
		values.reserve(obj.list_.size());

		for (const YamlObject &entry : obj.asList()) {
			auto value = entry.get<T>();
			if (!value)
				return std::nullopt;
			values.emplace_back(std::move(*value));
		}

		return values;
	}
};

template struct YamlObject::Accessor<std::vector<bool>>;
template struct YamlObject::Accessor<std::vector<float>>;
template struct YamlObject::Accessor<std::vector<double>>;
template struct YamlObject::Accessor<std::vector<int8_t>>;
template struct YamlObject::Accessor<std::vector<uint8_t>>;
template struct YamlObject::Accessor<std::vector<int16_t>>;
template struct YamlObject::Accessor<std::vector<uint16_t>>;
template struct YamlObject::Accessor<std::vector<int32_t>>;
template struct YamlObject::Accessor<std::vector<uint32_t>>;
template struct YamlObject::Accessor<std::vector<std::string>>;
#endif /* __DOXYGEN__ */

/**
 * \fn YamlObject::asDict() const
 * \brief Wrap a dictionary YamlObject in an adapter that exposes iterators
 *
 * The YamlObject class doesn't directly implement iterators, as the iterator
 * type depends on whether the object is a Dictionary or List. This function
 * wraps a YamlObject of Dictionary type into an adapter that exposes
 * iterators, as well as begin() and end() functions, allowing usage of
 * range-based for loops with YamlObject. As YAML mappings are not ordered, the
 * iteration order is not specified.
 *
 * The iterator's value_type is a
 * <em>std::pair<const std::string &, const \ref YamlObject &></em>.
 *
 * If the YamlObject is not of Dictionary type, the returned adapter operates
 * as an empty container.
 *
 * \return An adapter of unspecified type compatible with range-based for loops
 */

/**
 * \fn YamlObject::asList() const
 * \brief Wrap a list YamlObject in an adapter that exposes iterators
 *
 * The YamlObject class doesn't directly implement iterators, as the iterator
 * type depends on whether the object is a Dictionary or List. This function
 * wraps a YamlObject of List type into an adapter that exposes iterators, as
 * well as begin() and end() functions, allowing usage of range-based for loops
 * with YamlObject. As YAML lists are ordered, the iteration order is identical
 * to the list order in the YAML data.
 *
 * The iterator's value_type is a <em>const YamlObject &</em>.
 *
 * If the YamlObject is not of List type, the returned adapter operates as an
 * empty container.
 *
 * \return An adapter of unspecified type compatible with range-based for loops
 */

/**
 * \brief Retrieve the element from list YamlObject by index
 * \param[in] index The element index
 *
 * This function retrieves an element of the YamlObject. Only YamlObject
 * instances of List type associate elements with index, calling this function
 * on other types of instances or with an invalid index results in an empty
 * object.
 *
 * \return The YamlObject as an element of the list
 */
const YamlObject &YamlObject::operator[](std::size_t index) const
{
	if (type_ != Type::List || index >= size())
		return empty;

	return *list_[index].value;
}

/**
 * \brief Check if an element of a dictionary exists
 * \param[in] key The element key
 *
 * This function checks if the YamlObject contains an element for the given
 * \a key. Only YamlObject instances of Dictionary type associate elements with
 * keys, calling this function on other types of instances is invalid and
 * results in undefined behaviour.
 *
 * \return True if an element exists, false otherwise
 */
bool YamlObject::contains(std::string_view key) const
{
	return dictionary_.find(key) != dictionary_.end();
}

/**
 * \brief Retrieve a member by key from the dictionary
 * \param[in] key The element key
 *
 * This function retrieves a member of a YamlObject by \a key. Only YamlObject
 * instances of Dictionary type associate elements with keys, calling this
 * function on other types of instances or with a nonexistent key results in an
 * empty object.
 *
 * \return The YamlObject corresponding to the \a key member
 */
const YamlObject &YamlObject::operator[](std::string_view key) const
{
	if (type_ != Type::Dictionary)
		return empty;

	auto iter = dictionary_.find(key);
	if (iter == dictionary_.end())
		return empty;

	return *iter->second;
}

/**
 * \brief Add a child object to a list
 * \param[in] child The child object
 *
 * Append the \a child object as the last element of this object's children
 * list. This object must be empty, in which case it is converted to the
 * Type::List type, or be a list. Otherwise, the function returns a nullptr and
 * the \a child is not modified.
 *
 * \return A pointer to the child object if successfully added, nullptr
 * otherwise
 */
YamlObject *YamlObject::add(std::unique_ptr<YamlObject> &&child)
{
	if (type_ == Type::Empty)
		type_ = Type::List;

	if (type_ != Type::List)
		return nullptr;

	Value &elem = list_.emplace_back(std::string{}, std::move(child));
	return elem.value.get();
}

/**
 * \brief Add a child object to a dictionary
 * \param[in] key The dictionary key
 * \param[in] child The child object
 *
 * Add the \a child object with the given \a key to this object's children. This
 * object must be empty, in which case it is converted to the Type::Dictionary
 * type, or be a dictionary. Otherwise, the function returns a nullptr and the
 * \a child is not modified.
 *
 * Keys are unique. If a child with the same \a key already exists, the function
 * returns a nullptr and the \a child is not modified.
 *
 * \return A pointer to the child object if successfully added, nullptr
 * otherwise
 */
YamlObject *YamlObject::add(std::string key, std::unique_ptr<YamlObject> &&child)
{
	if (type_ == Type::Empty)
		type_ = Type::Dictionary;

	if (type_ != Type::Dictionary)
		return nullptr;

	auto [it, inserted] = dictionary_.try_emplace(std::move(key), child.get());
	if (!inserted)
		return nullptr;

	return list_.emplace_back(it->first, std::move(child)).value.get();
}

} /* namespace libcamera */
