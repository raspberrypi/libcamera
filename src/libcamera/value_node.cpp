/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 * Copyright (C) 2026, Ideas on Board.
 *
 * Data structure to manage tree of values
 */

#include "libcamera/internal/value_node.h"

#include <charconv>
#include <errno.h>
#include <string>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

/**
 * \file value_node.h
 * \brief Data structure to manage tree of values
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(ValueNode)

namespace {

/* Empty static ValueNode as a safe result for invalid operations */
static const ValueNode empty;

} /* namespace */

/**
 * \class ValueNode
 * \brief A class representing a tree structure of values
 *
 * The ValueNode class is designed to model a tree of values. Each node in the
 * tree is represented by a ValueNode instance. Intermediate nodes store
 * children either as an ordered list (sequence) or a string-indexed dictionary
 * (mapping). Leaf nodes can be empty or store a string value.
 */

ValueNode::ValueNode()
	: type_(Type::Empty)
{
}

/**
 * \fn template<typename T> ValueNode::ValueNode(T &&value)
 * \brief Construct a ValueNode instance with a value
 * \tparam T Type of the value
 * \param[in] value The value
 */

ValueNode::~ValueNode() = default;

/**
 * \fn ValueNode::isValue()
 * \brief Return whether the ValueNode is a value
 *
 * \return True if the ValueNode is a value, false otherwise
 */

/**
 * \fn ValueNode::isList()
 * \brief Return whether the ValueNode is a list
 *
 * \return True if the ValueNode is a list, false otherwise
 */

/**
 * \fn ValueNode::isDictionary()
 * \brief Return whether the ValueNode is a dictionary
 *
 * \return True if the ValueNode is a dictionary, false otherwise
 */

/**
 * \fn ValueNode::isEmpty()
 * \brief Return whether the ValueNode is an empty
 *
 * \return True if the ValueNode is empty, false otherwise
 */

/**
 * \fn ValueNode::operator bool()
 * \brief Return whether the ValueNode is a non-empty
 *
 * \return False if the ValueNode is empty, true otherwise
 */

/**
 * \fn ValueNode::size()
 * \brief Retrieve the number of elements in a dictionary or list ValueNode
 *
 * This function retrieves the size of the ValueNode, defined as the number of
 * child elements it contains. Only ValueNode instances of Dictionary or List
 * types have a size, calling this function on other types of instances is
 * invalid and results in undefined behaviour.
 *
 * \return The size of the ValueNode
 */
std::size_t ValueNode::size() const
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
 * \fn template<typename T> ValueNode::get<T>() const
 * \brief Parse the ValueNode as a \a T value
 * \tparam T Type of the value
 *
 * This function parses the value of the ValueNode as a \a T object, and
 * returns the value. If parsing fails (usually because the ValueNode doesn't
 * store a \a T value), std::nullopt is returned.
 *
 * If the type \a T is an std::vector, the ValueNode will be parsed as a list
 * of values.
 *
 * \return The ValueNode value, or std::nullopt if parsing failed
 */

/**
 * \fn template<typename T, typename U> ValueNode::get<T>(U &&defaultValue) const
 * \brief Parse the ValueNode as a \a T value
 * \tparam T Type of the value
 * \tparam U Type of the default value
 * \param[in] defaultValue The default value when failing to parse
 *
 * This function parses the value of the ValueNode as a \a T object, and
 * returns the value. If parsing fails (usually because the ValueNode doesn't
 * store a \a T value), the \a defaultValue is returned. Type \a U must be
 * convertible to type \a T.
 *
 * Unlike the get() function, this overload does not support std::vector for the
 * type \a T.
 *
 * \return The ValueNode value, or \a defaultValue if parsing failed
 */

/**
 * \fn template<typename T> ValueNode::set<T>(T &&value)
 * \brief Set the value of a ValueNode
 * \tparam T Type of the value
 * \param[in] value The value
 *
 * This function sets the value stored in a ValueNode to \a value. The value is
 * converted to a string in an implementation-specific way that guarantees that
 * subsequent calls to get<T>() will return the same value.
 *
 * \todo Implement the conversion guarantee for floating point types
 *
 * Values can only be set on ValueNode of Type::Value type or empty ValueNode.
 * Attempting to set a value on a node of type Type::Dict or Type::List does not
 * modify the ValueNode.
 */

#ifndef __DOXYGEN__

template<>
std::optional<bool>
ValueNode::Accessor<bool>::get(const ValueNode &obj) const
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
void ValueNode::Accessor<bool>::set(ValueNode &obj, bool value)
{
	if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
		return;

	obj.type_ = Type::Value;
	obj.value_ = value ? "true" : "false";
}

template<typename T>
struct ValueNode::Accessor<T, std::enable_if_t<
	std::is_same_v<int8_t, T> ||
	std::is_same_v<uint8_t, T> ||
	std::is_same_v<int16_t, T> ||
	std::is_same_v<uint16_t, T> ||
	std::is_same_v<int32_t, T> ||
	std::is_same_v<uint32_t, T>>>
{
	std::optional<T> get(const ValueNode &obj) const
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

	void set(ValueNode &obj, T value)
	{
		if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
			return;

		obj.type_ = Type::Value;
		obj.value_ = std::to_string(value);
	}
};

template struct ValueNode::Accessor<int8_t>;
template struct ValueNode::Accessor<uint8_t>;
template struct ValueNode::Accessor<int16_t>;
template struct ValueNode::Accessor<uint16_t>;
template struct ValueNode::Accessor<int32_t>;
template struct ValueNode::Accessor<uint32_t>;

template<>
std::optional<float>
ValueNode::Accessor<float>::get(const ValueNode &obj) const
{
	return obj.get<double>();
}

template<>
void ValueNode::Accessor<float>::set(ValueNode &obj, float value)
{
	obj.set<double>(value);
}

template<>
std::optional<double>
ValueNode::Accessor<double>::get(const ValueNode &obj) const
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
void ValueNode::Accessor<double>::set(ValueNode &obj, double value)
{
	if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
		return;

	obj.type_ = Type::Value;
	obj.value_ = std::to_string(value);
}

template<>
std::optional<std::string>
ValueNode::Accessor<std::string>::get(const ValueNode &obj) const
{
	if (obj.type_ != Type::Value)
		return std::nullopt;

	return obj.value_;
}

template<>
void ValueNode::Accessor<std::string>::set(ValueNode &obj, std::string value)
{
	if (obj.type_ != Type::Empty && obj.type_ != Type::Value)
		return;

	obj.type_ = Type::Value;
	obj.value_ = std::move(value);
}

template<typename T>
struct ValueNode::Accessor<std::vector<T>, std::enable_if_t<
	std::is_same_v<bool, T> ||
	std::is_same_v<float, T> ||
	std::is_same_v<double, T> ||
	std::is_same_v<int8_t, T> ||
	std::is_same_v<uint8_t, T> ||
	std::is_same_v<int16_t, T> ||
	std::is_same_v<uint16_t, T> ||
	std::is_same_v<int32_t, T> ||
	std::is_same_v<uint32_t, T> ||
	std::is_same_v<std::string, T>>>
{
	std::optional<std::vector<T>> get(const ValueNode &obj) const
	{
		if (obj.type_ != Type::List)
			return std::nullopt;

		std::vector<T> values;
		values.reserve(obj.list_.size());

		for (const ValueNode &entry : obj.asList()) {
			const auto value = entry.get<T>();
			if (!value)
				return std::nullopt;
			values.emplace_back(*value);
		}

		return values;
	}
};

template struct ValueNode::Accessor<std::vector<bool>>;
template struct ValueNode::Accessor<std::vector<float>>;
template struct ValueNode::Accessor<std::vector<double>>;
template struct ValueNode::Accessor<std::vector<int8_t>>;
template struct ValueNode::Accessor<std::vector<uint8_t>>;
template struct ValueNode::Accessor<std::vector<int16_t>>;
template struct ValueNode::Accessor<std::vector<uint16_t>>;
template struct ValueNode::Accessor<std::vector<int32_t>>;
template struct ValueNode::Accessor<std::vector<uint32_t>>;
template struct ValueNode::Accessor<std::vector<std::string>>;
#endif /* __DOXYGEN__ */

/**
 * \fn ValueNode::asDict()
 * \copydoc ValueNode::asDict() const
 */

/**
 * \fn ValueNode::asDict() const
 * \brief Wrap a dictionary ValueNode in an adapter that exposes iterators
 *
 * The ValueNode class doesn't directly implement iterators, as the iterator
 * type depends on whether the node is a Dictionary or List. This function wraps
 * a ValueNode of Dictionary type into an adapter that exposes iterators, as
 * well as begin() and end() functions, allowing usage of range-based for loops
 * with ValueNode. As mappings are not ordered, the iteration order is not
 * specified.
 *
 * The iterator's value_type is a
 * <em>std::pair<const std::string &, const \ref ValueNode &></em>.
 *
 * If the ValueNode is not of Dictionary type, the returned adapter operates
 * as an empty container.
 *
 * \return An adapter of unspecified type compatible with range-based for loops
 */

/**
 * \fn ValueNode::asList()
 * \copydoc ValueNode::asList() const
 */

/**
 * \fn ValueNode::asList() const
 * \brief Wrap a list ValueNode in an adapter that exposes iterators
 *
 * The ValueNode class doesn't directly implement iterators, as the iterator
 * type depends on whether the node is a Dictionary or List. This function wraps
 * a ValueNode of List type into an adapter that exposes iterators, as well as
 * begin() and end() functions, allowing usage of range-based for loops with
 * ValueNode. As lists are ordered, the iteration order matches the order in
 * which child nodes have been added.
 *
 * The iterator's value_type is a <em>const ValueNode &</em>.
 *
 * If the ValueNode is not of List type, the returned adapter operates as an
 * empty container.
 *
 * \return An adapter of unspecified type compatible with range-based for loops
 */

/**
 * \brief Retrieve the element from list ValueNode by index
 * \param[in] index The element index
 *
 * This function retrieves an element of the ValueNode. Only ValueNode
 * instances of List type associate elements with an index, calling this
 * function on other types of instances or with an invalid index returns a null
 * pointer.
 *
 * \return The ValueNode corresponding to \a index
 */
ValueNode *ValueNode::at(std::size_t index)
{
	if (type_ != Type::List || index >= size())
		return nullptr;

	return list_[index].value.get();
}

/**
 * \brief Retrieve the element from list ValueNode by index
 * \param[in] index The element index
 *
 * This function retrieves an element of the ValueNode. Only ValueNode
 * instances of List type associate elements with index, calling this function
 * on other types of instances or with an invalid index results in an empty
 * node.
 *
 * \return The ValueNode as an element of the list
 */
const ValueNode &ValueNode::operator[](std::size_t index) const
{
	if (type_ != Type::List || index >= size())
		return empty;

	return *list_[index].value;
}

/**
 * \brief Check if an element of a dictionary exists
 * \param[in] key The element key
 *
 * This function checks if the ValueNode contains an element for the given
 * \a key. Only ValueNode instances of Dictionary type associate elements with
 * keys, calling this function on other types of instances is invalid and
 * results in undefined behaviour.
 *
 * \return True if an element exists, false otherwise
 */
bool ValueNode::contains(std::string_view key) const
{
	return dictionary_.find(key) != dictionary_.end();
}

/**
 * \brief Retrieve a member by key from the dictionary
 * \param[in] key The element key
 *
 * This function retrieves a member of a ValueNode by \a key. Only ValueNode
 * instances of Dictionary type associate elements with keys, calling this
 * function on other types of instances or with a nonexistent key returns a null
 * pointer.
 *
 * \return The ValueNode corresponding to the \a key member
 */
ValueNode *ValueNode::at(std::string_view key)
{
	if (type_ != Type::Dictionary)
		return nullptr;

	auto iter = dictionary_.find(key);
	if (iter == dictionary_.end())
		return nullptr;

	return iter->second;
}

/**
 * \brief Retrieve a member by key from the dictionary
 * \param[in] key The element key
 *
 * This function retrieves a member of a ValueNode by \a key. Only ValueNode
 * instances of Dictionary type associate elements with keys, calling this
 * function on other types of instances or with a nonexistent key results in an
 * empty node.
 *
 * \return The ValueNode corresponding to the \a key member
 */
const ValueNode &ValueNode::operator[](std::string_view key) const
{
	if (type_ != Type::Dictionary)
		return empty;

	auto iter = dictionary_.find(key);
	if (iter == dictionary_.end())
		return empty;

	return *iter->second;
}

/**
 * \brief Retrieve a descendant node by path
 * \param[in] path The path
 *
 * This function retrieves a descendant of a ValueNode by following a \a path.
 * The path is a list of keys that index nested dictionary nodes. If any node
 * along the path is not a Dictionary node, an empty node is returned.
 *
 * \return The ValueNode corresponding to the \a path
 */
const ValueNode &ValueNode::operator[](std::initializer_list<std::string_view> path) const
{
	const ValueNode *node = this;

	for (const auto &part : path) {
		node = &(*node)[part];
		if (!*node)
			return empty;
	}

	return *node;
}

/**
 * \brief Add a child node to a list
 * \param[in] child The child node
 *
 * Append the \a child node as the last element of this node's children list.
 * This node must be empty, in which case it is converted to the Type::List
 * type, or be a list. Otherwise, the function returns a nullptr and the
 * \a child is not modified.
 *
 * \return A pointer to the \a child node if successfully added, nullptr
 * otherwise
 */
ValueNode *ValueNode::add(std::unique_ptr<ValueNode> &&child)
{
	if (type_ == Type::Empty)
		type_ = Type::List;

	if (type_ != Type::List)
		return nullptr;

	Value &elem = list_.emplace_back(std::string{}, std::move(child));
	return elem.value.get();
}

/**
 * \brief Add a child node to a dictionary
 * \param[in] key The dictionary key
 * \param[in] child The child node
 *
 * Add the \a child node with the given \a key to this node's children. This
 * node must be empty, in which case it is converted to the Type::Dictionary
 * type, or be a dictionary. Otherwise, the function returns a nullptr and the
 * \a child is not modified.
 *
 * Keys are unique. If a child with the same \a key already exists, the function
 * returns a nullptr and the \a child is not modified.
 *
 * \return A pointer to the \a child node if successfully added, nullptr
 * otherwise
 */
ValueNode *ValueNode::add(std::string key, std::unique_ptr<ValueNode> &&child)
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

/**
 * \brief Add a child node at the given path
 * \param[in] path The path
 * \param[in] child The child node
 *
 * Add the \a child node at the given \a path starting at this node. Missing
 * nodes are created along the path. Nodes along the path must be empty (in
 * which case they are converted to the Type::Dictionary type), be a dictionary,
 * or be missing. Otherwise, the function returns a nullptr and the \a child is
 * not modified.
 *
 * Path elements are unique in the context of a parent node. If a child with the
 * same \a key already exist at the end of the path, the function returns a
 * nullptr and the \a child is not modified.
 *
 * \note Any node added along the \a path will remain even if this function
 * returns a failure.
 *
 * \return A pointer to the \a child node if successfully added, nullptr
 * otherwise
 */
ValueNode *ValueNode::add(std::initializer_list<std::string_view> path,
			  std::unique_ptr<ValueNode> &&child)
{
	if (!path.size())
		return nullptr;

	ValueNode *node = this;

	for (const auto [i, name] : utils::enumerate(path)) {
		auto iter = node->dictionary_.find(name);
		if (iter == node->dictionary_.end()) {
			std::unique_ptr<ValueNode> obj;

			if (i < path.size() - 1)
				obj = std::make_unique<ValueNode>();

			node = node->add(std::string{ name },
					 obj ? std::move(obj) : std::move(child));
			if (!node) {
				Span<const std::string_view> pathName{ std::data(path), i + 1 };
				LOG(ValueNode, Error)
					<< "Failed to populate '"
					<< utils::join(pathName, "/") << "'";
				return nullptr;
			}
		} else {
			node = iter->second;
		}
	}

	return node;
}

/**
 * \brief Erase a child node in a dictionary
 * \param[in] key The dictionary key
 *
 * Erase the child node referenced by \a key in a dictionary node. If the \a
 * key does not exist, or if this node is not a dictionary, the function
 * returns without performing any operation.
 */
void ValueNode::erase(std::string_view key)
{
	auto node = dictionary_.find(key);
	if (node == dictionary_.end())
		return;

	/* \todo Not an ideal algorithm */
	for (auto iter = list_.begin(); iter != list_.end(); ++iter) {
		if (iter->value.get() != node->second)
			continue;

		list_.erase(iter);
		break;
	}

	dictionary_.erase(node);
}

/**
 * \brief Erase the child node at the given path
 * \param[in] path The path
 *
 * Erase the child node at the given \a path. If no child node exists for the
 * path, the function returns without performing any operation.
 */
void ValueNode::erase(std::initializer_list<std::string_view> path)
{
	if (!path.size())
		return;

	ValueNode *node = this;

	for (const auto [i, name] : utils::enumerate(path)) {
		if (i == path.size() - 1) {
			node->erase(name);
			return;
		}

		auto iter = node->dictionary_.find(name);
		if (iter == node->dictionary_.end())
			return;

		node = iter->second;
	}
}

} /* namespace libcamera */
