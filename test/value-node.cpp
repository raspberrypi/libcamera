/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026, Ideas on Board
 *
 * ValueNode tests
 */

#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <variant>

#include <libcamera/base/utils.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/value_node.h"

#include "test.h"

using namespace libcamera;
using namespace std;

class ValueNodeTest : public Test
{
protected:
	enum class NodeType {
		Empty,
		Value,
		List,
		Dictionary,
	};

	enum class ValueType {
		Int8,
		UInt8,
		Int16,
		UInt16,
		Int32,
		UInt32,
		Float,
		Double,
		String,
		Size,
	};

	int testNodeValueType(const ValueNode &node, std::string_view name, ValueType type)
	{
		bool isInteger8 = type == ValueType::Int8 || type == ValueType::UInt8;
		bool isInteger16 = type == ValueType::Int16 || type == ValueType::UInt16;
		bool isInteger32 = type == ValueType::Int32 || type == ValueType::UInt32;
		bool isIntegerUpTo16 = isInteger8 || isInteger16;
		bool isIntegerUpTo32 = isIntegerUpTo16 || isInteger32;
		bool isSigned = type == ValueType::Int8 || type == ValueType::Int16 ||
				type == ValueType::Int32;

		if (!isInteger8 && node.get<int8_t>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "int8_t" << std::endl;
			return TestFail;
		}

		if ((!isInteger8 || isSigned) && node.get<uint8_t>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "uint8_t" << std::endl;
			return TestFail;
		}

		if (!isIntegerUpTo16 && node.get<int16_t>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "int16_t" << std::endl;
			return TestFail;
		}

		if ((!isIntegerUpTo16 || isSigned) && node.get<uint16_t>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "uint16_t" << std::endl;
			return TestFail;
		}

		if (!isIntegerUpTo32 && node.get<int32_t>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "int32_t" << std::endl;
			return TestFail;
		}

		if ((!isIntegerUpTo32 || isSigned) && node.get<uint32_t>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "uint32_t" << std::endl;
			return TestFail;
		}

		if (!isIntegerUpTo32 && type != ValueType::Float &&
		    type != ValueType::Double && node.get<double>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "double" << std::endl;
			return TestFail;
		}

		if (type != ValueType::Size && node.get<Size>()) {
			std::cerr
				<< "Node " << name << " didn't fail to parse as "
				<< "Size" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	int testIntegerValue(const ValueNode &node, std::string_view name,
			     ValueType type, int64_t value)
	{
		uint64_t unsignedValue = static_cast<uint64_t>(value);
		std::string strValue = std::to_string(value);
		bool isSigned = type == ValueType::Int8 || type == ValueType::Int16 ||
				type == ValueType::Int32;
		bool isInteger8 = type == ValueType::Int8 || type == ValueType::UInt8;
		bool isInteger16 = type == ValueType::Int16 || type == ValueType::UInt16;

		/* All integers can be accessed as strings and double. */

		if (node.get<string>().value_or("") != strValue ||
		    node.get<string>("") != strValue) {
			std::cerr
				<< "Node " << name << " failed to parse as "
				<< "string" << std::endl;
			return TestFail;
		}

		if (node.get<double>().value_or(0.0) != value ||
		    node.get<double>(0.0) != value) {
			std::cerr
				<< "Node " << name << " failed to parse as "
				<< "double" << std::endl;
			return TestFail;
		}

		if (isInteger8) {
			if (node.get<int8_t>().value_or(0) != value ||
			    node.get<int8_t>(0) != value) {
				std::cerr
					<< "Node " << name << " failed to parse as "
					<< "int8_t" << std::endl;
				return TestFail;
			}
		}

		if (isInteger8 && !isSigned) {
			if (node.get<uint8_t>().value_or(0) != unsignedValue ||
			    node.get<uint8_t>(0) != unsignedValue) {
				std::cerr
					<< "Node " << name << " failed to parse as "
					<< "uint8_t" << std::endl;
				return TestFail;
			}
		}

		if (isInteger8 || isInteger16) {
			if (node.get<int16_t>().value_or(0) != value ||
			    node.get<int16_t>(0) != value) {
				std::cerr
					<< "Node " << name << " failed to parse as "
					<< "int16_t" << std::endl;
				return TestFail;
			}
		}

		if ((isInteger8 || isInteger16) && !isSigned) {
			if (node.get<uint16_t>().value_or(0) != unsignedValue ||
			    node.get<uint16_t>(0) != unsignedValue) {
				std::cerr
					<< "Node " << name << " failed to parse as "
					<< "uint16_t" << std::endl;
				return TestFail;
			}
		}

		if (node.get<int32_t>().value_or(0) != value ||
		    node.get<int32_t>(0) != value) {
			std::cerr
				<< "Node " << name << " failed to parse as "
				<< "int32_t" << std::endl;
			return TestFail;
		}

		if (!isSigned) {
			if (node.get<uint32_t>().value_or(0) != unsignedValue ||
			    node.get<uint32_t>(0) != unsignedValue) {
				std::cerr
					<< "Node " << name << " failed to parse as "
					<< "uint32_t" << std::endl;
				return TestFail;
			}
		}

		return TestPass;
	}

	template<typename T>
	bool equal(const ValueNode &node, T value)
	{
		constexpr T eps = std::numeric_limits<T>::epsilon();

		if (std::abs(node.get<T>().value_or(0.0) - value) >= eps)
			return false;
		if (std::abs(node.get<T>(0.0) - value) >= eps)
			return false;
		return true;
	}

	int testFloatValue(const ValueNode &node, std::string_view name, double value)
	{
		std::string strValue = std::to_string(value);

		if (node.get<string>().value_or("") != strValue ||
		    node.get<string>("") != strValue) {
			std::cerr
				<< "Node " << name << " failed to parse as "
				<< "string" << std::endl;
			return TestFail;
		}

		if (!equal<float>(node, value)) {
			std::cerr
				<< "Node " << name << " failed to parse as "
				<< "float" << std::endl;
			return TestFail;
		}

		if (!equal<double>(node, value)) {
			std::cerr
				<< "Node " << name << " failed to parse as "
				<< "double" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	bool testNodeType(const ValueNode &node, NodeType nodeType)
	{
		using NodeFunc = bool (ValueNode::*)() const;
		using NodeDesc = std::tuple<NodeType, std::string_view, NodeFunc>;

		static constexpr std::array<NodeDesc, 4> nodeTypes = { {
			NodeDesc{ NodeType::Empty, "empty", &ValueNode::isEmpty },
			NodeDesc{ NodeType::Value, "value", &ValueNode::isValue },
			NodeDesc{ NodeType::List, "list", &ValueNode::isList },
			NodeDesc{ NodeType::Dictionary, "dictionary", &ValueNode::isDictionary },
		} };

		for (const auto &[type, name, func] : nodeTypes) {
			bool value = type == nodeType;
			if ((node.*func)() != value) {
				std::cerr
					<< "Empty ValueNode should "
					<< (value ? "" : "not ") << "be a "
					<< name << std::endl;
				return false;
			}
		}

		return true;
	}

	int run()
	{
		/* Tests on empty nodes. */
		ValueNode emptyNode;

		if (!testNodeType(emptyNode, NodeType::Empty)) {
			std::cerr
				<< "Empty node should have empty type"
				<< std::endl;
			return TestFail;
		}

		if (static_cast<bool>(emptyNode)) {
			std::cerr
				<< "Empty node should cast to false"
				<< std::endl;
			return TestFail;
		}

		if (emptyNode.size()) {
			std::cerr
				<< "Empty node should have zero size"
				<< std::endl;
			return TestFail;
		}

		if (emptyNode.get<std::string>()) {
			std::cerr
				<< "Empty node should have no value"
				<< std::endl;
			return TestFail;
		}

		/* Tests on list nodes. */
		ValueNode listNode;

		static constexpr std::array<std::string_view, 3> listElemNames = {
			"libcamera", "linux", "isp"
		};

		for (const auto &name : listElemNames)
			listNode.add(std::make_unique<ValueNode>(std::string{ name }));

		if (!testNodeType(listNode, NodeType::List))
			return TestFail;

		if (!static_cast<bool>(listNode)) {
			std::cerr
				<< "List node should cast to true"
				<< std::endl;
			return TestFail;
		}

		if (listNode.size() != 3) {
			std::cerr << "Invalid list node size" << std::endl;
			return TestFail;
		}

		listNode.set("value"s);
		if (listNode.get<std::string>()) {
			std::cerr
				<< "Setting a value on a list node should fail"
				<< std::endl;
			return TestFail;
		}

		std::set<std::string_view> names{
			listElemNames.begin(), listElemNames.end()
		};

		for (const auto &child : listNode.asList()) {
			const std::string childName = child.get<std::string>("");

			if (!names.erase(childName)) {
				std::cerr
					<< "Invalid list child '" << childName
					<< "'" << std::endl;
				return TestFail;
			}
		}

		if (!names.empty()) {
			std::cerr
				<< "Missing elements in list: "
				<< utils::join(names, ", ") << std::endl;
			return TestFail;
		}

		/* Tests on dictionary nodes. */
		ValueNode dictNode;

		static const std::array<std::pair<std::string, int>, 3> dictElemKeyValues = { {
			{ "a", 1 },
			{ "b", 2 },
			{ "c", 3 },
		} };

		for (const auto &[key, value] : dictElemKeyValues)
			dictNode.add(key, std::make_unique<ValueNode>(value));

		if (!testNodeType(dictNode, NodeType::Dictionary))
			return TestFail;

		if (!static_cast<bool>(dictNode)) {
			std::cerr
				<< "Dictionary node should cast to true"
				<< std::endl;
			return TestFail;
		}

		if (dictNode.size() != 3) {
			std::cerr << "Invalid dictionary node size" << std::endl;
			return TestFail;
		}

		dictNode.set("value"s);
		if (dictNode.get<std::string>()) {
			std::cerr
				<< "Setting a value on a dict node should fail"
				<< std::endl;
			return TestFail;
		}

		std::map<std::string, int> keyValues{
			dictElemKeyValues.begin(), dictElemKeyValues.end()
		};

		for (const auto &[key, child] : dictNode.asDict()) {
			auto iter = keyValues.find(key);
			if (iter == keyValues.end()) {
				std::cerr
					<< "Invalid dictionary key '" << key
					<< "'" << std::endl;
				return TestFail;
			}

			const int value = child.get<int>(0);
			if (value != iter->second) {
				std::cerr
					<< "Invalid dictionary value " << value
					<< " for key '" << key << "'" << std::endl;
				return TestFail;
			}

			if (dictNode[key].get<int>(0) != value) {
				std::cerr
					<< "Dictionary lookup failed for key '"
					<< key << "'" << std::endl;
				return TestFail;
			}

			keyValues.erase(iter);
		}

		if (!keyValues.empty()) {
			std::cerr
				<< "Missing elements in dictionary: "
				<< utils::join(utils::map_keys(keyValues), ", ")
				<< std::endl;
			return TestFail;
		}

		if (!dictNode["nonexistent"].isEmpty()) {
			std::cerr
				<< "Accessing nonexistent dictionary element returns non-empty node"
				<< std::endl;
			return TestFail;
		}

		/* Make sure utils::map_keys() works on the adapter. */
		(void)utils::map_keys(dictNode.asDict());

		/* Tests on value nodes. */
		ValueNode values;

		values.add("int8_t", std::make_unique<ValueNode>(static_cast<int8_t>(-100)));
		values.add("uint8_t", std::make_unique<ValueNode>(static_cast<uint8_t>(100)));
		values.add("int16_t", std::make_unique<ValueNode>(static_cast<int16_t>(-1000)));
		values.add("uint16_t", std::make_unique<ValueNode>(static_cast<uint16_t>(1000)));
		values.add("int32_t", std::make_unique<ValueNode>(static_cast<int32_t>(-100000)));
		values.add("uint32_t", std::make_unique<ValueNode>(static_cast<uint32_t>(100000)));
		values.add("float", std::make_unique<ValueNode>(3.14159f));
		values.add("double", std::make_unique<ValueNode>(3.14159));
		values.add("string", std::make_unique<ValueNode>("libcamera"s));

		std::unique_ptr<ValueNode> sizeNode = std::make_unique<ValueNode>();
		sizeNode->add(std::make_unique<ValueNode>(640));
		sizeNode->add(std::make_unique<ValueNode>(480));

		values.add("size", std::move(sizeNode));

		using ValueVariant = std::variant<int64_t, double, Size, std::string>;

		static const
		std::array<std::tuple<std::string_view, ValueType, ValueVariant>, 10> nodesValues{ {
			{ "int8_t", ValueType::Int8, static_cast<int64_t>(-100) },
			{ "uint8_t", ValueType::UInt8, static_cast<int64_t>(100) },
			{ "int16_t", ValueType::Int16, static_cast<int64_t>(-1000) },
			{ "uint16_t", ValueType::UInt16, static_cast<int64_t>(1000) },
			{ "int32_t", ValueType::Int32, static_cast<int64_t>(-100000) },
			{ "uint32_t", ValueType::UInt32, static_cast<int64_t>(100000) },
			{ "float", ValueType::Float, 3.14159 },
			{ "double", ValueType::Double, 3.14159 },
			{ "string", ValueType::String, "libcamera" },
			{ "size", ValueType::Size, Size{ 640, 480 } },
		} };

		for (const auto &nodeValue : nodesValues) {
			/*
			 * P0588R1 (https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2017/p0588r1.html)
			 * explicitly forbids a lambda from capturing structured
			 * bindings. This was fixed in a later release of the
			 * C++ specification, but some compilers (including
			 * clang-14 used in CI) choke on it. We can't use
			 * structured bindings in the for loop, unpack the tuple
			 * manually instead.
			 */
			const auto &name = std::get<0>(nodeValue);
			const auto &type = std::get<1>(nodeValue);
			const auto &value = std::get<2>(nodeValue);

			const ValueNode &node = values[name];

			if (testNodeValueType(node, name, type) != TestPass)
				return TestFail;

			int ret = std::visit(utils::overloaded{
				[&](int64_t arg) -> int {
					return testIntegerValue(node, name, type, arg);
				},

				[&](double arg) -> int {
					return testFloatValue(node, name, arg);
				},

				[&](const Size &arg) -> int {
					if (node.get<Size>().value_or(Size{}) != arg ||
					    node.get<Size>(Size{}) != arg) {
						std::cerr
							<< "Invalid node size value"
							<< std::endl;
						return TestFail;
					}

					return TestPass;
				},

				[&](const std::string &arg) -> int {
					if (node.get<std::string>().value_or(std::string{}) != arg ||
					    node.get<std::string>(std::string{}) != arg) {
						std::cerr
							<< "Invalid node string value"
							<< std::endl;
						return TestFail;
					}

					return TestPass;
				},
			}, value);

			if (ret != TestPass)
				return ret;
		}

		/* Test erasure. */
		values.erase("float");
		if (values.contains("float")) {
			std::cerr << "Failed to erase child node" << std::endl;
			return TestFail;
		}

		values.add({ "a", "b", "c" }, std::make_unique<ValueNode>(0));
		values.erase({ "a", "b" });
		if (values["a"].contains("b")) {
			std::cerr << "Failed to erase descendant node" << std::endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ValueNodeTest)
