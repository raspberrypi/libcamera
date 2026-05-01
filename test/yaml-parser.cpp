/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * YAML parser operations tests
 */

#include <array>
#include <iostream>
#include <map>
#include <string>
#include <unistd.h>

#include <libcamera/base/file.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/yaml_parser.h"

#include "test.h"

using namespace libcamera;

static const std::string testYaml =
	"empty:\n"
	"value: 42\n"
	"list:\n"
	"  - libcamera\n"
	"  - linux\n"
	"  - \n"
	"level1:\n"
	"  level2:\n"
	"    - [1, 2]\n"
	"    - {one: 1, two: 2}\n";

static const std::string invalidYaml =
	"Invalid : - YAML : - Content";

class YamlParserTest : public Test
{
protected:
	bool createFile(const std::string &content, std::string &filename)
	{
		filename = "/tmp/libcamera.test.XXXXXX";
		int fd = mkstemp(&filename.front());
		if (fd == -1)
			return false;

		int ret = write(fd, content.c_str(), content.size());
		close(fd);

		if (ret != static_cast<int>(content.size()))
			return false;

		return true;
	}

	int init()
	{
		if (!createFile(testYaml, testYamlFile_))
			return TestFail;

		if (!createFile(invalidYaml, invalidYamlFile_))
			return TestFail;

		return TestPass;
	}

	int run()
	{
		/* Test parsing invalid YAML file. */
		File file{ invalidYamlFile_ };
		if (!file.open(File::OpenModeFlag::ReadOnly)) {
			std::cerr << "Failed to open invalid YAML file" << std::endl;
			return TestFail;
		}

		std::unique_ptr<ValueNode> root = YamlParser::parse(file);
		if (root) {
			std::cerr << "Invalid YAML file parsed successfully" << std::endl;
			return TestFail;
		}

		/* Test parsing valid YAML file. */
		file.close();
		file.setFileName(testYamlFile_);
		if (!file.open(File::OpenModeFlag::ReadOnly)) {
			std::cerr << "Fail to open test YAML file" << std::endl;
			return TestFail;
		}

		root = YamlParser::parse(file);

		if (!root) {
			std::cerr << "Fail to parse test YAML file: " << std::endl;
			return TestFail;
		}

		/* Test that the root dictionary node has been parsed correctly. */
		if (!root->isDictionary()) {
			std::cerr << "Dictionary node has wrong type" << std::endl;
			return TestFail;
		}

		using NodeFunc = bool (ValueNode::*)() const;

		std::map<std::string, NodeFunc> topLevelNodes = { {
			{ "empty", &ValueNode::isValue },
			{ "value", &ValueNode::isValue },
			{ "list", &ValueNode::isList },
			{ "level1", &ValueNode::isDictionary },
		} };

		if (root->size() != topLevelNodes.size()) {
			std::cerr << "Dictionary node has wrong size" << std::endl;
			return TestFail;
		}

		for (const auto &[key, value] : root->asDict()) {
			const auto iter = topLevelNodes.find(key);
			if (iter == topLevelNodes.end()) {
				std::cerr << "Dictionary key '" << key << "' unknown"
					  << std::endl;
				return TestFail;
			}

			const auto &func = iter->second;
			if (!(value.*func)()) {
				std::cerr << "Node '" << key << "' has wrong type"
					  << std::endl;
				return TestFail;
			}

			topLevelNodes.erase(iter);
		}

		/* Test empty node. */
		auto &emptyNode = (*root)["empty"];

		if (emptyNode.get<std::string>("-") != "") {
			std::cerr << "Empty node has incorrect content" << std::endl;
			return TestFail;
		}

		/* Test value node. */
		auto &valueNode = (*root)["value"];

		if (valueNode.get<std::string>("") != "42") {
			std::cerr << "Value node has incorrect content" << std::endl;
			return TestFail;
		}

		/* Test list node. */
		auto &listNode = (*root)["list"];

		static constexpr std::array<const char *, 3> listValues{
			"libcamera",
			"linux",
			"",
		};

		if (listNode.size() != listValues.size()) {
			std::cerr << "List node parsed with wrong size" << std::endl;
			return TestFail;
		}

		unsigned int i = 0;
		for (auto &elem : listNode.asList()) {
			if (i >= listValues.size()) {
				std::cerr << "Too many elements in list during iteration"
					  << std::endl;
				return TestFail;
			}

			std::string value = listValues[i];

			if (&elem != &listNode[i]) {
				std::cerr << "List element " << i << " has wrong address"
					  << std::endl;
				return TestFail;
			}

			if (elem.get<std::string>("") != value) {
				std::cerr << "List element " << i << " has wrong value"
					  << std::endl;
				return TestFail;
			}

			i++;
		}

		/* Ensure that empty list elements get parsed as empty strings. */
		if (!listNode[2].isValue()) {
			std::cerr << "Empty list element is not a value" << std::endl;
			return TestFail;
		}

		/* Test nested nodes. */
		auto &level1Node = (*root)["level1"];

		if (!level1Node.isDictionary()) {
			std::cerr << "level1 node failed to parse as Dictionary" << std::endl;
			return TestFail;
		}

		auto &level2Node = level1Node["level2"];

		if (!level2Node.isList() || level2Node.size() != 2) {
			std::cerr << "level2 node should be a 2 element list" << std::endl;
			return TestFail;
		}

		auto &firstElement = level2Node[0];
		if (!firstElement.isList() ||
		    firstElement.size() != 2 ||
		    firstElement[0].get<int32_t>(0) != 1 ||
		    firstElement[1].get<int32_t>(0) != 2) {
			std::cerr << "The first element of level2 node failed to parse as integer list" << std::endl;
			return TestFail;
		}

		const auto &values = firstElement.get<std::vector<uint16_t>>();
		if (!values || values->size() != 2 || (*values)[0] != 1 || (*values)[1] != 2) {
			std::cerr << "get() failed to return correct vector" << std::endl;
			return TestFail;
		}

		auto &secondElement = level2Node[1];
		if (!secondElement.isDictionary() ||
		    !secondElement.contains("one") ||
		    !secondElement.contains("two") ||
		    secondElement["one"].get<int32_t>(0) != 1 ||
		    secondElement["two"].get<int32_t>(0) != 2) {
			std::cerr << "The second element of level2 node failed to parse as dictionary" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		unlink(testYamlFile_.c_str());
		unlink(invalidYamlFile_.c_str());
	}

private:
	std::string testYamlFile_;
	std::string invalidYamlFile_;
};

TEST_REGISTER(YamlParserTest)
