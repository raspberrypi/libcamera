/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * yaml-parser.cpp - YAML parser operations tests
 */

#include <array>
#include <iostream>
#include <map>
#include <string>
#include <unistd.h>

#include <libcamera/base/file.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "test.h"

using namespace libcamera;
using namespace std;

static const string testYaml =
	"string: libcamera\n"
	"double: 3.14159\n"
	"uint32_t: 100\n"
	"int32_t: -100\n"
	"size: [1920, 1080]\n"
	"list:\n"
	"  - James\n"
	"  - Mary\n"
	"dictionary:\n"
	"  a: 1\n"
	"  b: 2\n"
	"  c: 3\n"
	"level1:\n"
	"  level2:\n"
	"    - [1, 2]\n"
	"    - {one: 1, two: 2}\n";

static const string invalidYaml =
	"Invalid : - YAML : - Content";

class YamlParserTest : public Test
{
protected:
	bool createFile(const string &content, string &filename)
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
		/* Test invalid YAML file */
		File file{ invalidYamlFile_ };
		if (!file.open(File::OpenModeFlag::ReadOnly)) {
			cerr << "Fail to open invalid YAML file" << std::endl;
			return TestFail;
		}

		std::unique_ptr<YamlObject> root = YamlParser::parse(file);
		if (root) {
			cerr << "Invalid YAML file parse successfully" << std::endl;
			return TestFail;
		}

		/* Test YAML file */
		file.close();
		file.setFileName(testYamlFile_);
		if (!file.open(File::OpenModeFlag::ReadOnly)) {
			cerr << "Fail to open test YAML file" << std::endl;
			return TestFail;
		}

		root = YamlParser::parse(file);

		if (!root) {
			cerr << "Fail to parse test YAML file: " << std::endl;
			return TestFail;
		}

		if (!root->isDictionary()) {
			cerr << "YAML root is not dictionary" << std::endl;
			return TestFail;
		}

		if (!root->contains("string")) {
			cerr << "Missing string object in YAML root" << std::endl;
			return TestFail;
		}

		if (!root->contains("double")) {
			cerr << "Missing double object in YAML root" << std::endl;
			return TestFail;
		}

		if (!root->contains("int32_t")) {
			cerr << "Missing int32_t object in YAML root" << std::endl;
			return TestFail;
		}

		if (!root->contains("uint32_t")) {
			cerr << "Missing uint32_t object in YAML root" << std::endl;
			return TestFail;
		}

		if (!root->contains("size")) {
			cerr << "Missing Size object in YAML root" << std::endl;
			return TestFail;
		}

		if (!root->contains("list")) {
			cerr << "Missing list object in YAML root" << std::endl;
			return TestFail;
		}

		if (!root->contains("dictionary")) {
			cerr << "Missing dictionary object in YAML root" << std::endl;
			return TestFail;
		}

		if (!root->contains("level1")) {
			cerr << "Missing leveled object in YAML root" << std::endl;
			return TestFail;
		}

		/* Test string object */
		bool ok;
		auto &strObj = (*root)["string"];

		if (strObj.isDictionary()) {
			cerr << "String object parse as Dictionary" << std::endl;
			return TestFail;
		}

		if (strObj.isList()) {
			cerr << "String object parse as List" << std::endl;
			return TestFail;
		}

		if (strObj.get<string>("", &ok) != "libcamera" || !ok) {
			cerr << "String object parse as wrong content" << std::endl;
			return TestFail;
		}

		if (strObj.get<int32_t>(-1, &ok) != -1 || ok) {
			cerr << "String object parse as integer" << std::endl;
			return TestFail;
		}

		if (strObj.get<uint32_t>(1, &ok) != 1 || ok) {
			cerr << "String object parse as unsigned integer" << std::endl;
			return TestFail;
		}

		if (strObj.get<double>(1.0, &ok) != 1.0 || ok) {
			cerr << "String object parse as double" << std::endl;
			return TestFail;
		}

		if (strObj.get<Size>(Size(0, 0), &ok) != Size(0, 0) || ok) {
			cerr << "String object parse as Size" << std::endl;
			return TestFail;
		}

		/* Test int32_t object */
		auto &int32Obj = (*root)["int32_t"];

		if (int32Obj.isDictionary()) {
			cerr << "Integer object parse as Dictionary" << std::endl;
			return TestFail;
		}

		if (int32Obj.isList()) {
			cerr << "Integer object parse as Integer" << std::endl;
			return TestFail;
		}

		if (int32Obj.get<int32_t>(-100, &ok) != -100 || !ok) {
			cerr << "Integer object parse as wrong value" << std::endl;
			return TestFail;
		}

		if (int32Obj.get<string>("", &ok) != "-100" || !ok) {
			cerr << "Integer object fail to parse as string" << std::endl;
			return TestFail;
		}

		if (int32Obj.get<double>(1.0, &ok) != -100.0 || !ok) {
			cerr << "Integer object fail to parse as double" << std::endl;
			return TestFail;
		}

		if (int32Obj.get<uint32_t>(1, &ok) != 1 || ok) {
			cerr << "Negative integer object parse as unsigned integer" << std::endl;
			return TestFail;
		}

		if (int32Obj.get<Size>(Size(0, 0), &ok) != Size(0, 0) || ok) {
			cerr << "Integer object parse as Size" << std::endl;
			return TestFail;
		}

		/* Test uint32_t object */
		auto &uint32Obj = (*root)["uint32_t"];

		if (uint32Obj.isDictionary()) {
			cerr << "Unsigned integer object parse as Dictionary" << std::endl;
			return TestFail;
		}

		if (uint32Obj.isList()) {
			cerr << "Unsigned integer object parse as List" << std::endl;
			return TestFail;
		}

		if (uint32Obj.get<int32_t>(-1, &ok) != 100 || !ok) {
			cerr << "Unsigned integer object fail to parse as integer" << std::endl;
			return TestFail;
		}

		if (uint32Obj.get<string>("", &ok) != "100" || !ok) {
			cerr << "Unsigned integer object fail to parse as string" << std::endl;
			return TestFail;
		}

		if (uint32Obj.get<double>(1.0, &ok) != 100.0 || !ok) {
			cerr << "Unsigned integer object fail to parse as double" << std::endl;
			return TestFail;
		}

		if (uint32Obj.get<uint32_t>(100, &ok) != 100 || !ok) {
			cerr << "Unsigned integer object parsed as wrong value" << std::endl;
			return TestFail;
		}

		if (uint32Obj.get<Size>(Size(0, 0), &ok) != Size(0, 0) || ok) {
			cerr << "Unsigned integer object parsed as Size" << std::endl;
			return TestFail;
		}

		/* Test double value */
		auto &doubleObj = (*root)["double"];

		if (doubleObj.isDictionary()) {
			cerr << "Double object parse as Dictionary" << std::endl;
			return TestFail;
		}

		if (doubleObj.isList()) {
			cerr << "Double object parse as List" << std::endl;
			return TestFail;
		}

		if (doubleObj.get<string>("", &ok) != "3.14159" || !ok) {
			cerr << "Double object fail to parse as string" << std::endl;
			return TestFail;
		}

		if (doubleObj.get<double>(1.0, &ok) != 3.14159 || !ok) {
			cerr << "Double object parse as wrong value" << std::endl;
			return TestFail;
		}

		if (doubleObj.get<int32_t>(-1, &ok) != -1 || ok) {
			cerr << "Double object parse as integer" << std::endl;
			return TestFail;
		}

		if (doubleObj.get<uint32_t>(1, &ok) != 1 || ok) {
			cerr << "Double object parse as unsigned integer" << std::endl;
			return TestFail;
		}

		if (doubleObj.get<Size>(Size(0, 0), &ok) != Size(0, 0) || ok) {
			cerr << "Double object parse as Size" << std::endl;
			return TestFail;
		}

		/* Test Size value */
		auto &sizeObj = (*root)["size"];

		if (sizeObj.isDictionary()) {
			cerr << "Size object parse as Dictionary" << std::endl;
			return TestFail;
		}

		if (!sizeObj.isList()) {
			cerr << "Size object parse as List" << std::endl;
			return TestFail;
		}

		if (sizeObj.get<string>("", &ok) != "" || ok) {
			cerr << "Size object parse as string" << std::endl;
			return TestFail;
		}

		if (sizeObj.get<double>(1.0, &ok) != 1.0 || ok) {
			cerr << "Size object parse as double" << std::endl;
			return TestFail;
		}

		if (sizeObj.get<int32_t>(-1, &ok) != -1 || ok) {
			cerr << "Size object parse as integer" << std::endl;
			return TestFail;
		}

		if (sizeObj.get<uint32_t>(1, &ok) != 1 || ok) {
			cerr << "Size object parse as unsigned integer" << std::endl;
			return TestFail;
		}

		if (sizeObj.get<Size>(Size(0, 0), &ok) != Size(1920, 1080) || !ok) {
			cerr << "Size object parse as wrong value" << std::endl;
			return TestFail;
		}

		/* Test list object */
		auto &listObj = (*root)["list"];

		if (listObj.isDictionary()) {
			cerr << "List object parse as Dictionary" << std::endl;
			return TestFail;
		}

		if (!listObj.isList()) {
			cerr << "List object fail to parse as List" << std::endl;
			return TestFail;
		}

		if (listObj.get<string>("", &ok) != "" || ok) {
			cerr << "List object parse as string" << std::endl;
			return TestFail;
		}

		if (listObj.get<double>(1.0, &ok) != 1.0 || ok) {
			cerr << "List object parse as double" << std::endl;
			return TestFail;
		}

		if (listObj.get<int32_t>(-1, &ok) != -1 || ok) {
			cerr << "List object parse as integer" << std::endl;
			return TestFail;
		}

		if (listObj.get<uint32_t>(1, &ok) != 1 || ok) {
			cerr << "List object parse as unsigne integer" << std::endl;
			return TestFail;
		}

		if (listObj.get<Size>(Size(0, 0), &ok) != Size(0, 0) || ok) {
			cerr << "String list object parse as Size" << std::endl;
			return TestFail;
		}

		static constexpr std::array<const char *, 2> listValues{
			"James",
			"Mary",
		};

		if (listObj.size() != listValues.size()) {
			cerr << "List object parse with wrong size" << std::endl;
			return TestFail;
		}

		unsigned int i = 0;
		for (auto &elem : listObj.asList()) {
			if (i >= listValues.size()) {
				std::cerr << "Too many elements in list during iteration"
					  << std::endl;
				return TestFail;
			}

			std::string value = listValues[i];

			if (&elem != &listObj[i]) {
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

		/* Test dictionary object */
		auto &dictObj = (*root)["dictionary"];

		if (!dictObj.isDictionary()) {
			cerr << "Dictionary object fail to parse as Dictionary" << std::endl;
			return TestFail;
		}

		if (dictObj.isList()) {
			cerr << "Dictionary object parse as List" << std::endl;
			return TestFail;
		}

		if (dictObj.get<string>("", &ok) != "" || ok) {
			cerr << "Dictionary object parse as string" << std::endl;
			return TestFail;
		}

		if (dictObj.get<double>(1.0, &ok) != 1.0 || ok) {
			cerr << "Dictionary object parse as double" << std::endl;
			return TestFail;
		}

		if (dictObj.get<int32_t>(-1, &ok) != -1 || ok) {
			cerr << "Dictionary object parse as integer" << std::endl;
			return TestFail;
		}

		if (dictObj.get<uint32_t>(1, &ok) != 1 || ok) {
			cerr << "Dictionary object parse as unsigned integer" << std::endl;
			return TestFail;
		}

		if (dictObj.get<Size>(Size(0, 0), &ok) != Size(0, 0) || ok) {
			cerr << "Dictionary object parse as Size" << std::endl;
			return TestFail;
		}

		std::map<std::string, int> dictValues{ {
			{ "a", 1 },
			{ "b", 2 },
			{ "c", 3 },
		} };

		size_t dictSize = dictValues.size();

		if (dictObj.size() != dictSize) {
			cerr << "Dictionary object has wrong size" << std::endl;
			return TestFail;
		}

		i = 0;
		for (const auto &[key, elem] : dictObj.asDict()) {
			if (i >= dictSize) {
				std::cerr << "Too many elements in dictionary during iteration"
					  << std::endl;
				return TestFail;
			}

			const auto item = dictValues.find(key);
			if (item == dictValues.end()) {
				std::cerr << "Dictionary key " << i << " has wrong value"
					  << std::endl;
				return TestFail;
			}

			if (&elem != &dictObj[key]) {
				std::cerr << "Dictionary element " << i << " has wrong address"
					  << std::endl;
				return TestFail;
			}

			if (elem.get<int32_t>(0) != item->second) {
				std::cerr << "Dictionary element " << i << " has wrong value"
					  << std::endl;
				return TestFail;
			}

			/*
			 * Erase the item to make sure that each iteration
			 * produces a different value.
			 */
			dictValues.erase(item);
			i++;
		}

		/* Make sure utils::map_keys() works on the adapter. */
		(void)utils::map_keys(dictObj.asDict());

		/* Test leveled objects */
		auto &level1Obj = (*root)["level1"];

		if (!level1Obj.isDictionary()) {
			cerr << "level1 object fail to parse as Dictionary" << std::endl;
			return TestFail;
		}

		auto &level2Obj = level1Obj["level2"];

		if (!level2Obj.isList() || level2Obj.size() != 2) {
			cerr << "level2 object should be 2 element list" << std::endl;
			return TestFail;
		}

		auto &firstElement = level2Obj[0];
		if (!firstElement.isList() ||
		    firstElement.size() != 2 ||
		    firstElement[0].get<int32_t>(0) != 1 ||
		    firstElement[1].get<int32_t>(0) != 2) {
			cerr << "The first element of level2 object fail to parse as integer list" << std::endl;
			return TestFail;
		}

		auto &secondElement = level2Obj[1];
		if (!secondElement.isDictionary() ||
		    !secondElement.contains("one") ||
		    !secondElement.contains("two") ||
		    secondElement["one"].get<int32_t>(0) != 1 ||
		    secondElement["two"].get<int32_t>(0) != 2) {
			cerr << "The second element of level2 object fail to parse as dictionary" << std::endl;
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
