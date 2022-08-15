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
	"int8_t: -100\n"
	"uint8_t: 100\n"
	"int16_t: -1000\n"
	"uint16_t: 1000\n"
	"int32_t: -100000\n"
	"uint32_t: 100000\n"
	"size: [1920, 1080]\n"
	"list:\n"
	"  - James\n"
	"  - Mary\n"
	"dictionary:\n"
	"  a: 1\n"
	"  c: 3\n"
	"  b: 2\n"
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

	enum class Type {
		String,
		Int8,
		UInt8,
		Int16,
		UInt16,
		Int32,
		UInt32,
		Double,
		Size,
		List,
		Dictionary,
	};

	int testObjectType(const YamlObject &obj, const char *name, Type type)
	{
		bool isList = type == Type::List || type == Type::Size;
		bool isScalar = !isList && type != Type::Dictionary;
		bool isInteger8 = type == Type::Int8 || type == Type::UInt8;
		bool isInteger16 = type == Type::Int16 || type == Type::UInt16;
		bool isInteger32 = type == Type::Int32 || type == Type::UInt32;
		bool isIntegerUpTo16 = isInteger8 || isInteger16;
		bool isIntegerUpTo32 = isIntegerUpTo16 || isInteger32;
		bool isSigned = type == Type::Int8 || type == Type::Int16 ||
			        type == Type::Int32;

		if ((isScalar && !obj.isValue()) || (!isScalar && obj.isValue())) {
			std::cerr
				<< "Object " << name << " type mismatch when compared to "
				<< "value" << std::endl;
			return TestFail;
		}

		if ((isList && !obj.isList()) || (!isList && obj.isList())) {
			std::cerr
				<< "Object " << name << " type mismatch when compared to "
				<< "list" << std::endl;
			return TestFail;
		}

		if ((type == Type::Dictionary && !obj.isDictionary()) ||
		    (type != Type::Dictionary && obj.isDictionary())) {
			std::cerr
				<< "Object " << name << " type mismatch when compared to "
				<< "dictionary" << std::endl;
			return TestFail;
		}

		if (!isScalar && obj.get<std::string>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "string" << std::endl;
			return TestFail;
		}

		if (!isInteger8 && obj.get<int8_t>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "int8_t" << std::endl;
			return TestFail;
		}

		if ((!isInteger8 || isSigned) && obj.get<uint8_t>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "uint8_t" << std::endl;
			return TestFail;
		}

		if (!isIntegerUpTo16 && obj.get<int16_t>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "int16_t" << std::endl;
			return TestFail;
		}

		if ((!isIntegerUpTo16 || isSigned) && obj.get<uint16_t>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "uint16_t" << std::endl;
			return TestFail;
		}

		if (!isIntegerUpTo32 && obj.get<int32_t>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "int32_t" << std::endl;
			return TestFail;
		}

		if ((!isIntegerUpTo32 || isSigned) && obj.get<uint32_t>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "uint32_t" << std::endl;
			return TestFail;
		}

		if (!isIntegerUpTo32 && type != Type::Double && obj.get<double>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "double" << std::endl;
			return TestFail;
		}

		if (type != Type::Size && obj.get<Size>()) {
			std::cerr
				<< "Object " << name << " didn't fail to parse as "
				<< "Size" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	int testIntegerObject(const YamlObject &obj, const char *name, Type type,
			      int64_t value)
	{
		uint64_t unsignedValue = static_cast<uint64_t>(value);
		std::string strValue = std::to_string(value);
		bool isInteger8 = type == Type::Int8 || type == Type::UInt8;
		bool isInteger16 = type == Type::Int16 || type == Type::UInt16;
		bool isSigned = type == Type::Int8 || type == Type::Int16 ||
				type == Type::Int32;

		/* All integers can be parsed as strings or double. */

		if (obj.get<string>().value_or("") != strValue ||
		    obj.get<string>("") != strValue) {
			std::cerr
				<< "Object " << name << " failed to parse as "
				<< "string" << std::endl;
			return TestFail;
		}

		if (obj.get<double>().value_or(0.0) != value ||
		    obj.get<double>(0.0) != value) {
			std::cerr
				<< "Object " << name << " failed to parse as "
				<< "double" << std::endl;
			return TestFail;
		}

		if (isInteger8) {
			if (obj.get<int8_t>().value_or(0) != value ||
			    obj.get<int8_t>(0) != value) {
				std::cerr
					<< "Object " << name << " failed to parse as "
					<< "int8_t" << std::endl;
				return TestFail;
			}
		}

		if (isInteger8 && !isSigned) {
			if (obj.get<uint8_t>().value_or(0) != unsignedValue ||
			    obj.get<uint8_t>(0) != unsignedValue) {
				std::cerr
					<< "Object " << name << " failed to parse as "
					<< "uint8_t" << std::endl;
				return TestFail;
			}
		}

		if (isInteger8 || isInteger16) {
			if (obj.get<int16_t>().value_or(0) != value ||
			    obj.get<int16_t>(0) != value) {
				std::cerr
					<< "Object " << name << " failed to parse as "
					<< "int16_t" << std::endl;
				return TestFail;
			}
		}

		if ((isInteger8 || isInteger16) && !isSigned) {
			if (obj.get<uint16_t>().value_or(0) != unsignedValue ||
			    obj.get<uint16_t>(0) != unsignedValue) {
				std::cerr
					<< "Object " << name << " failed to parse as "
					<< "uint16_t" << std::endl;
				return TestFail;
			}
		}

		if (obj.get<int32_t>().value_or(0) != value ||
		    obj.get<int32_t>(0) != value) {
			std::cerr
				<< "Object " << name << " failed to parse as "
				<< "int32_t" << std::endl;
			return TestFail;
		}

		if (!isSigned) {
			if (obj.get<uint32_t>().value_or(0) != unsignedValue ||
			    obj.get<uint32_t>(0) != unsignedValue) {
				std::cerr
					<< "Object " << name << " failed to parse as "
					<< "uint32_t" << std::endl;
				return TestFail;
			}
		}

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

		std::vector<const char *> rootElemNames = {
			"string", "double", "int8_t", "uint8_t", "int16_t",
			"uint16_t", "int32_t", "uint32_t", "size", "list",
			"dictionary", "level1",
		};

		for (const char *name : rootElemNames) {
			if (!root->contains(name)) {
				cerr << "Missing " << name << " object in YAML root"
				     << std::endl;
				return TestFail;
			}
		}

		/* Test string object */
		auto &strObj = (*root)["string"];

		if (testObjectType(strObj, "string", Type::String) != TestPass)
			return TestFail;

		if (strObj.get<string>().value_or("") != "libcamera" ||
		    strObj.get<string>("") != "libcamera") {
			cerr << "String object parse as wrong content" << std::endl;
			return TestFail;
		}

		/* Test int8_t object */
		auto &int8Obj = (*root)["int8_t"];

		if (testObjectType(int8Obj, "int8_t", Type::Int8) != TestPass)
			return TestFail;

		if (testIntegerObject(int8Obj, "int8_t", Type::Int8, -100) != TestPass)
			return TestFail;

		/* Test uint8_t object */
		auto &uint8Obj = (*root)["uint8_t"];

		if (testObjectType(uint8Obj, "uint8_t", Type::UInt8) != TestPass)
			return TestFail;

		if (testIntegerObject(uint8Obj, "uint8_t", Type::UInt8, 100) != TestPass)
			return TestFail;

		/* Test int16_t object */
		auto &int16Obj = (*root)["int16_t"];

		if (testObjectType(int16Obj, "int16_t", Type::Int16) != TestPass)
			return TestFail;

		if (testIntegerObject(int16Obj, "int16_t", Type::Int16, -1000) != TestPass)
			return TestFail;

		/* Test uint16_t object */
		auto &uint16Obj = (*root)["uint16_t"];

		if (testObjectType(uint16Obj, "uint16_t", Type::UInt16) != TestPass)
			return TestFail;

		if (testIntegerObject(uint16Obj, "uint16_t", Type::UInt16, 1000) != TestPass)
			return TestFail;

		/* Test int32_t object */
		auto &int32Obj = (*root)["int32_t"];

		if (testObjectType(int32Obj, "int32_t", Type::Int32) != TestPass)
			return TestFail;

		if (testIntegerObject(int32Obj, "int32_t", Type::Int32, -100000) != TestPass)
			return TestFail;

		/* Test uint32_t object */
		auto &uint32Obj = (*root)["uint32_t"];

		if (testObjectType(uint32Obj, "uint32_t", Type::UInt32) != TestPass)
			return TestFail;

		if (testIntegerObject(uint32Obj, "uint32_t", Type::UInt32, 100000) != TestPass)
			return TestFail;

		/* Test double value */
		auto &doubleObj = (*root)["double"];

		if (testObjectType(doubleObj, "double", Type::Double) != TestPass)
			return TestFail;

		if (doubleObj.get<string>().value_or("") != "3.14159" ||
		    doubleObj.get<string>("") != "3.14159") {
			cerr << "Double object fail to parse as string" << std::endl;
			return TestFail;
		}

		if (doubleObj.get<double>().value_or(0.0) != 3.14159 ||
		    doubleObj.get<double>(0.0) != 3.14159) {
			cerr << "Double object parse as wrong value" << std::endl;
			return TestFail;
		}

		/* Test Size value */
		auto &sizeObj = (*root)["size"];

		if (testObjectType(sizeObj, "size", Type::Size) != TestPass)
			return TestFail;

		if (sizeObj.get<Size>().value_or(Size(0, 0)) != Size(1920, 1080) ||
		    sizeObj.get<Size>(Size(0, 0)) != Size(1920, 1080)) {
			cerr << "Size object parse as wrong value" << std::endl;
			return TestFail;
		}

		/* Test list object */
		auto &listObj = (*root)["list"];

		if (testObjectType(listObj, "list", Type::List) != TestPass)
			return TestFail;

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

		if (testObjectType(dictObj, "dictionary", Type::Dictionary) != TestPass)
			return TestFail;

		static constexpr std::array<std::pair<const char *, int>, 3> dictValues{ {
			{ "a", 1 },
			{ "c", 3 },
			{ "b", 2 },
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

			const auto &item = dictValues[i];
			if (item.first != key) {
				std::cerr << "Dictionary key " << i << " has wrong value"
					  << std::endl;
				return TestFail;
			}

			if (&elem != &dictObj[key]) {
				std::cerr << "Dictionary element " << i << " has wrong address"
					  << std::endl;
				return TestFail;
			}

			if (elem.get<int32_t>(0) != item.second) {
				std::cerr << "Dictionary element " << i << " has wrong value"
					  << std::endl;
				return TestFail;
			}

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

		const auto &values = firstElement.getList<uint16_t>();
		if (!values || values->size() != 2 || (*values)[0] != 1 || (*values)[1] != 2) {
			cerr << "getList() failed to return correct vector" << std::endl;
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
