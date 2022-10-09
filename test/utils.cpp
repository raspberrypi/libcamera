/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * utils.cpp - Miscellaneous utility tests
 */

#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace std::literals::chrono_literals;

class UtilsTest : public Test
{
protected:
	int testDirname()
	{
		static const std::vector<std::string> paths = {
			"",
			"///",
			"/bin",
			"/usr/bin",
			"//etc////",
			"//tmp//d//",
			"current_file",
			"./current_file",
			"./current_dir/",
			"current_dir/",
		};

		static const std::vector<std::string> expected = {
			".",
			"/",
			"/",
			"/usr",
			"/",
			"//tmp",
			".",
			".",
			".",
			".",
		};

		std::vector<std::string> results;

		for (const auto &path : paths)
			results.push_back(utils::dirname(path));

		if (results != expected) {
			cerr << "utils::dirname() tests failed" << endl;

			cerr << "expected: " << endl;
			for (const auto &path : expected)
				cerr << "\t" << path << endl;

			cerr << "results: " << endl;
			for (const auto &path : results)
				cerr << "\t" << path << endl;

			return TestFail;
		}

		return TestPass;
	}

	int testEnumerate()
	{
		std::vector<unsigned int> integers{ 1, 2, 3, 4, 5 };
		unsigned int i = 0;

		for (auto [index, value] : utils::enumerate(integers)) {
			if (index != i || value != i + 1) {
				cerr << "utils::enumerate(<vector>) test failed: i=" << i
				     << ", index=" << index << ", value=" << value
				     << std::endl;
				return TestFail;
			}

			/* Verify that we can modify the value. */
			--value;
			++i;
		}

		if (integers != std::vector<unsigned int>{ 0, 1, 2, 3, 4 }) {
			cerr << "Failed to modify container in enumerated range loop" << endl;
			return TestFail;
		}

		Span<const unsigned int> span{ integers };
		i = 0;

		for (auto [index, value] : utils::enumerate(span)) {
			if (index != i || value != i) {
				cerr << "utils::enumerate(<span>) test failed: i=" << i
				     << ", index=" << index << ", value=" << value
				     << std::endl;
				return TestFail;
			}

			++i;
		}

		const unsigned int array[] = { 0, 2, 4, 6, 8 };
		i = 0;

		for (auto [index, value] : utils::enumerate(array)) {
			if (index != i || value != i * 2) {
				cerr << "utils::enumerate(<array>) test failed: i=" << i
				     << ", index=" << index << ", value=" << value
				     << std::endl;
				return TestFail;
			}

			++i;
		}

		return TestPass;
	}

	int testDuration()
	{
		std::ostringstream os;
		utils::Duration exposure;
		double ratio;

		exposure = 25ms + 25ms;
		if (exposure.get<std::micro>() != 50000.0) {
			cerr << "utils::Duration failed to return microsecond count";
			return TestFail;
		}

		exposure = 1.0s / 4;
		if (exposure != 250ms) {
			cerr << "utils::Duration failed scalar divide test";
			return TestFail;
		}

		exposure = 5000.5us;
		if (!exposure) {
			cerr << "utils::Duration failed boolean test";
			return TestFail;
		}

		os << exposure;
		if (os.str() != "5000.50us") {
			cerr << "utils::Duration operator << failed";
			return TestFail;
		}

		exposure = 100ms;
		ratio = exposure / 25ms;
		if (ratio != 4.0) {
			cerr << "utils::Duration failed ratio test";
			return TestFail;
		}

		return TestPass;
	}

	int run()
	{
		/* utils::hex() test. */
		std::ostringstream os;
		std::string ref;

		os << utils::hex(static_cast<int32_t>(0x42)) << " ";
		ref += "0x00000042 ";
		os << utils::hex(static_cast<uint32_t>(0x42)) << " ";
		ref += "0x00000042 ";
		os << utils::hex(static_cast<int64_t>(0x42)) << " ";
		ref += "0x0000000000000042 ";
		os << utils::hex(static_cast<uint64_t>(0x42)) << " ";
		ref += "0x0000000000000042 ";
		os << utils::hex(static_cast<int32_t>(0x42), 4) << " ";
		ref += "0x0042 ";
		os << utils::hex(static_cast<uint32_t>(0x42), 1) << " ";
		ref += "0x42 ";
		os << utils::hex(static_cast<int64_t>(0x42), 4) << " ";
		ref += "0x0042 ";
		os << utils::hex(static_cast<uint64_t>(0x42), 1) << " ";
		ref += "0x42 ";

		std::string s = os.str();
		if (s != ref) {
			cerr << "utils::hex() test failed, expected '" << ref
			     << "', got '" << s << "'";
			return TestFail;
		}

		/* utils::join() and utils::split() test. */
		std::vector<std::string> elements = {
			"/bin",
			"/usr/bin",
			"",
			"",
		};

		std::string path;
		for (const auto &element : elements)
			path += (path.empty() ? "" : ":") + element;

		if (path != utils::join(elements, ":")) {
			cerr << "utils::join() test failed" << endl;
			return TestFail;
		}

		std::vector<std::string> dirs;

		for (const auto &dir : utils::split(path, ":"))
			dirs.push_back(dir);

		if (dirs != elements) {
			cerr << "utils::split() test failed" << endl;
			return TestFail;
		}

		const auto &split = utils::split(path, ":");
		dirs = std::vector<std::string>{ split.begin(), split.end() };

		if (dirs != elements) {
			cerr << "utils::split() LegacyInputIterator test failed" << endl;
			return TestFail;
		}

		/* utils::join() with conversion function test. */
		std::vector<Size> sizes = { { 0, 0 }, { 100, 100 } };
		s = utils::join(sizes, "/", [](const Size &size) {
					return size.toString();
				});

		if (s != "0x0/100x100") {
			cerr << "utils::join() with conversion test failed" << endl;
			return TestFail;
		}

		/* utils::dirname() tests. */
		if (TestPass != testDirname())
			return TestFail;


		/* utils::map_keys() test. */
		const std::map<std::string, unsigned int> map{
			{ "zero", 0 },
			{ "one", 1 },
			{ "two", 2 },
		};
		std::vector<std::string> expectedKeys{
			"zero",
			"one",
			"two",
		};

		std::sort(expectedKeys.begin(), expectedKeys.end());

		const std::vector<std::string> keys = utils::map_keys(map);
		if (keys != expectedKeys) {
			cerr << "utils::map_keys() test failed" << endl;
			return TestFail;
		}

		/* utils::alignUp() and utils::alignDown() tests. */
		if (utils::alignDown(6, 3) != 6 || utils::alignDown(7, 3) != 6) {
			cerr << "utils::alignDown test failed" << endl;
			return TestFail;
		}

		if (utils::alignUp(6, 3) != 6 || utils::alignUp(7, 3) != 9) {
			cerr << "utils::alignUp test failed" << endl;
			return TestFail;
		}

		/* utils::enumerate() test. */
		if (testEnumerate() != TestPass)
			return TestFail;

		/* utils::Duration test. */
		if (testDuration() != TestPass)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(UtilsTest)
