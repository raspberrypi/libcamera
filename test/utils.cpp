/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * utils.cpp - Miscellaneous utility tests
 */

#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <libcamera/geometry.h>

#include "libcamera/internal/utils.h"

#include "test.h"

using namespace std;
using namespace libcamera;

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

		return TestPass;
	}
};

TEST_REGISTER(UtilsTest)
