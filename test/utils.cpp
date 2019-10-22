/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * utils.cpp - Miscellaneous utility tests
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "test.h"
#include "utils.h"

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

		/* utils::split() test. */
		std::vector<std::string> elements = {
			"/bin",
			"/usr/bin",
			"",
			"",
		};

		std::string path;
		for (const auto &element : elements)
			path += (path.empty() ? "" : ":") + element;

		std::vector<std::string> dirs;

		for (const auto &dir : utils::split(path, ":"))
			dirs.push_back(dir);

		if (dirs != elements) {
			cerr << "utils::split() test failed" << endl;
			return TestFail;
		}

		/* utils::dirname() tests. */
		if (TestPass != testDirname())
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(UtilsTest)
