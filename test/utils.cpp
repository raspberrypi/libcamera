/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * utils.cpp - Miscellaneous utility tests
 */

#include <iostream>
#include <sstream>

#include "test.h"
#include "utils.h"

using namespace std;
using namespace libcamera;

class UtilsTest : public Test
{
protected:
	int run()
	{
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

		return TestPass;
	}
};

TEST_REGISTER(UtilsTest)
