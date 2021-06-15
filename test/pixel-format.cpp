/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Kaaira Gupta
 * libcamera pixel format handling test
 */

#include <iostream>
#include <vector>

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include <libcamera/base/utils.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class PixelFormatTest : public Test
{
protected:
	int run()
	{
		std::vector<std::pair<PixelFormat, const char *>> formatsMap{
			{ formats::R8, "R8" },
			{ formats::SRGGB10_CSI2P, "SRGGB10_CSI2P" },
			{ PixelFormat(0, 0), "<INVALID>" },
			{ PixelFormat(0x20203843), "<C8  >" }
		};

		for (const auto &format : formatsMap) {
			if ((format.first).toString() != format.second) {
				cerr << "Failed to convert PixelFormat "
				     << utils::hex(format.first.fourcc()) << " to string"
				     << endl;
				return TestFail;
			}
		}

		if (PixelFormat().toString() != "<INVALID>") {
			cerr << "Failed to convert default PixelFormat to string"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(PixelFormatTest)
