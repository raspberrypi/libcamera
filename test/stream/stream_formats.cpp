/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream_formats.cpp - StreamFormats test
 */

#include <iostream>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class StreamFormatsTest : public Test
{
protected:
	int testSizes(std::string name, std::vector<Size> test, std::vector<Size> valid)
	{
		bool pass = false;

		for (Size &size : test) {
			pass = false;

			for (Size &validSize : valid) {
				if (size == validSize) {
					pass = true;
					break;
				}
			}

			if (!pass)
				break;
		}

		if (!pass) {
			cout << "Failed " << name << endl;
			cout << "Sizes to test:" << endl;
			for (Size &size : test)
				cout << size << endl;
			cout << "Valid sizes:" << endl;
			for (Size &size : valid)
				cout << size << endl;

			return TestFail;
		}

		return TestPass;
	}

	int run()
	{
		/* Test discrete sizes */
		StreamFormats discrete({
			{ PixelFormat(1), { SizeRange({ 100, 100 }), SizeRange({ 200, 200 }) } },
			{ PixelFormat(2), { SizeRange({ 300, 300 }), SizeRange({ 400, 400 }) } },
		});

		if (testSizes("discrete 1", discrete.sizes(PixelFormat(1)),
			      { Size(100, 100), Size(200, 200) }))
			return TestFail;
		if (testSizes("discrete 2", discrete.sizes(PixelFormat(2)),
			      { Size(300, 300), Size(400, 400) }))
			return TestFail;

		/* Test range sizes */
		StreamFormats range({
			{ PixelFormat(1), { SizeRange({ 640, 480 }, { 640, 480 }) } },
			{ PixelFormat(2), { SizeRange({ 640, 480 }, { 800, 600 }, 8, 8) } },
			{ PixelFormat(3), { SizeRange({ 640, 480 }, { 800, 600 }, 16, 16) } },
			{ PixelFormat(4), { SizeRange({ 128, 128 }, { 4096, 4096 }, 128, 128) } },
		});

		if (testSizes("range 1", range.sizes(PixelFormat(1)), { Size(640, 480) }))
			return TestFail;

		if (testSizes("range 2", range.sizes(PixelFormat(2)), {
			      Size(640, 480), Size(720, 480),
			      Size(720, 576), Size(768, 480),
			      Size(800, 600) }))
			return TestFail;

		if (testSizes("range 3", range.sizes(PixelFormat(3)), {
			      Size(640, 480), Size(720, 480),
			      Size(720, 576), Size(768, 480) }))
			return TestFail;

		if (testSizes("range 4", range.sizes(PixelFormat(4)), {
			      Size(1024, 768), Size(1280, 1024),
			      Size(2048, 1152), Size(2048, 1536),
			      Size(2560, 2048), Size(3200, 2048), }))
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(StreamFormatsTest)
