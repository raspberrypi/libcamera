/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy.
 *
 * stream_colorspace.cpp - Stream colorspace adjustment test
 */

#include <iostream>

#include <libcamera/camera.h>
#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "test.h"

using namespace libcamera;
using namespace std;

class TestCameraConfiguration : public CameraConfiguration
{
public:
	TestCameraConfiguration()
		: CameraConfiguration()
	{
	}

	Status validate() override
	{
		return validateColorSpaces();
	}
};

class StreamColorSpaceTest : public Test
{
protected:
	int run()
	{
		TestCameraConfiguration config;

		StreamConfiguration cfg;
		cfg.size = { 640, 320 };
		cfg.pixelFormat = formats::YUV422;
		cfg.colorSpace = ColorSpace::Srgb;
		config.addConfiguration(cfg);

		StreamConfiguration &streamCfg = config.at(0);

		/*
		 * YUV pixelformat with sRGB colorspace should have Y'CbCr encoding
		 * adjusted.
		 */
		config.validate();
		if (streamCfg.colorSpace->ycbcrEncoding == ColorSpace::YcbcrEncoding::None) {
			cerr << "YUV format must have YCbCr encoding" << endl;
			return TestFail;
		}

		/*
		 * For YUV pixelFormat, encoding should be picked up according
		 * to primaries and transfer function, if 'None' is specified.
		 */
		streamCfg.pixelFormat = formats::YUV422;
		streamCfg.colorSpace = ColorSpace(ColorSpace::Primaries::Rec2020,
						  ColorSpace::TransferFunction::Rec709,
						  ColorSpace::YcbcrEncoding::None,
						  ColorSpace::Range::Limited);
		config.validate();
		if (streamCfg.colorSpace->ycbcrEncoding != ColorSpace::YcbcrEncoding::Rec2020) {
			cerr << "Failed to adjust colorspace Y'CbCr encoding according"
			     << " to primaries and transfer function" << endl;
			return TestFail;
		}

		/* For RGB pixelFormat, Sycc colorspace should get adjusted to sRGB. */
		streamCfg.pixelFormat = formats::RGB888;
		streamCfg.colorSpace = ColorSpace::Sycc;
		config.validate();
		if (streamCfg.colorSpace != ColorSpace::Srgb) {
			cerr << "RGB format's colorspace should be set to Srgb" << endl;
			return TestFail;
		}

		/* Raw formats should always set colorspace to ColorSpace::Raw. */
		streamCfg.pixelFormat = formats::SBGGR8;
		streamCfg.colorSpace = ColorSpace::Rec709;
		config.validate();
		if (streamCfg.colorSpace != ColorSpace::Raw) {
			cerr << "Raw format must always have Raw colorspace" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(StreamColorSpaceTest)
