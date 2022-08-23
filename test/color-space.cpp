/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * libcamera ColorSpace test
 */

#include <array>
#include <iostream>

#include <libcamera/color_space.h>

#include "test.h"

using namespace libcamera;
using namespace std;

class ColorSpaceTest : public Test
{
protected:
	int run()
	{
		if (ColorSpace::toString(std::nullopt) != "Unset") {
			std::cerr << "Conversion from nullopt to string failed" << std::endl;
			return TestFail;
		}

		const std::array<std::pair<ColorSpace, std::string>, 10> colorSpaces = { {
			{ ColorSpace::Raw, "RAW" },
			{ ColorSpace::Srgb, "sRGB" },
			{ ColorSpace::Sycc, "sYCC" },
			{ ColorSpace::Smpte170m, "SMPTE170M" },
			{ ColorSpace::Rec709, "Rec709" },
			{ ColorSpace::Rec2020, "Rec2020" },
			{
				ColorSpace{
					ColorSpace::Primaries::Raw,
					ColorSpace::TransferFunction::Linear,
					ColorSpace::YcbcrEncoding::None,
					ColorSpace::Range::Limited
				},
				"RAW/Linear/None/Limited"
			}, {
				ColorSpace{
					ColorSpace::Primaries::Smpte170m,
					ColorSpace::TransferFunction::Srgb,
					ColorSpace::YcbcrEncoding::Rec601,
					ColorSpace::Range::Full
				},
				"SMPTE170M/sRGB/Rec601/Full"
			}, {
				ColorSpace{
					ColorSpace::Primaries::Rec709,
					ColorSpace::TransferFunction::Rec709,
					ColorSpace::YcbcrEncoding::Rec709,
					ColorSpace::Range::Full
				},
				"Rec709/Rec709/Rec709/Full"
			}, {
				ColorSpace{
					ColorSpace::Primaries::Rec2020,
					ColorSpace::TransferFunction::Linear,
					ColorSpace::YcbcrEncoding::Rec2020,
					ColorSpace::Range::Limited
				},
				"Rec2020/Linear/Rec2020/Limited"
			},
		} };

		for (const auto &[colorSpace, name] : colorSpaces) {
			if (colorSpace.toString() != name) {
				std::cerr
					<< "Conversion from ColorSpace to string failed: "
					<< "expected " << name
					<< ", got " << colorSpace.toString()
					<< std::endl;
				return TestFail;
			}

			if (ColorSpace::fromString(name) != colorSpace) {
				std::cerr
					<< "Conversion from string "
					<< name << " to ColorSpace failed"
					<< std::endl;
				return TestFail;
			}
		}

		if (ColorSpace::fromString("Invalid")) {
			std::cerr << "Conversion from invalid name string to color space succeeded"
				  << std::endl;
			return TestFail;
		}

		if (ColorSpace::fromString("Rec709/Rec709/Rec710/Limited")) {
			std::cerr << "Conversion from invalid component string to color space succeeded"
				  << std::endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ColorSpaceTest)
