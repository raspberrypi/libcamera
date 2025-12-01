/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Red Hat, Inc.
 *
 * PPM writer
 */

#include "ppm_writer.h"

#include <errno.h>
#include <fstream>
#include <iostream>
#include <vector>

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

using namespace libcamera;

int PPMWriter::write(const char *filename,
		     const StreamConfiguration &config,
		     const Span<uint8_t> &data)
{
	struct FormatTransformation {
		unsigned int rPos;
		unsigned int gPos;
		unsigned int bPos;
		unsigned int bytesPerPixel;
	};

	static const std::map<libcamera::PixelFormat, FormatTransformation> transforms = {
		{ libcamera::formats::R8, { 0, 0, 0, 1 } },
		{ libcamera::formats::RGB888, { 2, 1, 0, 3 } },
		{ libcamera::formats::BGR888, { 0, 1, 2, 3 } },
		{ libcamera::formats::ARGB8888, { 2, 1, 0, 4 } },
		{ libcamera::formats::XRGB8888, { 2, 1, 0, 4 } },
		{ libcamera::formats::ABGR8888, { 0, 1, 2, 4 } },
		{ libcamera::formats::XBGR8888, { 0, 1, 2, 4 } },
		{ libcamera::formats::RGBA8888, { 3, 2, 1, 4 } },
		{ libcamera::formats::RGBX8888, { 3, 2, 1, 4 } },
		{ libcamera::formats::BGRA8888, { 1, 2, 3, 4 } },
		{ libcamera::formats::BGRX8888, { 1, 2, 3, 4 } },
	};

	FormatTransformation transformation;
	if (auto search = transforms.find(config.pixelFormat); search != transforms.end()) {
		transformation = search->second;
	} else {
		std::cerr
			<< "Only RGB output pixel formats are supported ("
			<< config.pixelFormat << " requested)" << std::endl;
		return -EINVAL;
	}

	std::ofstream output(filename, std::ios::binary);
	if (!output) {
		std::cerr << "Failed to open ppm file: " << filename << std::endl;
		return -EIO;
	}

	output << "P6" << std::endl
	       << config.size.width << " " << config.size.height << std::endl
	       << "255" << std::endl;
	if (!output) {
		std::cerr << "Failed to write the file header" << std::endl;
		return -EIO;
	}

	const unsigned int rowLength = config.size.width * 3;
	const char *row = reinterpret_cast<const char *>(data.data());
	const bool transform = config.pixelFormat != formats::BGR888;
	std::vector<char> transformedRow(transform ? rowLength : 0);

	for (unsigned int y = 0; y < config.size.height; y++, row += config.stride) {
		if (transform) {
			for (unsigned int x = 0; x < config.size.width; x++) {
				transformedRow[x * 3] =
					row[x * transformation.bytesPerPixel + transformation.rPos];
				transformedRow[x * 3 + 1] =
					row[x * transformation.bytesPerPixel + transformation.gPos];
				transformedRow[x * 3 + 2] =
					row[x * transformation.bytesPerPixel + transformation.bPos];
			}
		}

		output.write(transform ? transformedRow.data() : row, rowLength);
		if (!output) {
			std::cerr << "Failed to write image data at row " << y << std::endl;
			return -EIO;
		}
	}

	return 0;
}
