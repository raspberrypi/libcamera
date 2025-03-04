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

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

using namespace libcamera;

int PPMWriter::write(const char *filename,
		     const StreamConfiguration &config,
		     const Span<uint8_t> &data)
{
	if (config.pixelFormat != formats::BGR888) {
		std::cerr << "Only BGR888 output pixel format is supported ("
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
	for (unsigned int y = 0; y < config.size.height; y++, row += config.stride) {
		output.write(row, rowLength);
		if (!output) {
			std::cerr << "Failed to write image data at row " << y << std::endl;
			return -EIO;
		}
	}

	return 0;
}
