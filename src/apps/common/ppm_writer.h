/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat, Inc.
 *
 * PPM writer
 */

#pragma once

#include <span>

#include <libcamera/stream.h>

class PPMWriter
{
public:
	static int write(const char *filename,
			 const libcamera::StreamConfiguration &config,
			 std::span<uint8_t> data);
};
