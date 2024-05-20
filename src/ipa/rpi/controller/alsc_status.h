/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ALSC (auto lens shading correction) control algorithm status
 */
#pragma once

#include <vector>

/*
 * The ALSC algorithm should post the following structure into the image's
 * "alsc.status" metadata.
 */

struct AlscStatus {
	std::vector<double> r;
	std::vector<double> g;
	std::vector<double> b;
	unsigned int rows;
	unsigned int cols;
};
