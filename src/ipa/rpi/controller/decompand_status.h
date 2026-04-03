/* SPDX-License-Identifier: BSD-2-Clause */
#pragma once

#include "libipa/pwl.h"

struct DecompandStatus {
	uint32_t bitdepth;
	libcamera::ipa::Pwl decompandCurve;
};
