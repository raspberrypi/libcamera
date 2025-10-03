#pragma once

#include "libipa/pwl.h"

struct DecompandStatus {
	uint32_t bitdepth;
	libcamera::ipa::Pwl decompandCurve;
};
