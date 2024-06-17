/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * PDAF Metadata
 */
#pragma once

#include <stdint.h>

#include "region_stats.h"

namespace RPiController {

struct PdafData {
	/* Confidence, in arbitrary units */
	uint16_t conf;
	/* Phase error, in s16 Q4 format (S.11.4) */
	int16_t phase;
};

using PdafRegions = RegionStats<PdafData>;

} /* namespace RPiController */
