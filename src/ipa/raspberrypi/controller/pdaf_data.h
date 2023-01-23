/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * pdaf_data.h - PDAF Metadata; for now this is
 * largely based on IMX708's PDAF "Type 1" output.
 */
#pragma once

#include <stdint.h>

#define PDAF_DATA_ROWS 12
#define PDAF_DATA_COLS 16

struct PdafData {
	/* Confidence values, in raster order, in arbitrary units */
	uint16_t conf[PDAF_DATA_ROWS][PDAF_DATA_COLS];

	/* Phase error, in raster order, in s11 Q4 format (S.6.4) */
	int16_t phase[PDAF_DATA_ROWS][PDAF_DATA_COLS];
};
