// SPDX-License-Identifier: GPL-2.0-only
/*
 * PY PiSP Front End statistics definitions
 *
 * Copyright (C) 2021 - Raspberry Pi (Trading) Ltd.
 *
 */
#ifndef _PISP_FE_STATISTICS_H_
#define _PISP_FE_STATISTICS_H_

#define PISP_FLOATING_STATS_NUM_ZONES 4
#define PISP_AGC_STATS_NUM_BINS 1024
#define PISP_AGC_STATS_SIZE 16
#define PISP_AGC_STATS_NUM_ZONES (PISP_AGC_STATS_SIZE * PISP_AGC_STATS_SIZE)
#define PISP_AGC_STATS_NUM_ROW_SUMS 512

struct pisp_agc_statistics_zone {
	uint64_t Y_sum;
	uint32_t counted;
	uint32_t pad;
};

struct pisp_agc_statistics {
	uint32_t row_sums[PISP_AGC_STATS_NUM_ROW_SUMS];
	/*
	 * 32-bits per bin means an image (just less than) 16384x16384 pixels
	 * in size can weight every pixel from 0 to 15.
	 */
	uint32_t histogram[PISP_AGC_STATS_NUM_BINS];
	struct pisp_agc_statistics_zone floating[PISP_FLOATING_STATS_NUM_ZONES];
};

#define PISP_AWB_STATS_SIZE 32
#define PISP_AWB_STATS_NUM_ZONES (PISP_AWB_STATS_SIZE * PISP_AWB_STATS_SIZE)

struct pisp_awb_statistics_zone {
	uint32_t R_sum;
	uint32_t G_sum;
	uint32_t B_sum;
	uint32_t counted;
};

struct pisp_awb_statistics {
	struct pisp_awb_statistics_zone zones[PISP_AWB_STATS_NUM_ZONES];
	struct pisp_awb_statistics_zone floating[PISP_FLOATING_STATS_NUM_ZONES];
};

#define PISP_CDAF_STATS_SIZE 8
#define PISP_CDAF_STATS_NUM_FOMS (PISP_CDAF_STATS_SIZE * PISP_CDAF_STATS_SIZE)

struct pisp_cdaf_statistics {
	uint64_t foms[PISP_CDAF_STATS_NUM_FOMS];
	uint64_t floating[PISP_FLOATING_STATS_NUM_ZONES];
};

struct pisp_statistics {
	struct pisp_awb_statistics awb;
	struct pisp_agc_statistics agc;
	struct pisp_cdaf_statistics cdaf;
};

#endif /* _PISP_FE_STATISTICS_H_ */
