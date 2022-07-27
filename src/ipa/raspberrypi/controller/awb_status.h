/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * awb_status.h - AWB control algorithm status
 */
#pragma once

/*
 * The AWB algorithm places its results into both the image and global metadata,
 * under the tag "awb.status".
 */

struct AwbStatus {
	char mode[32];
	double temperatureK;
	double gainR;
	double gainG;
	double gainB;
};
