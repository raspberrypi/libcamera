/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * alsc_status.h - ALSC (auto lens shading correction) control algorithm status
 */
#pragma once

/*
 * The ALSC algorithm should post the following structure into the image's
 * "alsc.status" metadata.
 */

constexpr unsigned int AlscCellsX = 16;
constexpr unsigned int AlscCellsY = 12;

struct AlscStatus {
	double r[AlscCellsY][AlscCellsX];
	double g[AlscCellsY][AlscCellsX];
	double b[AlscCellsY][AlscCellsX];
};
