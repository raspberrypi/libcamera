/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * CAC (Chromatic Abberation Correction) algorithm status
 */
#pragma once

struct CacStatus {
	std::vector<double> lutRx;
	std::vector<double> lutRy;
	std::vector<double> lutBx;
	std::vector<double> lutBy;
};
