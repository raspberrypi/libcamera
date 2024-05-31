/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, 2024 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * DebayerParams header
 */

#pragma once

#include <array>
#include <stdint.h>

namespace libcamera {

struct DebayerParams {
	static constexpr unsigned int kRGBLookupSize = 256;

	using ColorLookupTable = std::array<uint8_t, kRGBLookupSize>;

	ColorLookupTable red;
	ColorLookupTable green;
	ColorLookupTable blue;
};

} /* namespace libcamera */
