/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023-2026 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * DebayerParams header
 */

#pragma once

#include <array>
#include <stdint.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

namespace libcamera {

struct DebayerParams {
	Matrix<float, 3, 3> combinedMatrix = { { 1.0, 0.0, 0.0,
						 0.0, 1.0, 0.0,
						 0.0, 0.0, 1.0 } };
	RGB<double> blackLevel = RGB<double>({ 0.0, 0.0, 0.0 });
	float gamma = 1.0;
	float contrastExp = 1.0;
	RGB<double> gains = RGB<double>({ 1.0, 1.0, 1.0 });

	/**
	 * To prevent OpenGL alignment issues, the number of bytes in each row
	 * must be a multiple of 4 (the initial value of GL_UNPACK_ALIGNMENT)
	 * and lscLut is also aligned to 4 bytes.
	 **/
	static constexpr unsigned int kLscGridSize = 17;
	static constexpr unsigned int kLscValuesPerCell = 4;
	static constexpr unsigned int kLscBytesPerCell =
		kLscValuesPerCell * sizeof(uint8_t);
	using LscLookupTable =
		std::array<uint8_t, kLscGridSize * kLscGridSize * kLscValuesPerCell>;
	static constexpr auto identityLscLut = [] {
		LscLookupTable lut = {};
		/* lut.fill(64) could be used, but it fails with older gcc versions */
		for (size_t i = 0; kLscValuesPerCell * i < lut.size(); i++) {
			lut[i * kLscValuesPerCell + 0] = 64; /* == UQ<2, 6>(1.0f).quantized() */
			lut[i * kLscValuesPerCell + 1] = 64;
			lut[i * kLscValuesPerCell + 2] = 64;
		}
		return lut;
	}();
	alignas(4) LscLookupTable lscLut = identityLscLut;
	uint64_t lscLutVersion = 0;
};

} /* namespace libcamera */
