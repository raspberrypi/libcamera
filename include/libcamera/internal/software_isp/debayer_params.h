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

#include <stdint.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

namespace libcamera {

struct DebayerParams {
	Matrix<float, 3, 3> combinedMatrix;
	RGB<float> blackLevel;
	float gamma;
	float contrastExp;
	RGB<float> gains;
};

} /* namespace libcamera */
