/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * AWB grey world algorithm
 */

#pragma once

#include <optional>

#include "libcamera/internal/vector.h"

#include "awb.h"
#include "interpolator.h"

namespace libcamera {

namespace ipa {

class AwbGrey : public AwbAlgorithm
{
public:
	AwbGrey() = default;

	int init(const YamlObject &tuningData) override;
	AwbResult calculateAwb(const AwbStats &stats, unsigned int lux) override;
	std::optional<RGB<double>> gainsFromColourTemperature(double colourTemperature) override;

private:
	std::optional<Interpolator<Vector<double, 2>>> colourGainCurve_;
};

} /* namespace ipa */

} /* namespace libcamera */
