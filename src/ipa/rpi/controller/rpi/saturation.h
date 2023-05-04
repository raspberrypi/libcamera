/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * saturation.hpp - Saturation control algorithm
 */
#pragma once

#include "algorithm.h"

namespace RPiController {

struct SaturationConfig {
	uint8_t shiftR;
	uint8_t shiftG;
	uint8_t shiftB;
};

class Saturation : public Algorithm
{
public:
	Saturation(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;

private:
	SaturationConfig config_;
};

} // namespace RPiController
