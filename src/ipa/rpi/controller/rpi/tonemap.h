/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * tonemap.hpp - Tonemap control algorithm
 */
#pragma once

#include <libipa/pwl.h>

#include "algorithm.h"

namespace RPiController {

struct TonemapConfig {
	uint16_t detailConstant;
	double detailSlope;
	double iirStrength;
	double strength;
	libcamera::ipa::Pwl tonemap;
};

class Tonemap : public Algorithm
{
public:
	Tonemap(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;

private:
	TonemapConfig config_;
};

} // namespace RPiController
