/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * hdr.hpp - HDR control algorithm
 */
#pragma once

#include "algorithm.h"

namespace RPiController {

struct HdrConfig {
	uint16_t thresholdLo;
	uint8_t diffPower;
	double motionThreshold;
};

class Hdr : public Algorithm
{
public:
	Hdr(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;

private:
	HdrConfig config_;
};

} // namespace RPiController
