/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * geq.h - GEQ (green equalisation) control algorithm
 */
#pragma once

#include "../algorithm.h"
#include "../geq_status.h"

namespace RPiController {

/* Back End algorithm to apply appropriate GEQ settings. */

struct GeqConfig {
	uint16_t offset;
	double slope;
	Pwl strength; /* lux to strength factor */
};

class Geq : public Algorithm
{
public:
	Geq(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	GeqConfig config_;
};

} /* namespace RPiController */
