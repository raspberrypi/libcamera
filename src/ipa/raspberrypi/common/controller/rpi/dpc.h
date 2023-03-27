/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * dpc.h - DPC (defective pixel correction) control algorithm
 */
#pragma once

#include "../algorithm.h"
#include "../dpc_status.h"

namespace RPiController {

/* Back End algorithm to apply appropriate GEQ settings. */

struct DpcConfig {
	int strength;
};

class Dpc : public Algorithm
{
public:
	Dpc(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	DpcConfig config_;
};

} /* namespace RPiController */
