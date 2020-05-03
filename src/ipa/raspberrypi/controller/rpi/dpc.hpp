/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * dpc.hpp - DPC (defective pixel correction) control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../dpc_status.h"

namespace RPi {

// Back End algorithm to apply appropriate GEQ settings.

struct DpcConfig {
	int strength;
};

class Dpc : public Algorithm
{
public:
	Dpc(Controller *controller);
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Prepare(Metadata *image_metadata) override;

private:
	DpcConfig config_;
};

} // namespace RPi
