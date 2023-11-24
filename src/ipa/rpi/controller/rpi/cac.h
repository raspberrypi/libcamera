/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * cac.hpp - CAC control algorithm
 */
#pragma once

#include "algorithm.h"
#include "cac_status.h"

namespace RPiController {

struct CacConfig {
	bool enabled;
	std::vector<double> lutRx;
	std::vector<double> lutRy;
	std::vector<double> lutBx;
	std::vector<double> lutBy;
};

class Cac : public Algorithm
{
public:
	Cac(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	CacConfig config_;
	CacStatus cacStatus_;
};

} // namespace RPiController
