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
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;
	void setStrength(std::vector<double> &inputArray, std::vector<double> &outputArray,
			 double strengthFactor);

private:
	CacConfig config_;
	CacStatus cacStatus_;
	void arrayToSet(const libcamera::YamlObject &params, std::vector<double> &inputArray);
};

} // namespace RPiController
