/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * black level control algorithm
 */
#pragma once

#include "../black_level_algorithm.h"
#include "../black_level_status.h"

/* This is our implementation of the "black level algorithm". */

namespace RPiController {

class BlackLevel : public BlackLevelAlgorithm
{
public:
	BlackLevel(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialValues(uint16_t &blackLevelR, uint16_t &blackLevelG,
			   uint16_t &blackLevelB) override;
	void prepare(Metadata *imageMetadata) override;

private:
	double blackLevelR_;
	double blackLevelG_;
	double blackLevelB_;
};

} /* namespace RPiController */
