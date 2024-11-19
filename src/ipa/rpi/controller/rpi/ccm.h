/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * CCM (colour correction matrix) control algorithm
 */
#pragma once

#include <vector>

#include "libcamera/internal/matrix.h"
#include <libipa/pwl.h>

#include "../ccm_algorithm.h"

namespace RPiController {

/* Algorithm to calculate colour matrix. Should be placed after AWB. */

struct CtCcm {
	double ct;
	libcamera::Matrix<double, 3, 3> ccm;
};

struct CcmConfig {
	std::vector<CtCcm> ccms;
	libcamera::ipa::Pwl saturation;
};

class Ccm : public CcmAlgorithm
{
public:
	Ccm(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void setSaturation(double saturation) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;

private:
	CcmConfig config_;
	double saturation_;
};

} /* namespace RPiController */
