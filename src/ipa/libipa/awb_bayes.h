/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Base class for bayes AWB algorithms
 */

#pragma once

#include <libcamera/controls.h>

#include "libcamera/internal/vector.h"
#include "libcamera/internal/yaml_parser.h"

#include "awb.h"
#include "interpolator.h"
#include "pwl.h"

namespace libcamera {

namespace ipa {

class AwbBayes : public AwbAlgorithm
{
public:
	AwbBayes() = default;

	int init(const YamlObject &tuningData) override;
	AwbResult calculateAwb(const AwbStats &stats, unsigned int lux) override;
	std::optional<RGB<double>> gainsFromColourTemperature(double temperatureK) override;
	void handleControls(const ControlList &controls) override;

private:
	int readPriors(const YamlObject &tuningData);

	void fineSearch(double &t, double &r, double &b, ipa::Pwl const &prior,
			const AwbStats &stats) const;
	double coarseSearch(const ipa::Pwl &prior, const AwbStats &stats) const;
	double interpolateQuadratic(ipa::Pwl::Point const &a,
				    ipa::Pwl::Point const &b,
				    ipa::Pwl::Point const &c) const;

	Interpolator<Pwl> priors_;
	Interpolator<Vector<double, 2>> colourGainCurve_;

	ipa::Pwl ctR_;
	ipa::Pwl ctB_;
	ipa::Pwl ctRInverse_;
	ipa::Pwl ctBInverse_;

	double transversePos_;
	double transverseNeg_;

	ModeConfig *currentMode_ = nullptr;
};

} /* namespace ipa */

} /* namespace libcamera */
