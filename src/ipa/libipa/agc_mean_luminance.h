/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 agc_mean_luminance.h - Base class for mean luminance AGC algorithms
 */

#pragma once

#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include <libcamera/base/utils.h>

#include <libcamera/controls.h>

#include "libcamera/internal/yaml_parser.h"

#include "exposure_mode_helper.h"
#include "histogram.h"

namespace libcamera {

namespace ipa {

class AgcMeanLuminance
{
public:
	AgcMeanLuminance();
	virtual ~AgcMeanLuminance();

	struct AgcConstraint {
		enum class Bound {
			Lower = 0,
			Upper = 1
		};
		Bound bound;
		double qLo;
		double qHi;
		double yTarget;
	};

	int parseTuningData(const YamlObject &tuningData);

	void setLimits(utils::Duration minExposureTime, utils::Duration maxExposureTime,
		       double minGain, double maxGain);

	std::map<int32_t, std::vector<AgcConstraint>> constraintModes()
	{
		return constraintModes_;
	}

	std::map<int32_t, std::shared_ptr<ExposureModeHelper>> exposureModeHelpers()
	{
		return exposureModeHelpers_;
	}

	ControlInfoMap::Map controls()
	{
		return controls_;
	}

	std::tuple<utils::Duration, double, double>
	calculateNewEv(uint32_t constraintModeIndex, uint32_t exposureModeIndex,
		       const Histogram &yHist, utils::Duration effectiveExposureValue);

	void resetFrameCount()
	{
		frameCount_ = 0;
	}

private:
	virtual double estimateLuminance(const double gain) const = 0;

	void parseRelativeLuminanceTarget(const YamlObject &tuningData);
	void parseConstraint(const YamlObject &modeDict, int32_t id);
	int parseConstraintModes(const YamlObject &tuningData);
	int parseExposureModes(const YamlObject &tuningData);
	double estimateInitialGain() const;
	double constraintClampGain(uint32_t constraintModeIndex,
				   const Histogram &hist,
				   double gain);
	utils::Duration filterExposure(utils::Duration exposureValue);

	uint64_t frameCount_;
	utils::Duration filteredExposure_;
	double relativeLuminanceTarget_;

	std::map<int32_t, std::vector<AgcConstraint>> constraintModes_;
	std::map<int32_t, std::shared_ptr<ExposureModeHelper>> exposureModeHelpers_;
	ControlInfoMap::Map controls_;
};

} /* namespace ipa */

} /* namespace libcamera */
