/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class that performs computations relating to exposure
 */

#pragma once

#include <tuple>
#include <utility>
#include <vector>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

namespace libcamera {

namespace ipa {

class ExposureModeHelper
{
public:
	ExposureModeHelper(const Span<std::pair<utils::Duration, double>> stages);
	~ExposureModeHelper() = default;

	void setLimits(utils::Duration minExposureTime, utils::Duration maxExposureTime,
		       double minGain, double maxGain);

	std::tuple<utils::Duration, double, double>
	splitExposure(utils::Duration exposure) const;

	utils::Duration minExposureTime() const { return minExposureTime_; }
	utils::Duration maxExposureTime() const { return maxExposureTime_; }
	double minGain() const { return minGain_; }
	double maxGain() const { return maxGain_; }

private:
	utils::Duration clampExposureTime(utils::Duration exposureTime) const;
	double clampGain(double gain) const;

	std::vector<utils::Duration> exposureTimes_;
	std::vector<double> gains_;

	utils::Duration minExposureTime_;
	utils::Duration maxExposureTime_;
	double minGain_;
	double maxGain_;
};

} /* namespace ipa */

} /* namespace libcamera */
