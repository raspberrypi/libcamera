/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class that performs computations relating to exposure
 */

#pragma once

#include <span>
#include <utility>
#include <vector>

#include <libcamera/base/utils.h>

#include "camera_sensor_helper.h"

namespace libcamera {

namespace ipa {

class ExposureModeHelper
{
public:
	ExposureModeHelper(std::span<std::pair<utils::Duration, double>> stages);
	~ExposureModeHelper() = default;

	void configure(utils::Duration lineLength, const CameraSensorHelper *sensorHelper);
	void setLimits(utils::Duration minExposureTime, utils::Duration maxExposureTime,
		       double minGain, double maxGain);

	struct Result {
		utils::Duration exposureTime;
		double analogueGain;
		double quantizationGain;
		double digitalGain;
	};

	[[nodiscard]] Result splitExposure(utils::Duration exposure) const;

	utils::Duration minExposureTime() const { return minExposureTime_; }
	utils::Duration maxExposureTime() const { return maxExposureTime_; }
	double minGain() const { return minGain_; }
	double maxGain() const { return maxGain_; }

private:
	utils::Duration clampExposureTime(utils::Duration exposureTime,
					  double *quantizationGain = nullptr) const;
	double clampGain(double gain, double *quantizationGain = nullptr) const;

	std::vector<utils::Duration> exposureTimes_;
	std::vector<double> gains_;

	utils::Duration lineDuration_;
	utils::Duration minExposureTime_;
	utils::Duration maxExposureTime_;
	double minGain_;
	double maxGain_;
	const CameraSensorHelper *sensorHelper_;
};

} /* namespace ipa */

} /* namespace libcamera */
