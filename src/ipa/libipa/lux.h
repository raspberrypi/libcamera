/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class that implements lux estimation
 */

#pragma once

#include <libcamera/base/utils.h>

namespace libcamera {

class ValueNode;

namespace ipa {

class Histogram;

class Lux
{
public:
	Lux();

	int parseTuningData(const ValueNode &tuningData);
	double estimateLux(utils::Duration exposureTime,
			   double aGain, double dGain,
			   const Histogram &yHist) const;

private:
	utils::Duration referenceExposureTime_;
	double referenceAnalogueGain_;
	double referenceDigitalGain_;
	double referenceY_;
	double referenceLux_;
};

} /* namespace ipa */

} /* namespace libcamera */
