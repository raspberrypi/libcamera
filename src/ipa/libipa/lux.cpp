/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class that implements lux estimation
 */
#include "lux.h"

#include <algorithm>
#include <chrono>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "histogram.h"

/**
 * \file lux.h
 * \brief Helper class that implements lux estimation
 *
 * Estimating the lux level of an image is a common operation that can for
 * instance be used to adjust the target Y value in AGC or for Bayesian AWB
 * estimation.
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(Lux)

namespace ipa {

/**
 * \class Lux
 * \brief Class that implements lux estimation
 *
 * IPAs that wish to use lux estimation should create a Lux algorithm module
 * that lightly wraps this module by providing the platform-specific luminance
 * histogram. The Lux entry in the tuning file must then precede the algorithms
 * that depend on the estimated lux value.
 */

/**
 * \var Lux::referenceExposureTime_
 * \brief The exposure time of the reference image, in microseconds
 */

/**
 * \var Lux::referenceAnalogueGain_
 * \brief The analogue gain of the reference image
 */

/**
 * \var Lux::referenceDigitalGain_
 * \brief The analogue gain of the reference image
 */

/**
 * \var Lux::referenceY_
 * \brief The measured luminance of the reference image, normalized to 1
 *
 */

/**
 * \var Lux::referenceLux_
 * \brief The estimated lux level of the reference image
 */

/**
  * \brief Construct the Lux helper module
  */
Lux::Lux()
{
}

/**
 * \brief Parse tuning data
 * \param[in] tuningData The YamlObject representing the tuning data
 *
 * This function parses yaml tuning data for the common Lux module. It requires
 * reference exposure time, analogue gain, digital gain, and lux values.
 *
 * \code{.unparsed}
 * algorithms:
 *   - Lux:
 *       referenceExposureTime: 10000
 *       referenceAnalogueGain: 4.0
 *       referenceDigitalGain: 1.0
 *       referenceY: 0.1831
 *       referenceLux: 1000
 * \endcode
 *
 * \return 0 on success or a negative error code
 */
int Lux::parseTuningData(const YamlObject &tuningData)
{
	auto value = tuningData["referenceExposureTime"].get<double>();
	if (!value) {
		LOG(Lux, Error) << "Missing tuning parameter: "
				<< "'referenceExposureTime'";
		return -EINVAL;
	}
	referenceExposureTime_ = *value * 1.0us;

	value = tuningData["referenceAnalogueGain"].get<double>();
	if (!value) {
		LOG(Lux, Error) << "Missing tuning parameter: "
				<< "'referenceAnalogueGain'";
		return -EINVAL;
	}
	referenceAnalogueGain_ = *value;

	value = tuningData["referenceDigitalGain"].get<double>();
	if (!value) {
		LOG(Lux, Error) << "Missing tuning parameter: "
				<< "'referenceDigitalGain'";
		return -EINVAL;
	}
	referenceDigitalGain_ = *value;

	value = tuningData["referenceY"].get<double>();
	if (!value) {
		LOG(Lux, Error) << "Missing tuning parameter: "
				<< "'referenceY'";
		return -EINVAL;
	}
	referenceY_ = *value;

	value = tuningData["referenceLux"].get<double>();
	if (!value) {
		LOG(Lux, Error) << "Missing tuning parameter: "
				<< "'referenceLux'";
		return -EINVAL;
	}
	referenceLux_ = *value;

	return 0;
}

/**
 * \brief Estimate lux given runtime values
 * \param[in] exposureTime Exposure time applied to the frame
 * \param[in] aGain Analogue gain applied to the frame
 * \param[in] dGain Digital gain applied to the frame
 * \param[in] yHist Histogram from the ISP statistics
 *
 * Estimate the lux given the exposure time, gain, and histogram.
 *
 * \return Estimated lux value
 */
double Lux::estimateLux(utils::Duration exposureTime,
			double aGain, double dGain,
			const Histogram &yHist) const
{
	double currentY = yHist.interQuantileMean(0, 1);
	double exposureTimeRatio = referenceExposureTime_ / exposureTime;
	double aGainRatio = referenceAnalogueGain_ / aGain;
	double dGainRatio = referenceDigitalGain_ / dGain;
	double yRatio = (currentY / yHist.bins()) / referenceY_;

	double estimatedLux = exposureTimeRatio * aGainRatio * dGainRatio *
			      yRatio * referenceLux_;

	LOG(Lux, Debug) << "Estimated lux " << estimatedLux;
	return estimatedLux;
}

} /* namespace ipa */

} /* namespace libcamera */
