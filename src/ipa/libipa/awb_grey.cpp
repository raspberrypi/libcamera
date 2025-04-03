/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Implementation of grey world AWB algorithm
 */

#include "awb_grey.h"

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>

#include "colours.h"

using namespace libcamera::controls;

/**
 * \file awb_grey.h
 * \brief Implementation of a grey world AWB algorithm
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(Awb)
namespace ipa {

/**
 * \class AwbGrey
 * \brief A Grey world auto white balance algorithm
 */

/**
 * \brief Initialize the algorithm with the given tuning data
 * \param[in] tuningData The tuning data for the algorithm
 *
 * Load the colour temperature curve from the tuning data. If there is no tuning
 * data available, continue with a warning. Manual colour temperature will not
 * work in that case.
 *
 * \return 0 on success, a negative error code otherwise
 */
int AwbGrey::init(const YamlObject &tuningData)
{
	Interpolator<Vector<double, 2>> gains;
	int ret = gains.readYaml(tuningData["colourGains"], "ct", "gains");
	if (ret < 0)
		LOG(Awb, Warning)
			<< "Failed to parse 'colourGains' "
			<< "parameter from tuning file; "
			<< "manual colour temperature will not work properly";
	else
		colourGainCurve_ = gains;

	return 0;
}

/**
 * \brief Calculate AWB data from the given statistics
 * \param[in] stats The statistics to use for the calculation
 * \param[in] lux The lux value of the scene
 *
 * The colour temperature is estimated based on the colours::estimateCCT()
 * function. The gains are calculated purely based on the RGB means provided by
 * the \a stats. The colour temperature is not taken into account when
 * calculating the gains.
 *
 * The \a lux parameter is not used in this algorithm.
 *
 * \return The AWB result
 */
AwbResult AwbGrey::calculateAwb(const AwbStats &stats, [[maybe_unused]] unsigned int lux)
{
	AwbResult result;
	auto means = stats.rgbMeans();
	result.colourTemperature = estimateCCT(means);

	/*
	 * Estimate the red and blue gains to apply in a grey world. The green
	 * gain is hardcoded to 1.0. Avoid divisions by zero by clamping the
	 * divisor to a minimum value of 1.0.
	 */
	result.gains.r() = means.g() / std::max(means.r(), 1.0);
	result.gains.g() = 1.0;
	result.gains.b() = means.g() / std::max(means.b(), 1.0);
	return result;
}

/**
 * \brief Compute white balance gains from a colour temperature
 * \param[in] colourTemperature The colour temperature in Kelvin
 *
 * Compute the white balance gains from a \a colourTemperature. This function
 * does not take any statistics into account. It simply interpolates the colour
 * gains configured in the colour temperature curve.
 *
 * \return The colour gains if a colour temperature curve is available,
 * [1, 1, 1] otherwise.
 */
std::optional<RGB<double>> AwbGrey::gainsFromColourTemperature(double colourTemperature)
{
	if (!colourGainCurve_) {
		LOG(Awb, Error) << "No gains defined";
		return std::nullopt;
	}

	auto gains = colourGainCurve_->getInterpolated(colourTemperature);
	return RGB<double>{ { gains[0], 1.0, gains[1] } };
}

} /* namespace ipa */

} /* namespace libcamera */
