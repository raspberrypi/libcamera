/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Generic AWB algorithms
 */

#include "awb.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

/**
 * \file awb.h
 * \brief Base classes for AWB algorithms
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Awb)

namespace ipa {

/**
 * \class AwbResult
 * \brief The result of an AWB calculation
 *
 * This class holds the result of an auto white balance calculation.
 */

/**
 * \var AwbResult::gains
 * \brief The calculated white balance gains
 */

/**
 * \var AwbResult::colourTemperature
 * \brief The calculated colour temperature in Kelvin
 */

/**
 * \class AwbStats
 * \brief An abstraction class wrapping hardware-specific AWB statistics
 *
 * IPA modules using an AWB algorithm based on the AwbAlgorithm class need to
 * implement this class to give the algorithm access to the hardware-specific
 * statistics data.
 */

/**
 * \fn AwbStats::computeColourError()
 * \brief Compute an error value for when the given gains would be applied
 * \param[in] gains The gains to apply
 *
 * Compute an error value (non-greyness) assuming the given \a gains would be
 * applied. To keep the actual implementations computationally inexpensive,
 * the squared colour error shall be returned.
 *
 * If the AWB statistics provide multiple zones, the average of the individual
 * squared errors shall be returned. Averaging/normalizing is necessary so that
 * the numeric dimensions are the same on all hardware platforms.
 *
 * \return The computed error value
 */

/**
 * \fn AwbStats::rgbMeans()
 * \brief Get RGB means of the statistics
 *
 * Fetch the RGB means from the statistics. The values of each channel are
 * dimensionless and only the ratios are used for further calculations. This is
 * used by the simple grey world model to calculate the gains to apply.
 *
 * \return The RGB means
 */

/**
 * \class AwbAlgorithm
 * \brief A base class for auto white balance algorithms
 *
 * This class is a base class for auto white balance algorithms. It defines an
 * interface for the algorithms to implement, and is used by the IPAs to
 * interact with the concrete implementation.
 */

/**
 * \fn AwbAlgorithm::init()
 * \brief Initialize the algorithm with the given tuning data
 * \param[in] tuningData The tuning data to use for the algorithm
 *
 * \return 0 on success, a negative error code otherwise
 */

/**
 * \fn AwbAlgorithm::calculateAwb()
 * \brief Calculate AWB data from the given statistics
 * \param[in] stats The statistics to use for the calculation
 * \param[in] lux The lux value of the scene
 *
 * Calculate an AwbResult object from the given statistics and lux value. A \a
 * lux value of 0 means it is unknown or invalid and the algorithm shall ignore
 * it.
 *
 * \return The AWB result
 */

/**
 * \fn AwbAlgorithm::gainsFromColourTemperature()
 * \brief Compute white balance gains from a colour temperature
 * \param[in] colourTemperature The colour temperature in Kelvin
 *
 * Compute the white balance gains from a \a colourTemperature. This function
 * does not take any statistics into account. It is used to compute the colour
 * gains when the user manually specifies a colour temperature.
 *
 * \return The colour gains or std::nullopt if the conversion is not possible
 */

/**
 * \fn AwbAlgorithm::controls()
 * \brief Get the controls info map for this algorithm
 *
 * \return The controls info map
 */

/**
 * \fn AwbAlgorithm::handleControls()
 * \param[in] controls The controls to handle
 * \brief Handle the controls supplied in a request
 */

/**
 * \brief Parse the mode configurations from the tuning data
 * \param[in] tuningData the YamlObject representing the tuning data
 * \param[in] def The default value for the AwbMode control
 *
 * Utility function to parse the tuning data for an AwbMode entry and read all
 * provided modes. It adds controls::AwbMode to AwbAlgorithm::controls_ and
 * populates AwbAlgorithm::modes_. For a list of possible modes see \ref
 * controls::AwbModeEnum.
 *
 * Each mode entry must contain a "lo" and "hi" key to specify the lower and
 * upper colour temperature of that mode. For example:
 *
 * \code{.unparsed}
 * algorithms:
 *   - Awb:
 *     AwbMode:
 *       AwbAuto:
 *         lo: 2500
 *         hi: 8000
 *       AwbIncandescent:
 *         lo: 2500
 *         hi: 3000
 *       ...
 * \endcode
 *
 * If \a def is supplied but not contained in the the \a tuningData, -EINVAL is
 * returned.
 *
 * \sa controls::AwbModeEnum
 * \return Zero on success, negative error code otherwise
 */
int AwbAlgorithm::parseModeConfigs(const YamlObject &tuningData,
				   const ControlValue &def)
{
	std::vector<ControlValue> availableModes;

	const YamlObject &yamlModes = tuningData[controls::AwbMode.name()];
	if (!yamlModes.isDictionary()) {
		LOG(Awb, Error)
			<< "AwbModes must be a dictionary.";
		return -EINVAL;
	}

	for (const auto &[modeName, modeDict] : yamlModes.asDict()) {
		if (controls::AwbModeNameValueMap.find(modeName) ==
		    controls::AwbModeNameValueMap.end()) {
			LOG(Awb, Warning)
				<< "Skipping unknown AWB mode '"
				<< modeName << "'";
			continue;
		}

		if (!modeDict.isDictionary()) {
			LOG(Awb, Error)
				<< "Invalid AWB mode '" << modeName << "'";
			return -EINVAL;
		}

		const auto &modeValue = static_cast<controls::AwbModeEnum>(
			controls::AwbModeNameValueMap.at(modeName));

		ModeConfig &config = modes_[modeValue];

		auto hi = modeDict["hi"].get<double>();
		if (!hi) {
			LOG(Awb, Error) << "Failed to read hi param of mode "
					<< modeName;
			return -EINVAL;
		}
		config.ctHi = *hi;

		auto lo = modeDict["lo"].get<double>();
		if (!lo) {
			LOG(Awb, Error) << "Failed to read low param of mode "
					<< modeName;
			return -EINVAL;
		}
		config.ctLo = *lo;

		availableModes.push_back(modeValue);
	}

	if (modes_.empty()) {
		LOG(Awb, Error) << "No AWB modes configured";
		return -EINVAL;
	}

	if (!def.isNone() &&
	    modes_.find(def.get<controls::AwbModeEnum>()) == modes_.end()) {
		const auto &names = controls::AwbMode.enumerators();
		LOG(Awb, Error) << names.at(def.get<controls::AwbModeEnum>())
				<< " mode is missing in the configuration.";
		return -EINVAL;
	}

	controls_[&controls::AwbMode] = ControlInfo(availableModes, def);

	return 0;
}

/**
 * \class AwbAlgorithm::ModeConfig
 * \brief Holds the configuration of a single AWB mode
 *
 * AWB modes limit the regulation of the AWB algorithm to a specific range of
 * colour temperatures.
 */

/**
 * \var AwbAlgorithm::ModeConfig::ctLo
 * \brief The lowest valid colour temperature of that mode
 */

/**
 * \var AwbAlgorithm::ModeConfig::ctHi
 * \brief The highest valid colour temperature of that mode
 */

/**
 * \var AwbAlgorithm::controls_
 * \brief Controls info map for the controls provided by the algorithm
 */

/**
 * \var AwbAlgorithm::modes_
 * \brief Map of all configured modes
 * \sa AwbAlgorithm::parseModeConfigs
 */

} /* namespace ipa */

} /* namespace libcamera */
