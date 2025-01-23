/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Generic AWB algorithms
 */

#include "awb.h"

#include <libcamera/base/log.h>

/**
 * \file awb.h
 * \brief Base classes for AWB algorithms
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Awb)

namespace ipa {

/**
 * \class AwbResult
 * \brief The result of an awb calculation
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
 * Pipeline handlers using an AWB algorithm based on the AwbAlgorithm class need
 * to implement this class to give the algorithm access to the hardware-specific
 * statistics data.
 */

/**
 * \fn AwbStats::computeColourError
 * \brief Compute an error value for when the given gains would be applied
 * \param[in] gains The gains to apply
 *
 * Compute an error value (non-greyness) assuming the given \a gains would be
 * applied. To keep the actual implementations computationally inexpensive,
 * the squared colour error shall be returned.
 *
 * If the awb statistics provide multiple zones, the sum over all zones needs to
 * calculated.
 *
 * \return The computed error value
 */

/**
 * \fn AwbStats::getRGBMeans
 * \brief Get RGB means of the statistics
 *
 * Fetch the RGB means from the statistics. The values of each channel are
 * dimensionless and only the ratios are used for further calculations. This is
 * used by the simple gray world model to calculate the gains to apply.
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
 * \fn AwbAlgorithm::init
 * \brief Initialize the algorithm with the given tuning data
 * \param[in] tuningData The tuning data to use for the algorithm
 *
 * \return 0 on success, a negative error code otherwise
 */

/**
 * \fn AwbAlgorithm::calculateAwb
 * \brief Calculate awb data from the given statistics
 * \param[in] stats The statistics to use for the calculation
 * \param[in] lux The lux value of the scene
 *
 * Calculate an AwbResult object from the given statistics and lux value. A \a
 * lux value of 0 means it is unknown or invalid and the algorithm shall ignore
 * it.
 *
 * \return The awb result
 */

/**
 * \fn AwbAlgorithm::gainsFromColourTemperature
 * \brief Compute white balance gains from a colour temperature
 * \param[in] colourTemperature The colour temperature in Kelvin
 *
 * Compute the white balance gains from a \a colourTemperature. This function
 * does not take any statistics into account. It is used to compute the colour
 * gains when the user manually specifies a colour temperature.
 *
 * \return The colour gains
 */

/**
 * \fn AwbAlgorithm::controls
 * \brief Get the controls info map for this algorithm
 *
 * \return The controls info map
 */

/**
 * \fn AwbAlgorithm::handleControls
 * \param[in] controls The controls to handle
 * \brief Handle the controls supplied in a request
 */

/**
 * \var AwbAlgorithm::controls_
 * \brief Controls info map for the controls provided by the algorithm
 */

} /* namespace ipa */

} /* namespace libcamera */
