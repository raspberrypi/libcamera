/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy
 *
 * Base classes and types for LSC algorithm implementations
 */

#include "lsc_base.h"

/**
 * \file lsc_base.h
 * \brief Base types and definitions for LscImplementation class hierarchy
 */

namespace libcamera {

namespace ipa {

/**
 * \struct LscDescriptor
 * \brief Describe the ISP LSC engine
 *
 * \var LscDescriptor::keys
 * \brief The list of colour components to which a list of gains is associated
 * with in the tuning file. Used for parsing the tuning file
 *
 * \var LscDescriptor::numHSamples
 * \brief Number of horizontal gain samples of the ISP LSC grid. Used for
 * validating the list of gains parsed from tuning file
 *
 * \var LscDescriptor::numVSamples
 * \brief Number of vertical gain samples of the ISP LSC grid. Used for
 * validating the list of gains parsed from tuning file
 *
 * \var LscDescriptor::sensorSize
 * \brief The physical sensor size. This is the largest frame size used to
 * generate the LSC table. Only used by the polynomial LSC algorithm
 *
 * \todo: Most likely the reference frame should be native_size.
 * Let's wait how the internal discussions progress.
 */

/**
 * \class LscImplementation
 * \brief Pure virtual base class for LSC algorithm implementations
 *
 * Defines the interface for the LSC algorithm implementation.
 */

/**
 * \typedef LscImplementation::Components
 * \brief Associate colour components with a list of gains in float format
 *
 * LSC tables are defined as a list of gain values associated to a colour
 * component.
 *
 * As different ISP support different colour components (usually 'r', 'gr',
 * 'gb', 'b' or just 'r', 'g', 'b') this class associates a string
 * identifier for the colour component to a list of gains.
 *
 * The gain values are represented as floats and are either loaded from tuning
 * file or are the result of a polynomial expansion, depending on the
 * LscImplementation derived class in use (LscTable or LscPolynomial).
 *
 * The gain values are converted to the platform's register format before being
 * returned to the IPA module by LscAlgorithm::configure().
 *
 * \todo Tabular LSC data are already expressed in register format. Make the
 * tuning files express gains in float format as well and actually perform
 * quantization in LscAlgorithm::configure().
 */

/**
 * \typedef LscImplementation::ComponentsMap
 * \brief Associate a colour temperature to a LSC table
 *
 * An LSC table is generated during the tuning phase for a specific colour
 * temperature, and a tuning file usually contains LSC tables generated for
 * several different colour temperatures.
 */

/**
 * \fn LscImplementation::~LscImplementation
 * \brief Virtual class destructor
 */

/**
 * \fn LscImplementation::parseLscData
 * \brief Parse \a tuningData
 * \param[in] tuningData The tuning data
 * \param[in] descriptor The LSC engine descriptor
 *
 * Parse the tuning file using the \a descriptor to identify the colour
 * components in the tuning data and validate the size of the loaded gains
 * tables.
 *
 * \return 0 on success, a negative error number otherwise
 */

/**
 * \fn LscImplementation::sampleForCrop
 * \brief Re-sample the LSC components for \a cropRectangle
 * \param[in] cropRectangle The sensor analogue crop rectangle
 * \param[in] xPos List of horizontal positions of the LSC grid nodes
 * \param[in] yPos List of vertical positions of the LSC grid nodes
 *
 * LSC tables have to be re-sampled every time a new sensor configuration is
 * used, as each streaming session might use a different sensor crop rectangle
 * \a cropRectangle.
 *
 * \a cropRectangle represents the size of the frame on which the LSC tables
 * have to be re-sampled on.
 *
 * \a xPos and \a yPos represent the position of the grid nodes vertexes in
 * the [0, 1] interval. For example an equally spaced grid of 16 nodes will have
 * each segment of size 0.0625 and the list of nodes position will be
 * [0, 0.0625, 0.125, 0.1875, ... , 1]. It is expected that the first position
 * is 0 and the last position is 1.
 *
 * LSC tables are expressed in two formats:
 * - A list of gain values (LscTable)
 * - A radial polynomial (LscPolynomial)
 *
 * Table-based LSC tables are generated using an image at a fixed resolution and
 * can't at the moment be re-sampled when a different resolution is used for a
 * streaming session. Re-sampling a grid LSC table will return the same table
 * as loaded from the tuning file.
 *
 * \todo: Implement grid based re-sampling
 *
 * Polynomial configurations are more flexible and can be re-sampled for a given
 * sensor frame resolution using a list of horizontal and vertical nodes that
 * define the LSC grid on which the polynomial is re-sampled on.
 */

} /* namespace ipa */

} /* namespace libcamera */
