/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023-2025 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * debayer base class
 */

#include "debayer.h"

namespace libcamera {

/**
 * \struct DebayerParams
 * \brief Struct to hold the debayer parameters.
 */

/**
 * \var DebayerParams::kRGBLookupSize
 * \brief Size of a color lookup table
 */

/**
 * \struct DebayerParams::CcmColumn
 * \brief Type of a single column of a color correction matrix (CCM)
 *
 * When multiplying an input pixel, columns in the CCM correspond to the red,
 * green or blue component of input pixel values, while rows correspond to the
 * red, green or blue components of the output pixel values. The members of the
 * CcmColumn structure are named after the colour components of the output pixel
 * values they correspond to.
 */

/**
 * \var DebayerParams::CcmColumn::r
 * \brief Red (first) component of a CCM column
 */

/**
 * \var DebayerParams::CcmColumn::g
 * \brief Green (second) component of a CCM column
 */

/**
 * \var DebayerParams::CcmColumn::b
 * \brief Blue (third) component of a CCM column
 */

/**
 * \typedef DebayerParams::LookupTable
 * \brief Type of the lookup tables for single lookup values
 */

/**
 * \typedef DebayerParams::CcmLookupTable
 * \brief Type of the CCM lookup tables for red, green, blue values
 */

/**
 * \var DebayerParams::red
 * \brief Lookup table for red color, mapping input values to output values
 */

/**
 * \var DebayerParams::green
 * \brief Lookup table for green color, mapping input values to output values
 */

/**
 * \var DebayerParams::blue
 * \brief Lookup table for blue color, mapping input values to output values
 */

/**
 * \var DebayerParams::redCcm
 * \brief Lookup table for the CCM red column, mapping input values to output values
 */

/**
 * \var DebayerParams::greenCcm
 * \brief Lookup table for the CCM green column, mapping input values to output values
 */

/**
 * \var DebayerParams::blueCcm
 * \brief Lookup table for the CCM blue column, mapping input values to output values
 */

/**
 * \var DebayerParams::gammaLut
 * \brief Gamma lookup table used with color correction matrix
 */

/**
 * \class Debayer
 * \brief Base debayering class
 *
 * Base class that provides functions for setting up the debayering process.
 */

LOG_DEFINE_CATEGORY(Debayer)

Debayer::~Debayer()
{
}

/**
 * \fn int Debayer::configure()
 * \brief Configure the debayer object according to the passed in parameters
 * \param[in] inputCfg The input configuration
 * \param[in] outputCfgs The output configurations
 * \param[in] ccmEnabled Whether a color correction matrix is applied
 *
 * \return 0 on success, a negative errno on failure
 */

/**
 * \fn Size Debayer::patternSize(PixelFormat inputFormat)
 * \brief Get the width and height at which the bayer pattern repeats
 * \param[in] inputFormat The input format
 *
 * Valid sizes are: 2x2, 4x2 or 4x4.
 *
 * \return Pattern size or an empty size for unsupported inputFormats
 */

/**
 * \fn std::vector<PixelFormat> Debayer::formats(PixelFormat inputFormat)
 * \brief Get the supported output formats
 * \param[in] inputFormat The input format
 *
 * \return All supported output formats or an empty vector if there are none
 */

/**
 * \fn std::tuple<unsigned int, unsigned int> Debayer::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
 * \brief Get the stride and the frame size
 * \param[in] outputFormat The output format
 * \param[in] size The output size
 *
 * \return A tuple of the stride and the frame size, or a tuple with 0,0 if
 * there is no valid output config
 */

/**
 * \fn void Debayer::process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params)
 * \brief Process the bayer data into the requested format
 * \param[in] frame The frame number
 * \param[in] input The input buffer
 * \param[in] output The output buffer
 * \param[in] params The parameters to be used in debayering
 *
 * \note DebayerParams is passed by value deliberately so that a copy is passed
 * when this is run in another thread by invokeMethod().
 */

/**
 * \fn virtual SizeRange Debayer::sizes(PixelFormat inputFormat, const Size &inputSize)
 * \brief Get the supported output sizes for the given input format and size
 * \param[in] inputFormat The input format
 * \param[in] inputSize The input size
 *
 * \return The valid size ranges or an empty range if there are none
 */

/**
 * \var Signal<FrameBuffer *> Debayer::inputBufferReady
 * \brief Signals when the input buffer is ready
 */

/**
 * \var Signal<FrameBuffer *> Debayer::outputBufferReady
 * \brief Signals when the output buffer is ready
 */

} /* namespace libcamera */
