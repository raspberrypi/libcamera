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
 * \fn Debayer::Debayer(const GlobalConfiguration &configuration)
 * \brief Construct a Debayer object
 * \param[in] configuration Global configuration reference
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

Debayer::Debayer(const GlobalConfiguration &configuration) : bench_(configuration)
{
	/* Initialize color lookup tables */
	for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
		red_[i] = green_[i] = blue_[i] = i;
		redCcm_[i] = { static_cast<int16_t>(i), 0, 0 };
		greenCcm_[i] = { 0, static_cast<int16_t>(i), 0 };
		blueCcm_[i] = { 0, 0, static_cast<int16_t>(i) };
	}
}

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
 * \fn const SharedFD &Debayer::getStatsFD()
 * \brief Get the file descriptor for the statistics
 *
 * This file descriptor provides access to the output statistics buffer
 * associated with the current debayering process.
 *
 * \return The file descriptor pointing to the statistics data
 */

/**
 * \fn unsigned int Debayer::frameSize()
 * \brief Get the output frame size
 *
 * \return The total output frame size in bytes as configured for the
 * current stream.
 */

/**
 * \var Signal<FrameBuffer *> Debayer::inputBufferReady
 * \brief Signals when the input buffer is ready
 */

/**
 * \var Signal<FrameBuffer *> Debayer::outputBufferReady
 * \brief Signals when the output buffer is ready
 */

/**
 * \struct Debayer::DebayerInputConfig
 * \brief Structure describing the incoming Bayer parameters
 *
 * The DebayerInputConfig structure defines the characteristics of the raw
 * Bayer frame being processed, including:
 *  - The Bayer pattern dimensions (\ref patternSize)
 *  - Memory layout parameters such as stride and bytes per pixel (\ref bpp)
 *  - A list of supported output pixel formats.
 *
 * \var Debayer::DebayerInputConfig::patternSize
 * Size of the Bayer pattern in pixels. For standard Bayer formats such as
 * BGGR, GRBG, GBRG, and RGGB, this is typically 2×2 pixels.
 *
 * \var Debayer::DebayerInputConfig::bpp
 * Number of bytes used per pixel in memory. This reflects storage size,
 * not precision.
 *
 * \var Debayer::DebayerInputConfig::stride
 * Line stride in bytes for the Bayer input frame.
 *
 * \var Debayer::DebayerInputConfig::outputFormats
 * List of pixel formats supported as output for this input configuration.
 */

/**
 * \struct Debayer::DebayerOutputConfig
 * \brief Structure describing the output frame configuration
 *
 * Defines how the output of the debayer process is laid out in memory.
 * It includes per-pixel size, stride, and total frame size.
 *
 * \var Debayer::DebayerOutputConfig::bpp
 * Bytes used per pixel in the output format.
 *
 * \var Debayer::DebayerOutputConfig::stride
 * Line stride in bytes for the output frame.
 *
 * \var Debayer::DebayerOutputConfig::frameSize
 * Total frame size in bytes for the output buffer.
 */

/**
 * \var Debayer::inputConfig_
 * \brief Input configuration parameters for the current debayer operation
 *
 * Holds metadata describing the incoming Bayer image layout, including
 * pattern size, bytes per pixel, stride, and supported output formats.
 * Populated during configuration.
 */

/**
 * \var Debayer::outputConfig_
 * \brief Output configuration data for the debayered frame
 *
 * Contains bytes per pixel, stride, and total frame size for the
 * output image buffer. Set during stream configuration.
 */

/**
 * \var Debayer::red_
 * \brief Lookup table for red channel gain and correction values
 *
 * This table provides precomputed per-pixel or per-intensity
 * correction values for the red color channel used during debayering.
 */

/**
 * \var Debayer::green_
 * \brief Lookup table for green channel gain and correction values
 *
 * This table provides precomputed per-pixel or per-intensity
 * correction values for the green color channel used during debayering.
 */

/**
 * \var Debayer::blue_
 * \brief Lookup table for blue channel gain and correction values
 *
 * This table provides precomputed per-pixel or per-intensity
 * correction values for the blue color channel used during debayering.
 */

/**
 * \var Debayer::redCcm_
 * \brief Red channel Color Correction Matrix (CCM) lookup table
 *
 * Contains coefficients for green channel color correction.
 */

/**
 * \var Debayer::greenCcm_
 * \brief Green channel Color Correction Matrix (CCM) lookup table
 *
 * Contains coefficients for green channel color correction.
 */

/**
 * \var Debayer::blueCcm_
 * \brief Blue channel Color Correction Matrix (CCM) lookup table
 *
 * Contains coefficients for blue channel color correction.
 */

/**
 * \var Debayer::gammaLut_
 * \brief Gamma correction lookup table
 */

/**
 * \var Debayer::swapRedBlueGains_
 * \brief Flag indicating whether red and blue channel gains should be swapped
 *
 * Used when the Bayer pattern order indicates that red/blue color channels are
 * reversed.
 */

/**
 * \var Debayer::bench_
 * \brief Benchmarking utility instance for performance measurements
 *
 * Used internally to track timing and performance metrics during
 * debayer processing.
 */

/**
 * \fn void Debayer::setParams(DebayerParams &params)
 * \brief Select the bayer params to use for the next frame debayer
 * \param[in] params The parameters to be used in debayering
 */
void Debayer::setParams(DebayerParams &params)
{
	green_ = params.green;
	greenCcm_ = params.greenCcm;
	if (swapRedBlueGains_) {
		red_ = params.blue;
		blue_ = params.red;
		redCcm_ = params.blueCcm;
		blueCcm_ = params.redCcm;
		for (unsigned int i = 0; i < 256; i++) {
			std::swap(redCcm_[i].r, redCcm_[i].b);
			std::swap(greenCcm_[i].r, greenCcm_[i].b);
			std::swap(blueCcm_[i].r, blueCcm_[i].b);
		}
	} else {
		red_ = params.red;
		blue_ = params.blue;
		redCcm_ = params.redCcm;
		blueCcm_ = params.blueCcm;
	}
	gammaLut_ = params.gammaLut;
}

/**
 * \fn void Debayer::dmaSyncBegin(DebayerParams &params)
 * \brief Common CPU/GPU Dma Sync Buffer begin
 */
void Debayer::dmaSyncBegin(std::vector<DmaSyncer> &dmaSyncers, FrameBuffer *input, FrameBuffer *output)
{
	for (const FrameBuffer::Plane &plane : input->planes())
		dmaSyncers.emplace_back(plane.fd, DmaSyncer::SyncType::Read);

	if (output) {
		for (const FrameBuffer::Plane &plane : output->planes())
			dmaSyncers.emplace_back(plane.fd, DmaSyncer::SyncType::Write);
	}
}

} /* namespace libcamera */
