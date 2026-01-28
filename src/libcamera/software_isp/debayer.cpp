/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023-2026 Red Hat Inc.
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
 * \var DebayerParams::gains
 * \brief Colour channel gains
 */

/**
 * \var DebayerParams::combinedMatrix
 * \brief Colour correction matrix, including other adjustments
 */

/**
 * \var DebayerParams::blackLevel
 * \brief Black level values
 */

/**
 * \var DebayerParams::gamma
 * \brief Gamma value, e.g. 1/2.2
 */

/**
 * \var DebayerParams::contrastExp
 * \brief Contrast value to be used as an exponent
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
 * \var Debayer::inputPixelFormat_
 * \brief The incoming pixel format
 */

/**
 * \var Debayer::outputPixelFormat_
 * \brief The output pixel format
 */

/**
 * \var Debayer::outputSize_
 * \brief Output size object
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
 * \fn int Debayer::start()
 * \brief Execute a start signal in the debayer object from workerthread context
 *
 * The start() method is invoked so that a Debayer object can initialise
 * internal variables or data. It is called from the software_isp::start
 * method.
 *
 * This method is particularly useful with DebayerEGL as it allows for the
 * initialisation of the EGL stack after configure in a thread-safe manner.
 */

/**
 * \fn void Debayer::stop()
 * \brief Stop the debayering process and perform cleanup
 *
 * The stop() method is invoked as the logically corollary of start().
 * Debayer::stop() will be called by software_isp::stop() allowing for any
 * cleanup which should happend with stop().
 *
 * The stop method similar to start() is useful for DebayerEGL as it allows
 * for cleanup of EGL context and/or data that happens in
 * DebayerEGL::start.
 */

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

/**
 * \fn void Debayer::isStandardBayerOrder(BayerFormat::Order order)
 * \brief Common method to validate standard 2x2 Bayer pattern of 2 Green, 1 Blue, 1 Red pixels
 */
bool Debayer::isStandardBayerOrder(BayerFormat::Order order)
{
	return order == BayerFormat::BGGR || order == BayerFormat::GBRG ||
	       order == BayerFormat::GRBG || order == BayerFormat::RGGB;
}

} /* namespace libcamera */
