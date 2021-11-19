/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * ipa_context.cpp - IPU3 IPA Context
 */

#include "ipa_context.h"

/**
 * \file ipa_context.h
 * \brief Context and state information shared between the algorithms
 */

namespace libcamera::ipa::ipu3 {

/**
 * \struct IPASessionConfiguration
 * \brief Session configuration for the IPA module
 *
 * The session configuration contains all IPA configuration parameters that
 * remain constant during the capture session, from IPA module start to stop.
 * It is typically set during the configure() operation of the IPA module, but
 * may also be updated in the start() operation.
 */

/**
 * \struct IPAFrameContext
 * \brief Per-frame context for algorithms
 *
 * The frame context stores data specific to a single frame processed by the
 * IPA. Each frame processed by the IPA has a context associated with it,
 * accessible through the IPAContext structure.
 *
 * \todo Detail how to access contexts for a particular frame
 *
 * Each of the fields in the frame context belongs to either a specific
 * algorithm, or to the top-level IPA module. A field may be read by any
 * algorithm, but should only be written by its owner.
 */

/**
 * \struct IPAContext
 * \brief Global IPA context data shared between all algorithms
 *
 * \var IPAContext::configuration
 * \brief The IPA session configuration, immutable during the session
 *
 * \var IPAContext::frameContext
 * \brief The frame context for the frame being processed
 *
 * \todo While the frame context is supposed to be per-frame, this
 * single frame context stores data related to both the current frame
 * and the previous frames, with fields being updated as the algorithms
 * are run. This needs to be turned into real per-frame data storage.
 */

/**
 * \var IPASessionConfiguration::grid
 * \brief Grid configuration of the IPA
 *
 * \var IPASessionConfiguration::grid.bdsGrid
 * \brief Bayer Down Scaler grid plane config used by the kernel
 *
 * \var IPASessionConfiguration::grid.bdsOutputSize
 * \brief BDS output size configured by the pipeline handler
 *
 * \var IPASessionConfiguration::grid.stride
 * \brief Number of cells on one line including the ImgU padding
 */

/**
 * \var IPASessionConfiguration::agc
 * \brief AGC parameters configuration of the IPA
 *
 * \var IPASessionConfiguration::agc.minShutterSpeed
 * \brief Minimum shutter speed supported with the configured sensor
 *
 * \var IPASessionConfiguration::agc.maxShutterSpeed
 * \brief Maximum shutter speed supported with the configured sensor
 *
 * \var IPASessionConfiguration::agc.minAnalogueGain
 * \brief Minimum analogue gain supported with the configured sensor
 *
 * \var IPASessionConfiguration::agc.maxAnalogueGain
 * \brief Maximum analogue gain supported with the configured sensor
 */

/**
 * \var IPAFrameContext::agc
 * \brief Context for the Automatic Gain Control algorithm
 *
 * The exposure and gain determined are expected to be applied to the sensor
 * at the earliest opportunity.
 *
 * \var IPAFrameContext::agc.exposure
 * \brief Exposure time expressed as a number of lines
 *
 * \var IPAFrameContext::agc.gain
 * \brief Analogue gain multiplier
 *
 * The gain should be adapted to the sensor specific gain code before applying.
 */

/**
 * \var IPAFrameContext::awb
 * \brief Context for the Automatic White Balance algorithm
 *
 * \struct IPAFrameContext::awb.gains
 * \brief White balance gains
 *
 * \var IPAFrameContext::awb.gains.red
 * \brief White balance gain for R channel
 *
 * \var IPAFrameContext::awb.gains.green
 * \brief White balance gain for G channel
 *
 * \var IPAFrameContext::awb.gains.blue
 * \brief White balance gain for B channel
 *
 * \var IPAFrameContext::awb.temperatureK
 * \brief Estimated color temperature
 */

/**
 * \var IPAFrameContext::sensor
 * \brief Effective sensor values
 *
 * \var IPAFrameContext::sensor.exposure
 * \brief Exposure time expressed as a number of lines
 *
 * \var IPAFrameContext::sensor.gain
 * \brief Analogue gain multiplier
 */

/**
 * \var IPAFrameContext::toneMapping
 * \brief Context for ToneMapping and Gamma control
 *
 * \var IPAFrameContext::toneMapping.gamma
 * \brief Gamma value for the LUT
 *
 * \var IPAFrameContext::toneMapping.gammaCorrection
 * \brief Per-pixel tone mapping implemented as a LUT
 *
 * The LUT structure is defined by the IPU3 kernel interface. See
 * <linux/intel-ipu3.h> struct ipu3_uapi_gamma_corr_lut for further details.
 */

} /* namespace libcamera::ipa::ipu3 */
