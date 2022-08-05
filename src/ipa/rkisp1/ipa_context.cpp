/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * ipa_context.cpp - RkISP1 IPA Context
 */

#include "ipa_context.h"

/**
 * \file ipa_context.h
 * \brief Context and state information shared between the algorithms
 */

namespace libcamera::ipa::rkisp1 {

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
 *
 * \var IPASessionConfiguration::agc.measureWindow
 * \brief AGC measure window
 *
 * \var IPASessionConfiguration::hw
 * \brief RkISP1-specific hardware information
 *
 * \var IPASessionConfiguration::hw.revision
 * \brief Hardware revision of the ISP
 */

/**
 * \var IPASessionConfiguration::awb
 * \brief AWB parameters configuration of the IPA
 *
 * \var IPASessionConfiguration::awb.measureWindow
 * \brief AWB measure window
 *
 * \var IPASessionConfiguration::awb.enabled
 * \brief Indicates if the AWB hardware is enabled and applies colour gains
 *
 * The AWB module of the ISP applies colour gains and computes statistics. It is
 * enabled when the AWB algorithm is loaded, regardless of whether the algorithm
 * operates in manual or automatic mode.
 */

/**
 * \var IPASessionConfiguration::lsc
 * \brief Lens Shading Correction configuration of the IPA
 *
 * \var IPASessionConfiguration::lsc.enabled
 * \brief Indicates if the LSC hardware is enabled
 */

/**
 * \var IPASessionConfiguration::sensor
 * \brief Sensor-specific configuration of the IPA
 *
 * \var IPASessionConfiguration::sensor.lineDuration
 * \brief Line duration in microseconds
 *
 * \var IPASessionConfiguration::sensor.size
 * \brief Sensor output resolution
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
 *
 * \var IPAFrameContext::awb.autoEnabled
 * \brief Whether the Auto White Balance algorithm is enabled
 */

/**
 * \var IPAFrameContext::cproc
 * \brief Context for the Color Processing algorithm
 *
 * \struct IPAFrameContext::cproc.brightness
 * \brief Brightness level
 *
 * \var IPAFrameContext::cproc.contrast
 * \brief Contrast level
 *
 * \var IPAFrameContext::cproc.saturation
 * \brief Saturation level
 *
 * \var IPAFrameContext::cproc.updateParams
 * \brief Indicates if ISP parameters need to be updated
 */

/**
 * \var IPAFrameContext::dpf
 * \brief Context for the Denoise Pre-Filter algorithm
 *
 * \var IPAFrameContext::dpf.denoise
 * \brief Indicates if denoise is activated
 *
 * \var IPAFrameContext::dpf.updateParams
 * \brief Indicates if ISP parameters need to be updated
 */

/**
 * \var IPAFrameContext::filter
 * \brief Context for the Filter algorithm
 *
 * \struct IPAFrameContext::filter.denoise
 * \brief Denoising level
 *
 * \var IPAFrameContext::filter.sharpness
 * \brief Sharpness level
 *
 * \var IPAFrameContext::filter.updateParams
 * \brief Indicates if ISP parameters need to be updated
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
 * \var IPAFrameContext::frameCount
 * \brief Counter of requests queued to the IPA module
 *
 * The counter is reset to 0 when the IPA module is configured, and is
 * incremented for each request being queued, after calling the
 * Algorithm::prepare() function of all algorithms.
 */

} /* namespace libcamera::ipa::rkisp1 */
