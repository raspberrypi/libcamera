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
 * \struct IPAActiveState
 * \brief Active state for algorithms
 *
 * The active state stores algorithm-specific data that needs to be shared
 * between multiple algorithms and the IPA module. It is accessible through the
 * IPAContext structure.
 *
 * \todo Split the data contained in this structure between the active state
 * and the frame contexts.
 *
 * Each of the fields in the active state belongs to either a specific
 * algorithm, or to the top-level IPA module. A field may be read by any
 * algorithm, but should only be written by its owner.
 */

/**
 * \var IPAActiveState::agc
 * \brief State for the Automatic Gain Control algorithm
 *
 * The exposure and gain are the latest values computed by the AGC algorithm.
 *
 * \var IPAActiveState::agc.exposure
 * \brief Exposure time expressed as a number of lines
 *
 * \var IPAActiveState::agc.gain
 * \brief Analogue gain multiplier
 */

/**
 * \var IPAActiveState::awb
 * \brief State for the Automatic White Balance algorithm
 *
 * \struct IPAActiveState::awb.gains
 * \brief White balance gains
 *
 * \struct IPAActiveState::awb.gains.manual
 * \brief Manual white balance gains (set through requests)
 *
 * \var IPAActiveState::awb.gains.manual.red
 * \brief Manual white balance gain for R channel
 *
 * \var IPAActiveState::awb.gains.manual.green
 * \brief Manual white balance gain for G channel
 *
 * \var IPAActiveState::awb.gains.manual.blue
 * \brief Manual white balance gain for B channel
 *
 * \struct IPAActiveState::awb.gains.automatic
 * \brief Automatic white balance gains (computed by the algorithm)
 *
 * \var IPAActiveState::awb.gains.automatic.red
 * \brief Automatic white balance gain for R channel
 *
 * \var IPAActiveState::awb.gains.automatic.green
 * \brief Automatic white balance gain for G channel
 *
 * \var IPAActiveState::awb.gains.automatic.blue
 * \brief Automatic white balance gain for B channel
 *
 * \var IPAActiveState::awb.autoEnabled
 * \brief Whether the Auto White Balance algorithm is enabled
 */

/**
 * \var IPAActiveState::cproc
 * \brief State for the Color Processing algorithm
 *
 * \struct IPAActiveState::cproc.brightness
 * \brief Brightness level
 *
 * \var IPAActiveState::cproc.contrast
 * \brief Contrast level
 *
 * \var IPAActiveState::cproc.saturation
 * \brief Saturation level
 *
 * \var IPAActiveState::cproc.updateParams
 * \brief Indicates if ISP parameters need to be updated
 */

/**
 * \var IPAActiveState::dpf
 * \brief State for the Denoise Pre-Filter algorithm
 *
 * \var IPAActiveState::dpf.denoise
 * \brief Indicates if denoise is activated
 *
 * \var IPAActiveState::dpf.updateParams
 * \brief Indicates if ISP parameters need to be updated
 */

/**
 * \var IPAActiveState::filter
 * \brief State for the Filter algorithm
 *
 * \struct IPAActiveState::filter.denoise
 * \brief Denoising level
 *
 * \var IPAActiveState::filter.sharpness
 * \brief Sharpness level
 *
 * \var IPAActiveState::filter.updateParams
 * \brief Indicates if ISP parameters need to be updated
 */

/**
 * \struct IPAFrameContext
 * \brief Per-frame context for algorithms
 *
 * \todo Populate the frame context for all algorithms
 */

/**
 * \var IPAFrameContext::agc
 * \brief Automatic Gain Control parameters for this frame
 *
 * The exposure and gain are provided by the AGC algorithm, and are to be
 * applied to the sensor in order to take effect for this frame.
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
 * \brief Automatic White Balance parameters for this frame
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
 * \var IPAFrameContext::sensor
 * \brief Sensor configuration that used been used for this frame
 *
 * \var IPAFrameContext::sensor.exposure
 * \brief Exposure time expressed as a number of lines
 *
 * \var IPAFrameContext::sensor.gain
 * \brief Analogue gain multiplier
 */

/**
 * \struct IPAContext
 * \brief Global IPA context data shared between all algorithms
 *
 * \var IPAContext::configuration
 * \brief The IPA session configuration, immutable during the session
 *
 * \var IPAContext::activeState
 * \brief The IPA active state, storing the latest state for all algorithms
 *
 * \var IPAContext::frameContexts
 * \brief Ring buffer of per-frame contexts
 */

} /* namespace libcamera::ipa::rkisp1 */
