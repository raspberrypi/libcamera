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
 * \struct IPAActiveState
 * \brief The active state of the IPA algorithms
 *
 * The IPA is fed with the statistics generated from the latest frame captured
 * by the hardware. The statistics are then processed by the IPA algorithms to
 * compute ISP parameters required for the next frame capture. The current state
 * of the algorithms is reflected through the IPAActiveState to store the values
 * most recently computed by the IPA algorithms.
 */

/**
 * \struct IPAContext
 * \brief Global IPA context data shared between all algorithms
 *
 * \var IPAContext::configuration
 * \brief The IPA session configuration, immutable during the session
 *
 * \var IPAContext::frameContexts
 * \brief Ring buffer of the IPAFrameContext(s)
 *
 * \var IPAContext::activeState
 * \brief The current state of IPA algorithms
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
 * \var IPASessionConfiguration::af
 * \brief AF grid configuration of the IPA
 *
 * \var IPASessionConfiguration::af.afGrid
 * \brief AF scene grid configuration
 */

/**
 * \var IPAActiveState::af
 * \brief Context for the Automatic Focus algorithm
 *
 * \var IPAActiveState::af.focus
 * \brief Current position of the lens
 *
 * \var IPAActiveState::af.maxVariance
 * \brief The maximum variance of the current image
 *
 * \var IPAActiveState::af.stable
 * \brief It is set to true, if the best focus is found
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
 * \var IPASessionConfiguration::sensor
 * \brief Sensor-specific configuration of the IPA
 *
 * \var IPASessionConfiguration::sensor.lineDuration
 * \brief Line duration in microseconds
 *
 * \var IPASessionConfiguration::sensor.defVBlank
 * \brief The default vblank value of the sensor
 *
 * \var IPASessionConfiguration::sensor.size
 * \brief Sensor output resolution
 */

/**
 * \var IPAActiveState::agc
 * \brief Context for the Automatic Gain Control algorithm
 *
 * The exposure and gain determined are expected to be applied to the sensor
 * at the earliest opportunity.
 *
 * \var IPAActiveState::agc.exposure
 * \brief Exposure time expressed as a number of lines
 *
 * \var IPAActiveState::agc.gain
 * \brief Analogue gain multiplier
 *
 * The gain should be adapted to the sensor specific gain code before applying.
 */

/**
 * \var IPAActiveState::awb
 * \brief Context for the Automatic White Balance algorithm
 *
 * \var IPAActiveState::awb.gains
 * \brief White balance gains
 *
 * \var IPAActiveState::awb.gains.red
 * \brief White balance gain for R channel
 *
 * \var IPAActiveState::awb.gains.green
 * \brief White balance gain for G channel
 *
 * \var IPAActiveState::awb.gains.blue
 * \brief White balance gain for B channel
 *
 * \var IPAActiveState::awb.temperatureK
 * \brief Estimated color temperature
 */

/**
 * \var IPAActiveState::toneMapping
 * \brief Context for ToneMapping and Gamma control
 *
 * \var IPAActiveState::toneMapping.gamma
 * \brief Gamma value for the LUT
 *
 * \var IPAActiveState::toneMapping.gammaCorrection
 * \brief Per-pixel tone mapping implemented as a LUT
 *
 * The LUT structure is defined by the IPU3 kernel interface. See
 * <linux/intel-ipu3.h> struct ipu3_uapi_gamma_corr_lut for further details.
 */

/**
 * \struct IPAFrameContext
 * \brief IPU3-specific FrameContext
 *
 * \var IPAFrameContext::sensor
 * \brief Effective sensor values that were applied for the frame
 *
 * \var IPAFrameContext::sensor.exposure
 * \brief Exposure time expressed as a number of lines
 *
 * \var IPAFrameContext::sensor.gain
 * \brief Analogue gain multiplier
 */

} /* namespace libcamera::ipa::ipu3 */
