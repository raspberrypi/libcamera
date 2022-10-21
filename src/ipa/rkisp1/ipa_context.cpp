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
 * \var IPASessionConfiguration::agc.measureWindow
 * \brief AGC measure window
 */

/**
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
 * \var IPASessionConfiguration::sensor.minShutterSpeed
 * \brief Minimum shutter speed supported with the sensor
 *
 * \var IPASessionConfiguration::sensor.maxShutterSpeed
 * \brief Maximum shutter speed supported with the sensor
 *
 * \var IPASessionConfiguration::sensor.minAnalogueGain
 * \brief Minimum analogue gain supported with the sensor
 *
 * \var IPASessionConfiguration::sensor.maxAnalogueGain
 * \brief Maximum analogue gain supported with the sensor
 *
 * \var IPASessionConfiguration::sensor.defVBlank
 * \brief The default vblank value of the sensor
 *
 * \var IPASessionConfiguration::sensor.lineDuration
 * \brief Line duration in microseconds
 *
 * \var IPASessionConfiguration::sensor.size
 * \brief Sensor output resolution
 */

/**
 * \var IPASessionConfiguration::raw
 * \brief Indicates if the camera is configured to capture raw frames
 */

/**
 * \struct IPAActiveState
 * \brief Active state for algorithms
 *
 * The active state contains all algorithm-specific data that needs to be
 * maintained by algorithms across frames. Unlike the session configuration,
 * the active state is mutable and constantly updated by algorithms. The active
 * state is accessible through the IPAContext structure.
 *
 * The active state stores two distinct categories of information:
 *
 *  - The consolidated value of all algorithm controls. Requests passed to
 *    the queueRequest() function store values for controls that the
 *    application wants to modify for that particular frame, and the
 *    queueRequest() function updates the active state with those values.
 *    The active state thus contains a consolidated view of the value of all
 *    controls handled by the algorithm.
 *
 *  - The value of parameters computed by the algorithm when running in auto
 *    mode. Algorithms running in auto mode compute new parameters every
 *    time statistics buffers are received (either synchronously, or
 *    possibly in a background thread). The latest computed value of those
 *    parameters is stored in the active state in the process() function.
 *
 * Each of the members in the active state belongs to a specific algorithm. A
 * member may be read by any algorithm, but shall only be written by its owner.
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
 * \var IPAActiveState::awb.temperatureK
 * \brief Estimated color temperature
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
 */

/**
 * \var IPAActiveState::dpf
 * \brief State for the Denoise Pre-Filter algorithm
 *
 * \var IPAActiveState::dpf.denoise
 * \brief Indicates if denoise is activated
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
 */

/**
 * \struct IPAFrameContext
 * \brief Per-frame context for algorithms
 *
 * The frame context stores two distinct categories of information:
 *
 * - The value of the controls to be applied to the frame. These values are
 *   typically set in the queueRequest() function, from the consolidated
 *   control values stored in the active state. The frame context thus stores
 *   values for all controls related to the algorithm, not limited to the
 *   controls specified in the corresponding request, but consolidated from all
 *   requests that have been queued so far.
 *
 *   For controls that can be set manually or computed by an algorithm
 *   (depending on the algorithm operation mode), such as for instance the
 *   colour gains for the AWB algorithm, the control value will be stored in
 *   the frame context in the queueRequest() function only when operating in
 *   manual mode. When operating in auto mode, the values are computed by the
 *   algorithm in process(), stored in the active state, and copied to the
 *   frame context in prepare(), just before being stored in the ISP parameters
 *   buffer.
 *
 *   The queueRequest() function can also store ancillary data in the frame
 *   context, such as flags to indicate if (and what) control values have
 *   changed compared to the previous request.
 *
 * - Status information computed by the algorithm for a frame. For instance,
 *   the colour temperature estimated by the AWB algorithm from ISP statistics
 *   calculated on a frame is stored in the frame context for that frame in
 *   the process() function.
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
 * \var IPAFrameContext::cproc
 * \brief Color Processing parameters for this frame
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
 * \var IPAFrameContext::cproc.update
 * \brief Indicates if the color processing parameters have been updated
 * compared to the previous frame
 */

/**
 * \var IPAFrameContext::dpf
 * \brief Denoise Pre-Filter parameters for this frame
 *
 * \var IPAFrameContext::dpf.denoise
 * \brief Indicates if denoise is activated
 *
 * \var IPAFrameContext::dpf.update
 * \brief Indicates if the denoise pre-filter parameters have been updated
 * compared to the previous frame
 */

/**
 * \var IPAFrameContext::filter
 * \brief Filter parameters for this frame
 *
 * \struct IPAFrameContext::filter.denoise
 * \brief Denoising level
 *
 * \var IPAFrameContext::filter.sharpness
 * \brief Sharpness level
 *
 * \var IPAFrameContext::filter.updateParams
 * \brief Indicates if the filter parameters have been updated compared to the
 * previous frame
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
