/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 IPA Context
 */

#include "ipa_context.h"

/**
 * \file ipa_context.h
 * \brief Context and state information shared between the algorithms
 */

namespace libcamera::ipa::rkisp1 {

/**
 * \struct IPAHwSettings
 * \brief RkISP1 version-specific hardware parameters
 */

/**
 * \var IPAHwSettings::numAeCells
 * \brief Number of cells in the AE exposure means grid
 *
 * \var IPAHwSettings::numHistogramBins
 * \brief Number of bins in the histogram
 *
 * \var IPAHwSettings::numHistogramWeights
 * \brief Number of weights in the histogram grid
 *
 * \var IPAHwSettings::numGammaOutSamples
 * \brief Number of samples in the gamma out table
 */

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
 * \var IPASessionConfiguration::sensor.minExposureTime
 * \brief Minimum exposure time supported with the sensor
 *
 * \var IPASessionConfiguration::sensor.maxExposureTime
 * \brief Maximum exposure time supported with the sensor
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
 * \var IPASessionConfiguration::paramFormat
 * \brief The fourcc of the parameters buffers format
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
 * The \a automatic variables track the latest values computed by algorithm
 * based on the latest processed statistics. All other variables track the
 * consolidated controls requested in queued requests.
 *
 * \struct IPAActiveState::agc.manual
 * \brief Manual exposure time and analog gain (set through requests)
 *
 * \var IPAActiveState::agc.manual.exposure
 * \brief Manual exposure time expressed as a number of lines as set by the
 * ExposureTime control
 *
 * \var IPAActiveState::agc.manual.gain
 * \brief Manual analogue gain as set by the AnalogueGain control
 *
 * \struct IPAActiveState::agc.automatic
 * \brief Automatic exposure time and analog gain (computed by the algorithm)
 *
 * \var IPAActiveState::agc.automatic.exposure
 * \brief Automatic exposure time expressed as a number of lines
 *
 * \var IPAActiveState::agc.automatic.gain
 * \brief Automatic analogue gain multiplier
 *
 * \var IPAActiveState::agc.autoExposureEnabled
 * \brief Manual/automatic AGC state (exposure) as set by the ExposureTimeMode control
 *
 * \var IPAActiveState::agc.autoGainEnabled
 * \brief Manual/automatic AGC state (gain) as set by the AnalogueGainMode control
 *
 * \var IPAActiveState::agc.constraintMode
 * \brief Constraint mode as set by the AeConstraintMode control
 *
 * \var IPAActiveState::agc.exposureMode
 * \brief Exposure mode as set by the AeExposureMode control
 *
 * \var IPAActiveState::agc.meteringMode
 * \brief Metering mode as set by the AeMeteringMode control
 *
 * \var IPAActiveState::agc.minFrameDuration
 * \brief Minimum frame duration as set by the FrameDurationLimits control
 *
 * \var IPAActiveState::agc.maxFrameDuration
 * \brief Maximum frame duration as set by the FrameDurationLimits control
 */

/**
 * \var IPAActiveState::awb
 * \brief State for the Automatic White Balance algorithm
 *
 * \struct IPAActiveState::awb::AwbState
 * \brief Struct for the AWB regulation state
 *
 * \var IPAActiveState::awb::AwbState.gains
 * \brief White balance gains
 *
 * \var IPAActiveState::awb::AwbState.temperatureK
 * \brief Color temperature
 *
 * \var IPAActiveState::awb.manual
 * \brief Manual regulation state (set through requests)
 *
 * \var IPAActiveState::awb.automatic
 * \brief Automatic regulation state (computed by the algorithm)
 *
 * \var IPAActiveState::awb.autoEnabled
 * \brief Whether the Auto White Balance algorithm is enabled
 */

/**
 * \var IPAActiveState::ccm
 * \brief State for the Colour Correction Matrix algorithm
 *
 * \var IPAActiveState::ccm.manual
 * \brief Manual CCM (set through requests)
 *
 * \var IPAActiveState::awb.automatic
 * \brief Automatic CCM (computed by the algorithm)
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
 * \var IPAActiveState::goc
 * \brief State for the goc algorithm
 *
 * \var IPAActiveState::goc.gamma
 * \brief Gamma value applied as 1.0/gamma
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
 * applied to the sensor in order to take effect for this frame. Additionally
 * the vertical blanking period is determined to maintain a consistent frame
 * rate matched to the FrameDurationLimits as set by the user.
 *
 * \var IPAFrameContext::agc.exposure
 * \brief Exposure time expressed as a number of lines computed by the algorithm
 *
 * \var IPAFrameContext::agc.gain
 * \brief Analogue gain multiplier computed by the algorithm
 *
 * The gain should be adapted to the sensor specific gain code before applying.
 *
 * \var IPAFrameContext::agc.vblank
 * \brief Vertical blanking parameter computed by the algorithm
 *
 * \var IPAFrameContext::agc.autoExposureEnabled
 * \brief Manual/automatic AGC state (exposure) as set by the ExposureTimeMode control
 *
 * \var IPAFrameContext::agc.autoGainEnabled
 * \brief Manual/automatic AGC state (gain) as set by the AnalogueGainMode control
 *
 * \var IPAFrameContext::agc.constraintMode
 * \brief Constraint mode as set by the AeConstraintMode control
 *
 * \var IPAFrameContext::agc.exposureMode
 * \brief Exposure mode as set by the AeExposureMode control
 *
 * \var IPAFrameContext::agc.meteringMode
 * \brief Metering mode as set by the AeMeteringMode control
 *
 * \var IPAFrameContext::agc.minFrameDuration
 * \brief Minimum frame duration as set by the FrameDurationLimits control
 *
 * \var IPAFrameContext::agc.maxFrameDuration
 * \brief Maximum frame duration as set by the FrameDurationLimits control
 *
 * \var IPAFrameContext::agc.frameDuration
 * \brief The actual FrameDuration used by the algorithm for the frame
 *
 * \var IPAFrameContext::agc.updateMetering
 * \brief Indicate if new ISP AGC metering parameters need to be applied
 *
 * \var IPAFrameContext::agc.autoExposureModeChange
 * \brief Indicate if autoExposureEnabled has changed from true in the previous
 * frame to false in the current frame, and no manual exposure value has been
 * supplied in the current frame.
 *
 * \var IPAFrameContext::agc.autoGainModeChange
 * \brief Indicate if autoGainEnabled has changed from true in the previous
 * frame to false in the current frame, and no manual gain value has been
 * supplied in the current frame.
 */

/**
 * \var IPAFrameContext::awb
 * \brief Automatic White Balance parameters for this frame
 *
 * \struct IPAFrameContext::awb.gains
 * \brief White balance gains
 *
 * \var IPAFrameContext::awb.temperatureK
 * \brief Color temperature used for processing this frame
 *
 * This does not match the color temperature estimated for this frame as the
 * measurements were taken on a previous frame.
 *
 * \var IPAFrameContext::awb.autoEnabled
 * \brief Whether the Auto White Balance algorithm is enabled
 */

/**
 * \var IPAFrameContext::ccm
 * \brief Colour Correction Matrix parameters for this frame
 *
 * \struct IPAFrameContext::ccm.ccm
 * \brief Colour Correction Matrix
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
 * \var IPAFrameContext::goc
 * \brief Gamma out correction parameters for this frame
 *
 * \var IPAFrameContext::goc.gamma
 * \brief Gamma value applied as 1.0/gamma
 *
 * \var IPAFrameContext::goc.update
 * \brief Indicates if the goc parameters have been updated compared to the
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
 * \var IPAContext::hw
 * \brief RkISP1 version-specific hardware parameters
 *
 * \var IPAContext::sensorInfo
 * \brief The IPA session sensorInfo, immutable during the session
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
