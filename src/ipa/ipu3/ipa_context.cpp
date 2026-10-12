/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * IPU3 IPA Context
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
 * \fn IPAContext::IPAContext
 * \brief Initialize the instance with the given number of frame contexts
 * \param[in] frameContextSize Size of the frame context ring buffer
 *
 * \var IPAContext::configuration
 * \brief The IPA session configuration, immutable during the session
 *
 * \var IPAContext::sensorInfo
 * \brief The IPA camera session details, immutable during the session
 *
 * \var IPAContext::sensorControls
 * \brief The camera sensor controls, immutable during the session
 *
 * \var IPAContext::frameContexts
 * \brief Ring buffer of the IPAFrameContext(s)
 *
 * \var IPAContext::activeState
 * \brief The current state of IPA algorithms
 *
 * \var IPAContext::camHelper
 * \brief The camera sensor helper
 *
 * \var IPAContext::ctrlMap
 * \brief A ControlInfoMap::Map of controls populated by the algorithms
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
 */

/**
 * \var IPAActiveState::agc
 * \brief Context for the Automatic Gain Control algorithm
 */

/**
 * \var IPAActiveState::awb
 * \brief Active auto-white balance parameters for the IPA
 */

/**
 * \var IPAActiveState::ccm
 * \brief Active colour Correction Matrix parameters for the IPA
 */

/**
 * \var IPAActiveState::gamma
 * \brief Active gamma correction parameters for the IPA
 */

/**
 * \var IPAActiveState::lsc
 * \brief Active lens shading correction parameters for the IPA
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
 *
 * \var IPAFrameContext::agc
 * \brief Per-frame state for the AGC algorithm
 */

/**
 * \var IPAFrameContext::awb
 * \brief Per-frame auto-white balance parameters for the IPA
 */

/**
 * \var IPAFrameContext::ccm
 * \brief Per-frame colour Correction Matrix parameters for the IPA
 */

/**
 * \var IPAFrameContext::gamma
 * \brief Per-frame gamma correction parameters for the IPA
 */

/**
 * \var IPAFrameContext::lsc
 * \brief Per-frame lens shading correction parameters for the IPA
 */

} /* namespace libcamera::ipa::ipu3 */
