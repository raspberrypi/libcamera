/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 * Copyright (C) 2024 Red Hat Inc.
 *
 * Software ISP IPA Context
 */

#include "ipa_context.h"

/**
 * \file ipa_context.h
 * \brief Context and state information shared between the algorithms
 */

namespace libcamera::ipa::soft {

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
 * The IPA is fed with the statistics generated from the latest frame processed.
 * The statistics are then processed by the IPA algorithms to compute parameters
 * required for the next frame capture and processing. The current state of the
 * algorithms is reflected through the IPAActiveState to store the values most
 * recently computed by the IPA algorithms.
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
 * \var IPASessionConfiguration::gamma
 * \brief Gamma value to be used in the raw image processing
 */

/**
 * \var IPAActiveState::black
 * \brief Context for the Black Level algorithm
 *
 * \var IPAActiveState::black.level
 * \brief Current determined black level
 */

/**
 * \var IPAActiveState::gains
 * \brief Context for gains in the Colors algorithm
 *
 * \var IPAActiveState::gains.red
 * \brief Gain of red color
 *
 * \var IPAActiveState::gains.green
 * \brief Gain of green color
 *
 * \var IPAActiveState::gains.blue
 * \brief Gain of blue color
 */

/**
 * \var IPAActiveState::agc
 * \brief Context for the AGC algorithm
 *
 * \var IPAActiveState::agc.exposure
 * \brief Current exposure value
 *
 * \var IPAActiveState::agc.again
 * \brief Current analog gain value
 */

/**
 * \var IPAActiveState::gamma
 * \brief Context for gamma in the Colors algorithm
 *
 * \var IPAActiveState::gamma.gammaTable
 * \brief Computed gamma table
 *
 * \var IPAActiveState::gamma.blackLevel
 * \brief Black level used for the gamma table computation
 */

} /* namespace libcamera::ipa::soft */
