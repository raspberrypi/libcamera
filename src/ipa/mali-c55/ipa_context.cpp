/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * ipa_context.cpp - MaliC55 IPA Context
 */

#include "ipa_context.h"

/**
 * \file ipa_context.h
 * \brief Context and state information shared between the algorithms
 */

namespace libcamera::ipa::mali_c55 {

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

} /* namespace libcamera::ipa::mali_c55 */
