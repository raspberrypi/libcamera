/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
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

} /* namespace libcamera::ipa::rkisp1 */
