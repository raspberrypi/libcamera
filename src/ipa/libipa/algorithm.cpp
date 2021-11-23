/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.cpp - IPA control algorithm interface
 */

#include "algorithm.h"

/**
 * \file algorithm.h
 * \brief Algorithm common interface
 */

namespace libcamera {

namespace ipa {

/**
 * \class Algorithm
 * \brief The base class for all IPA algorithms
 * \tparam Context The type of shared IPA context
 * \tparam Config The type of the IPA configuration data
 * \tparam Params The type of the ISP specific parameters
 * \tparam Stats The type of the IPA statistics and ISP results
 *
 * The Algorithm class defines a standard interface for IPA algorithms. By
 * abstracting algorithms, it makes possible the implementation of generic code
 * to manage algorithms regardless of their specific type.
 */

/**
 * \fn Algorithm::configure()
 * \brief Configure the Algorithm given an IPAConfigInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data, received from the pipeline
 * handler
 *
 * Algorithms may implement a configure operation to pre-calculate
 * parameters prior to commencing streaming.
 *
 * Configuration state may be stored in the IPASessionConfiguration structure of
 * the IPAContext.
 *
 * \return 0 if successful, an error code otherwise
 */

/**
 * \fn Algorithm::prepare()
 * \brief Fill the \a params buffer with ISP processing parameters for a frame
 * \param[in] context The shared IPA context
 * \param[out] params The ISP specific parameters.
 *
 * This function is called for every frame when the camera is running before it
 * is processed by the ISP to prepare the ISP processing parameters for that
 * frame.
 *
 * Algorithms shall fill in the parameter structure fields appropriately to
 * configure the ISP processing blocks that they are responsible for. This
 * includes setting fields and flags that enable those processing blocks.
 */

/**
 * \fn Algorithm::process()
 * \brief Process ISP statistics, and run algorithm operations
 * \param[in] context The shared IPA context
 * \param[in] stats The IPA statistics and ISP results
 *
 * This function is called while camera is running for every frame processed by
 * the ISP, to process statistics generated from that frame by the ISP.
 * Algorithms shall use this data to run calculations and update their state
 * accordingly.
 *
 * Processing shall not take an undue amount of time, and any extended or
 * computationally expensive calculations or operations must be handled
 * asynchronously in a separate thread.
 *
 * Algorithms can store state in their respective IPAFrameContext structures,
 * and reference state from the IPAFrameContext of other algorithms.
 *
 * \todo Historical data may be required as part of the processing.
 * Either the previous frame, or the IPAFrameContext state of the frame
 * that generated the statistics for this operation may be required for
 * some advanced algorithms to prevent oscillations or support control
 * loops correctly. Only a single IPAFrameContext is available currently,
 * and so any data stored may represent the results of the previously
 * completed operations.
 *
 * Care shall be taken to ensure the ordering of access to the information
 * such that the algorithms use up to date state as required.
 */

} /* namespace ipa */

} /* namespace libcamera */
