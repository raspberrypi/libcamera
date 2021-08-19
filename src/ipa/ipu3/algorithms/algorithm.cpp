/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.cpp - IPU3 control algorithm interface
 */

#include "algorithm.h"

/**
 * \file algorithm.h
 * \brief Algorithm common interface
 */

namespace libcamera {

namespace ipa::ipu3 {

/**
 * \class Algorithm
 * \brief The base class for all IPU3 algorithms
 *
 * The Algorithm class defines a standard interface for IPA algorithms. By
 * abstracting algorithms, it makes possible the implementation of generic code
 * to manage algorithms regardless of their specific type.
 */

/**
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
int Algorithm::configure([[maybe_unused]] IPAContext &context,
			 [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	return 0;
}

/**
 * \brief Fill the \a params buffer with ISP processing parameters for a frame
 * \param[in] context The shared IPA context
 * \param[out] params The IPU3 specific parameters.
 *
 * This function is called for every frame when the camera is running before it
 * is processed by the ImgU to prepare the ImgU processing parameters for that
 * frame.
 *
 * Algorithms shall fill in the parameter structure fields appropriately to
 * configure the ImgU processing blocks that they are responsible for. This
 * includes setting fields and flags that enable those processing blocks.
 */
void Algorithm::prepare([[maybe_unused]] IPAContext &context,
			[[maybe_unused]] ipu3_uapi_params *params)
{
}

/**
 * \brief Process ISP statistics, and run algorithm operations
 * \param[in] context The shared IPA context
 * \param[in] stats The IPU3 statistics and ISP results
 *
 * This function is called while camera is running for every frame processed by
 * the ImgU, to process statistics generated from that frame by the ImgU.
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
void Algorithm::process([[maybe_unused]] IPAContext &context,
			[[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
}

} /* namespace ipa::ipu3 */

} /* namespace libcamera */
