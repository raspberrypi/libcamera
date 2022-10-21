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
 * \tparam Module The IPA module type for this class of algorithms
 *
 * The Algorithm class defines a standard interface for IPA algorithms
 * compatible with the \a Module. By abstracting algorithms, it makes possible
 * the implementation of generic code to manage algorithms regardless of their
 * specific type.
 *
 * To specialize the Algorithm class template, an IPA module shall specialize
 * the Module class template with module-specific context and configuration
 * types, and pass the specialized Module class as the \a Module template
 * argument.
 */

/**
 * \typedef Algorithm::Module
 * \brief The IPA module type for this class of algorithms
 */

/**
 * \fn Algorithm::init()
 * \brief Initialize the Algorithm with tuning data
 * \param[in] context The shared IPA context
 * \param[in] tuningData The tuning data for the algorithm
 *
 * This function is called once, when the IPA module is initialized, to
 * initialize the algorithm. The \a tuningData YamlObject contains the tuning
 * data for algorithm.
 *
 * \return 0 if successful, an error code otherwise
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
 * \fn Algorithm::queueRequest()
 * \brief Provide control values to the algorithm
 * \param[in] context The shared IPA context
 * \param[in] frame The frame number to apply the control values
 * \param[in] frameContext The current frame's context
 * \param[in] controls The list of user controls
 *
 * This function is called for each request queued to the camera. It provides
 * the controls stored in the request to the algorithm. The \a frame number
 * is the Request sequence number and identifies the desired corresponding
 * frame to target for the controls to take effect.
 *
 * Algorithms shall read the applicable controls and store their value for later
 * use during frame processing.
 */

/**
 * \fn Algorithm::prepare()
 * \brief Fill the \a params buffer with ISP processing parameters for a frame
 * \param[in] context The shared IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The FrameContext for this frame
 * \param[out] params The ISP specific parameters
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
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The current frame's context
 * \param[in] stats The IPA statistics and ISP results
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * This function is called while camera is running for every frame processed by
 * the ISP, to process statistics generated from that frame by the ISP.
 * Algorithms shall use this data to run calculations, update their state
 * accordingly, and fill the frame metadata.
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

/**
 * \class AlgorithmFactory
 * \brief Registration of Algorithm classes and creation of instances
 * \tparam _Algorithm The algorithm class type for this factory
 *
 * To facilitate instantiation of Algorithm classes, the AlgorithmFactory class
 * implements auto-registration of algorithms with the IPA Module class. Each
 * Algorithm subclass shall register itself using the REGISTER_IPA_ALGORITHM()
 * macro, which will create a corresponding instance of an AlgorithmFactory and
 * register it with the IPA Module.
 */

/**
 * \fn AlgorithmFactory::AlgorithmFactory()
 * \brief Construct an algorithm factory
 * \param[in] name Name of the algorithm class
 *
 * Creating an instance of the factory automatically registers is with the IPA
 * Module class, enabling creation of algorithm instances through
 * Module::createAlgorithm().
 *
 * The factory \a name identifies the algorithm and shall be unique.
 */

/**
 * \fn AlgorithmFactory::create()
 * \brief Create an instance of the Algorithm corresponding to the factory
 * \return A pointer to a newly constructed instance of the Algorithm subclass
 * corresponding to the factory
 */

/**
 * \def REGISTER_IPA_ALGORITHM
 * \brief Register an algorithm with the IPA module
 * \param[in] algorithm Class name of Algorithm derived class to register
 * \param[in] name Name of the algorithm
 *
 * Register an Algorithm subclass with the IPA module to make it available for
 * instantiation through Module::createAlgorithm(). The \a name identifies the
 * algorithm and must be unique across all algorithms registered for the IPA
 * module.
 */

} /* namespace ipa */

} /* namespace libcamera */
