/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Ideas On Board
 *
 * module.cpp - IPA Module
 */

#include "module.h"

/**
 * \file module.h
 * \brief IPA Module common interface
 */

namespace libcamera {

/**
 * \brief The IPA namespace
 *
 * The IPA namespace groups all types specific to IPA modules. It serves as the
 * top-level namespace for the IPA library libipa, and also contains
 * module-specific namespaces for IPA modules.
 */
namespace ipa {

/**
 * \class Module
 * \brief The base class for all IPA modules
 * \tparam Context The type of the shared IPA context
 * \tparam FrameContext The type of the frame context
 * \tparam Config The type of the IPA configuration data
 * \tparam Params The type of the ISP specific parameters
 * \tparam Stats The type of the IPA statistics and ISP results
 *
 * The Module class template defines a standard internal interface between IPA
 * modules and libipa.
 *
 * While IPA modules are platform-specific, many of their internal functions are
 * conceptually similar, even if they take different types of platform-specifc
 * parameters. For instance, IPA modules could share code that instantiates,
 * initializes and run algorithms if it wasn't for the fact that the the format
 * of ISP parameters or statistics passed to the related functions is
 * device-dependent.
 *
 * To enable a shared implementation of those common tasks in libipa, the Module
 * class template defines a standard internal interface between IPA modules and
 * libipa. The template parameters specify the types of module-dependent data.
 * IPA modules shall create a specialization of the Module class template in
 * their namespace, and use it to specialize other classes of libipa, such as
 * the Algorithm class.
 */

/**
 * \typedef Module::Context
 * \brief The type of the shared IPA context
 */

/**
 * \typedef Module::FrameContext
 * \brief The type of the frame context
 */

/**
 * \typedef Module::Config
 * \brief The type of the IPA configuration data
 */

/**
 * \typedef Module::Params
 * \brief The type of the ISP specific parameters
 */

/**
 * \typedef Module::Stats
 * \brief The type of the IPA statistics and ISP results
 */

} /* namespace ipa */

} /* namespace libcamera */
