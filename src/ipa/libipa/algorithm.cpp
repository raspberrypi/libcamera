/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.cpp - ISP control algorithms
 */

#include "algorithm.h"

/**
 * \file algorithm.h
 * \brief Algorithm common interface
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
 * \class Algorithm
 * \brief The base class for all IPA algorithms
 *
 * The Algorithm class defines a standard interface for IPA algorithms. By
 * abstracting algorithms, it makes possible the implementation of generic code
 * to manage algorithms regardless of their specific type.
 */

Algorithm::~Algorithm() = default;

} /* namespace ipa */

} /* namespace libcamera */
