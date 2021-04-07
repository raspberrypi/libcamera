/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_validator.cpp - Control validator
 */

#include "libcamera/internal/control_validator.h"

/**
 * \file control_validator.h
 * \brief Abstract control validator
 */

namespace libcamera {

/**
 * \class ControlValidator
 * \brief Interface for the control validator
 *
 * The ControlValidator class is used by the ControlList class to validate
 * controls added to the list. It is an abstract class providing an interface
 * for object-specific control validation, such a Camera controls and V4L2
 * controls.
 */

/**
 * \fn ControlValidator::name()
 * \brief Retrieve the name of the object associated with the validator
 * \return The name of the object associated with the validator
 */

/**
 * \fn ControlValidator::validate()
 * \brief Validate a control
 * \param[in] id The control ID
 *
 * This function validates the control \a id against the object corresponding to
 * the validator. It shall at least validate that the control is applicable to
 * the object instance, and may perform additional checks.
 *
 * \return True if the control is valid, false otherwise
 */

} /* namespace libcamera */
