/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Camera controls
 */

#include "libcamera/internal/camera_controls.h"

#include <libcamera/camera.h>
#include <libcamera/controls.h>

/**
 * \file camera_controls.h
 * \brief Controls for Camera instances
 */

namespace libcamera {

/**
 * \class CameraControlValidator
 * \brief A control validator for Camera instances
 *
 * This ControlValidator specialisation validates that controls exist in the
 * Camera associated with the validator.
 */

/**
 * \brief Construst a CameraControlValidator for the \a camera
 * \param[in] camera The camera
 */
CameraControlValidator::CameraControlValidator(Camera *camera)
	: camera_(camera)
{
}

const std::string &CameraControlValidator::name() const
{
	return camera_->id();
}

/**
 * \brief Validate a control
 * \param[in] id The control ID
 * \return True if the control is valid, false otherwise
 */
bool CameraControlValidator::validate(unsigned int id) const
{
	const ControlInfoMap &controls = camera_->controls();
	return controls.find(id) != controls.end();
}

} /* namespace libcamera */
