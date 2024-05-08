/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Camera controls
 */

#pragma once

#include "libcamera/internal/control_validator.h"

namespace libcamera {

class Camera;

class CameraControlValidator final : public ControlValidator
{
public:
	CameraControlValidator(Camera *camera);

	const std::string &name() const override;
	bool validate(unsigned int id) const override;

private:
	Camera *camera_;
};

} /* namespace libcamera */
