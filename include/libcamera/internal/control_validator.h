/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_validator.h - Control validator
 */

#pragma once

#include <string>

namespace libcamera {

class ControlId;

class ControlValidator
{
public:
	virtual ~ControlValidator() = default;

	virtual const std::string &name() const = 0;
	virtual bool validate(unsigned int id) const = 0;
};

} /* namespace libcamera */
