/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_validator.h - Control validator
 */
#ifndef __LIBCAMERA_INTERNAL_CONTROL_VALIDATOR_H__
#define __LIBCAMERA_INTERNAL_CONTROL_VALIDATOR_H__

#include <string>

namespace libcamera {

class ControlId;

class ControlValidator
{
public:
	virtual ~ControlValidator() {}

	virtual const std::string &name() const = 0;
	virtual bool validate(unsigned int id) const = 0;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_CONTROL_VALIDATOR_H__ */
