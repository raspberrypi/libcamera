/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * tests.h - Test modules
 */
#ifndef __LC_COMPLIANCE_TESTS_H__
#define __LC_COMPLIANCE_TESTS_H__

#include <libcamera/libcamera.h>

#include "results.h"

Results testSingleStream(std::shared_ptr<libcamera::Camera> camera);

#endif /* __LC_COMPLIANCE_TESTS_H__ */
