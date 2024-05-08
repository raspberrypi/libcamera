/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Public API validation
 */

#include <libcamera/libcamera.h>

#include "test.h"

class PublicAPITest : public Test
{
	int run()
	{
#ifdef LIBCAMERA_BASE_PRIVATE
#error "Public interfaces should not be exposed to LIBCAMERA_BASE_PRIVATE"
		return TestFail;
#else
		return TestPass;
#endif
	}
};

TEST_REGISTER(PublicAPITest)
