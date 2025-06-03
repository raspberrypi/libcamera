/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Collabora Ltd.
 *
 * test_base.h - Base definitions for tests
 */

#ifndef __LC_COMPLIANCE_TEST_BASE_H__
#define __LC_COMPLIANCE_TEST_BASE_H__

#include <libcamera/libcamera.h>

#include <gtest/gtest.h>

class CameraHolder
{
protected:
	void acquireCamera();
	void releaseCamera();

	std::shared_ptr<libcamera::Camera> camera_;
};

#endif /* __LC_COMPLIANCE_TEST_BASE_H__ */
