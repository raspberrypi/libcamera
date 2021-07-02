/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Collabora Ltd.
 *
 * environment.h - Common environment for tests
 */
#ifndef __LC_COMPLIANCE_ENVIRONMENT_H__
#define __LC_COMPLIANCE_ENVIRONMENT_H__

#include <libcamera/libcamera.h>

using namespace libcamera;

class Environment
{
public:
	static Environment *get();

	void setup(CameraManager *cm, std::string cameraId);

	const std::string &cameraId() const { return cameraId_; }
	CameraManager *cm() const { return cm_; }

private:
	Environment() = default;

	std::string cameraId_;
	CameraManager *cm_;
};

#endif /* __LC_COMPLIANCE_ENVIRONMENT_H__ */
