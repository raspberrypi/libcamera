/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.cpp - Camera device
 */

#include <libcamera/camera.h>

#include "log.h"

/**
 * \file camera.h
 * \brief Camera device handling
 *
 * At the core of libcamera is the camera device, combining one image source
 * with processing hardware able to provide one or multiple image streams. The
 * Camera class represents a camera device.
 *
 * A camera device contains a single image source, and separate camera device
 * instances relate to different image sources. For instance, a phone containing
 * front and back image sensors will be modelled with two camera devices, one
 * for each sensor. When multiple streams can be produced from the same image
 * source, all those streams are guaranteed to be part of the same camera
 * device.
 *
 * While not sharing image sources, separate camera devices can share other
 * system resources, such as an ISP. For this reason camera device instances may
 * not be fully independent, in which case usage restrictions may apply. For
 * instance, a phone with a front and a back camera device may not allow usage
 * of the two devices simultaneously.
 */

namespace libcamera {

/**
 * \class Camera
 * \brief Camera device
 *
 * The Camera class models a camera capable of producing one or more image
 * streams from a single image source. It provides the main interface to
 * configuring and controlling the device, and capturing image streams. It is
 * the central object exposed by libcamera.
 */

/**
 * \brief Construct a named camera device
 *
 * \param[in] name The name to set on the camera device
 *
 * The caller is responsible for guaranteeing unicity of the camera
 * device name.
 */
Camera::Camera(const std::string &name)
	: ref_(1), name_(name)
{
}

/**
 * \brief Retrieve the name of the camera
 *
 * \return Name of the camera device
 */
const std::string &Camera::name() const
{
	return name_;
}

/**
 * \brief Acquire a reference to the camera
 */
void Camera::get()
{
	ref_++;
}

/**
 * \brief Release a reference to the camera
 *
 * When the last reference is released the camera device is deleted. Callers
 * shall not access the camera device after calling this function.
 */
void Camera::put()
{
	if (--ref_ == 0)
		delete this;
}

} /* namespace libcamera */
