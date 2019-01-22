/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.cpp - Camera device
 */

#include <libcamera/camera.h>

#include "log.h"
#include "pipeline_handler.h"

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
 *
 * To support the central nature of Camera objects, libcamera manages the
 * lifetime of camera instances with std::shared_ptr<>. Instances shall be
 * created with the create() function which returns a shared pointer. The
 * Camera constructors and destructor are private, to prevent instances from
 * being constructed and destroyed manually.
 */

/**
 * \brief Create a camera instance
 * \param[in] name The name of the camera device
 * \param[in] pipe The pipeline handler responsible for the camera device
 *
 * The caller is responsible for guaranteeing unicity of the camera name.
 *
 * \return A shared pointer to the newly created camera object
 */
std::shared_ptr<Camera> Camera::create(PipelineHandler *pipe,
				       const std::string &name)
{
	struct Allocator : std::allocator<Camera> {
		void construct(void *p, PipelineHandler *pipe,
			       const std::string &name)
		{
			::new(p) Camera(pipe, name);
		}
		void destroy(Camera *p)
		{
			p->~Camera();
		}
	};

	return std::allocate_shared<Camera>(Allocator(), pipe, name);
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

Camera::Camera(PipelineHandler *pipe, const std::string &name)
	: pipe_(pipe->shared_from_this()), name_(name)
{
}

Camera::~Camera()
{
}

} /* namespace libcamera */
