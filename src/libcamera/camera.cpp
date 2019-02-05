/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.cpp - Camera device
 */

#include <libcamera/camera.h>
#include <libcamera/stream.h>

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

LOG_DECLARE_CATEGORY(Camera)

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
 * \param[in] streams Array of streams the camera provides
 *
 * The caller is responsible for guaranteeing unicity of the camera name.
 *
 * \return A shared pointer to the newly created camera object
 */
std::shared_ptr<Camera> Camera::create(PipelineHandler *pipe,
				       const std::string &name,
				       const std::vector<Stream *> &streams)
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

	std::shared_ptr<Camera> camera =
		std::allocate_shared<Camera>(Allocator(), pipe, name);

	camera->streams_ = streams;

	return camera;
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
 * \var Camera::requestCompleted
 * \brief Signal emitted when a request queued to the camera has completed
 */

/**
 * \var Camera::disconnected
 * \brief Signal emitted when the camera is disconnected from the system
 *
 * This signal is emitted when libcamera detects that the camera has been
 * removed from the system. For hot-pluggable devices this is usually caused by
 * physical device disconnection. The media device is passed as a parameter.
 *
 * As soon as this signal is emitted the camera instance will refuse all new
 * application API calls by returning errors immediately.
 */

Camera::Camera(PipelineHandler *pipe, const std::string &name)
	: pipe_(pipe->shared_from_this()), name_(name), acquired_(false),
	  disconnected_(false)
{
}

Camera::~Camera()
{
	if (acquired_)
		LOG(Camera, Error) << "Removing camera while still in use";
}

/**
 * \brief Notify camera disconnection
 *
 * This method is used to notify the camera instance that the underlying
 * hardware has been unplugged. In response to the disconnection the camera
 * instance notifies the application by emitting the #disconnected signal, and
 * ensures that all new calls to the application-facing Camera API return an
 * error immediately.
 */
void Camera::disconnect()
{
	LOG(Camera, Debug) << "Disconnecting camera " << name_;

	disconnected_ = true;
	disconnected.emit(this);
}

/**
 * \brief Acquire the camera device for exclusive access
 *
 * After opening the device with open(), exclusive access must be obtained
 * before performing operations that change the device state. This function is
 * not blocking, if the device has already been acquired (by the same or another
 * process) the -EBUSY error code is returned.
 *
 * Once exclusive access isn't needed anymore, the device should be released
 * with a call to the release() function.
 *
 * \todo Implement exclusive access across processes.
 *
 * \return 0 on success or a negative error code on error.
 */
int Camera::acquire()
{
	if (acquired_)
		return -EBUSY;

	acquired_ = true;
	return 0;
}

/**
 * \brief Release exclusive access to the camera device
 *
 * Releasing the camera device allows other users to acquire exclusive access
 * with the acquire() function.
 */
void Camera::release()
{
	acquired_ = false;
}

/**
 * \brief Retrieve all the camera's stream information
 *
 * Retrieve all of the camera's static stream information. The static
 * information describes among other things how many streams the camera
 * supports and the capabilities of each stream.
 *
 * \return An array of all the camera's streams.
 */
const std::vector<Stream *> &Camera::streams() const
{
	return streams_;
}

/**
 * \brief Retrieve a group of stream configurations
 * \param[in] streams A map of stream IDs and configurations to setup
 *
 * Retrieve the camera's configuration for a specified group of streams. The
 * caller can specifies which of the camera's streams to retrieve configuration
 * from by populating \a streams.
 *
 * The easiest way to populate the array of streams to fetch configuration from
 * is to first retrieve the camera's full array of stream with streams() and
 * then potentially trim it down to only contain the streams the caller
 * are interested in.
 *
 * \return A map of successfully retrieved stream IDs and configurations or an
 * empty list on error.
 */
std::map<Stream *, StreamConfiguration>
Camera::streamConfiguration(std::vector<Stream *> &streams)
{
	if (disconnected_ || !streams.size())
		std::map<unsigned int, StreamConfiguration> {};

	return pipe_->streamConfiguration(this, streams);
}

/**
 * \brief Configure the camera's streams prior to capture
 * \param[in] config A map of stream IDs and configurations to setup
 *
 * Prior to starting capture, the camera must be configured to select a
 * group of streams to be involved in the capture and their configuration.
 * The caller specifies which streams are to be involved and their configuration
 * by populating \a config.
 *
 * The easiest way to populate the array of config is to fetch an initial
 * configuration from the camera with streamConfiguration() and then change the
 * parameters to fit the caller's need and once all the streams parameters are
 * configured hand that over to configureStreams() to actually setup the camera.
 *
 * Exclusive access to the camera shall be ensured by a call to acquire() prior
 * to calling this function, otherwise an -EACCES error will be returned.
 *
 * \return 0 on success or a negative error code on error.
 * \retval -ENODEV The camera is not connected to any hardware
 * \retval -EACCES The user has not acquired exclusive access to the camera
 * \retval -EINVAL The configuration is not valid
 */
int Camera::configureStreams(std::map<Stream *, StreamConfiguration> &config)
{
	int ret;

	ret = exclusiveAccess();
	if (ret)
		return ret;

	if (!config.size()) {
		LOG(Camera, Error)
			<< "Can't configure streams without a configuration";
		return -EINVAL;
	}

	ret = pipe_->configureStreams(this, config);
	if (ret)
		return ret;

	activeStreams_.clear();
	for (auto const &iter : config) {
		Stream *stream = iter.first;
		const StreamConfiguration &cfg = iter.second;

		stream->configuration_ = cfg;
		activeStreams_.push_back(stream);
	}

	return 0;
}

int Camera::exclusiveAccess()
{
	if (disconnected_)
		return -ENODEV;

	if (!acquired_)
		return -EACCES;

	return 0;
}

} /* namespace libcamera */
