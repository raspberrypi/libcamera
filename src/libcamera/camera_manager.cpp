/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera_manager.h - Camera management
 */

#include <libcamera/camera_manager.h>

#include <libcamera/camera.h>
#include <libcamera/event_dispatcher.h>

#include "device_enumerator.h"
#include "event_dispatcher_poll.h"
#include "log.h"
#include "pipeline_handler.h"
#include "utils.h"

/**
 * \file camera_manager.h
 * \brief The camera manager
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Camera)

/**
 * \class CameraManager
 * \brief Provide access and manage all cameras in the system
 *
 * The camera manager is the entry point to libcamera. It enumerates devices,
 * associates them with pipeline managers, and provides access to the cameras
 * in the system to applications. The manager owns all Camera objects and
 * handles hot-plugging and hot-unplugging to manage the lifetime of cameras.
 *
 * To interact with libcamera, an application retrieves the camera manager
 * instance with CameraManager::instance(). The manager is initially stopped,
 * and shall be configured before being started. In particular a custom event
 * dispatcher shall be installed if needed with
 * CameraManager::setEventDispatcher().
 *
 * Once the camera manager is configured, it shall be started with start().
 * This will enumerate all the cameras present in the system, which can then be
 * listed with list() and retrieved with get().
 *
 * Cameras are shared through std::shared_ptr<>, ensuring that a camera will
 * stay valid until the last reference is released without requiring any special
 * action from the application. Once the application has released all the
 * references it held to cameras, the camera manager can be stopped with
 * stop().
 *
 * \todo Add interface to register a notification callback to the user to be
 * able to inform it new cameras have been hot-plugged or cameras have been
 * removed due to hot-unplug.
 */

CameraManager::CameraManager()
	: enumerator_(nullptr), dispatcher_(nullptr)
{
}

CameraManager::~CameraManager()
{
}

/**
 * \brief Start the camera manager
 *
 * Start the camera manager and enumerate all devices in the system. Once
 * the start has been confirmed the user is free to list and otherwise
 * interact with cameras in the system until either the camera manager
 * is stopped or the camera is unplugged from the system.
 *
 * \return 0 on success or a negative error code otherwise
 */
int CameraManager::start()
{
	if (enumerator_)
		return -EBUSY;

	enumerator_ = DeviceEnumerator::create();
	if (!enumerator_ || enumerator_->enumerate())
		return -ENODEV;

	/*
	 * TODO: Try to read handlers and order from configuration
	 * file and only fallback on all handlers if there is no
	 * configuration file.
	 */
	std::vector<PipelineHandlerFactory *> &factories = PipelineHandlerFactory::factories();

	for (PipelineHandlerFactory *factory : factories) {
		/*
		 * Try each pipeline handler until it exhaust
		 * all pipelines it can provide.
		 */
		while (1) {
			std::shared_ptr<PipelineHandler> pipe = factory->create(this);
			if (!pipe->match(enumerator_.get()))
				break;

			LOG(Camera, Debug)
				<< "Pipeline handler \"" << factory->name()
				<< "\" matched";
			pipes_.push_back(std::move(pipe));
		}
	}

	/* TODO: register hot-plug callback here */

	return 0;
}

/**
 * \brief Stop the camera manager
 *
 * Before stopping the camera manager the caller is responsible for making
 * sure all cameras provided by the manager are returned to the manager.
 *
 * After the manager has been stopped no resource provided by the camera
 * manager should be consider valid or functional even if they for one
 * reason or another have yet to be deleted.
 */
void CameraManager::stop()
{
	/* TODO: unregister hot-plug callback here */

	/*
	 * Release all references to cameras and pipeline handlers to ensure
	 * they all get destroyed before the device enumerator deletes the
	 * media devices.
	 */
	pipes_.clear();
	cameras_.clear();

	enumerator_.reset(nullptr);
}

/**
 * \fn CameraManager::cameras()
 * \brief Retrieve all available cameras
 *
 * Before calling this function the caller is responsible for ensuring that
 * the camera manger is running.
 *
 * \return List of all available cameras
 */

/**
 * \brief Get a camera based on name
 * \param[in] name Name of camera to get
 *
 * Before calling this function the caller is responsible for ensuring that
 * the camera manger is running.
 *
 * \return Shared pointer to Camera object or nullptr if camera not found
 */
std::shared_ptr<Camera> CameraManager::get(const std::string &name)
{
	for (std::shared_ptr<Camera> camera : cameras_) {
		if (camera->name() == name)
			return camera;
	}

	return nullptr;
}

/**
 * \brief Add a camera to the camera manager
 * \param[in] camera The camera to be added
 *
 * This function is called by pipeline handlers to register the cameras they
 * handle with the camera manager. Registered cameras are immediately made
 * available to the system.
 */
void CameraManager::addCamera(std::shared_ptr<Camera> camera)
{
	for (std::shared_ptr<Camera> c : cameras_) {
		if (c->name() == camera->name()) {
			LOG(Camera, Warning)
				<< "Registering camera with duplicate name '"
				<< camera->name() << "'";
			break;
		}
	}

	cameras_.push_back(std::move(camera));
}

/**
 * \brief Remove a camera from the camera manager
 * \param[in] camera The camera to be removed
 *
 * This function is called by pipeline handlers to unregister cameras from the
 * camera manager. Unregistered cameras won't be reported anymore by the
 * cameras() and get() calls, but references may still exist in applications.
 */
void CameraManager::removeCamera(Camera *camera)
{
	for (auto iter = cameras_.begin(); iter != cameras_.end(); ++iter) {
		if (iter->get() == camera) {
			LOG(Camera, Debug)
				<< "Unregistering camera '"
				<< camera->name() << "'";
			cameras_.erase(iter);
			return;
		}
	}
}

/**
 * \brief Retrieve the camera manager instance
 *
 * The CameraManager is a singleton and can't be constructed manually. This
 * function shall instead be used to retrieve the single global instance of the
 * manager.
 *
 * \return The camera manager instance
 */
CameraManager *CameraManager::instance()
{
	static CameraManager manager;
	return &manager;
}

/**
 * \brief Set the event dispatcher
 * \param[in] dispatcher Pointer to the event dispatcher
 *
 * libcamera requires an event dispatcher to integrate event notification and
 * timers with the application event loop. Applications that want to provide
 * their own event dispatcher shall call this function once and only once before
 * the camera manager is started with start(). If no event dispatcher is
 * provided, a default poll-based implementation will be used.
 *
 * The CameraManager takes ownership of the event dispatcher and will delete it
 * when the application terminates.
 */
void CameraManager::setEventDispatcher(std::unique_ptr<EventDispatcher> dispatcher)
{
	if (dispatcher_) {
		LOG(Camera, Warning) << "Event dispatcher is already set";
		return;
	}

	dispatcher_ = std::move(dispatcher);
}

/**
 * \brief Retrieve the event dispatcher
 *
 * This function retrieves the event dispatcher set with setEventDispatcher().
 * If no dispatcher has been set, a default poll-based implementation is created
 * and returned, and no custom event dispatcher may be installed anymore.
 *
 * The returned event dispatcher is valid until the camera manager is destroyed.
 *
 * \return Pointer to the event dispatcher
 */
EventDispatcher *CameraManager::eventDispatcher()
{
	if (!dispatcher_)
		dispatcher_ = utils::make_unique<EventDispatcherPoll>();

	return dispatcher_.get();
}

} /* namespace libcamera */
