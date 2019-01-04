/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera_manager.h - Camera management
 */

#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>

#include "device_enumerator.h"
#include "event_dispatcher_poll.h"
#include "log.h"
#include "pipeline_handler.h"

/**
 * \file camera_manager.h
 * \brief Manage all cameras handled by libcamera
 *
 * The responsibility of the camera manager is to control the lifetime
 * management of objects provided by libcamera.
 *
 * When a user wish to interact with libcamera it creates and starts a
 * CameraManager object. Once confirmed the camera manager is running
 * the application can list all cameras detected by the library, get
 * one or more of the cameras and interact with them.
 *
 * When the user is done with the camera it should be returned to the
 * camera manager. Once all cameras are returned to the camera manager
 * the user is free to stop the manager.
 *
 * \todo Add ability to add and remove media devices based on
 *       hot-(un)plug events coming from the device enumerator.
 *
 * \todo Add interface to register a notification callback to the user
 *       to be able to inform it new cameras have been hot-plugged or
 *       cameras have been removed due to hot-unplug.
 */

namespace libcamera {

CameraManager::CameraManager()
	: enumerator_(nullptr), dispatcher_(nullptr)
{
}

CameraManager::~CameraManager()
{
	delete dispatcher_;
}

/**
 * \brief Start the camera manager
 *
 * Start the camera manager and enumerate all devices in the system. Once
 * the start has been confirmed the user is free to list and otherwise
 * interact with cameras in the system until either the camera manager
 * is stopped or the camera is unplugged from the system.
 *
 * \return true on successful start false otherwise
 */
int CameraManager::start()
{

	if (enumerator_)
		return -ENODEV;

	enumerator_ = DeviceEnumerator::create();
	if (enumerator_->enumerate())
		return -ENODEV;

	/*
	 * TODO: Try to read handlers and order from configuration
	 * file and only fallback on all handlers if there is no
	 * configuration file.
	 */
	std::vector<std::string> handlers = PipelineHandlerFactory::handlers();

	for (std::string const &handler : handlers) {
		PipelineHandler *pipe;

		/*
		 * Try each pipeline handler until it exhaust
		 * all pipelines it can provide.
		 */
		do {
			pipe = PipelineHandlerFactory::create(handler, enumerator_);
			if (pipe)
				pipes_.push_back(pipe);
		} while (pipe);
	}

	/* TODO: register hot-plug callback here */

	return 0;
}

/**
 * \brief Stop the camera manager
 *
 * Before stopping the camera manger the caller is responsible for making
 * sure all cameras provided by the manager are returned to the manager.
 *
 * After the manager has been stopped no resource provided by the camera
 * manager should be consider valid or functional even if they for one
 * reason or another have yet to be deleted.
 */
void CameraManager::stop()
{
	/* TODO: unregister hot-plug callback here */

	for (PipelineHandler *pipe : pipes_)
		delete pipe;

	pipes_.clear();

	if (enumerator_)
		delete enumerator_;

	enumerator_ = nullptr;
}

/**
 * \brief List all detected cameras
 *
 * Before calling this function the caller is responsible for ensuring that
 * the camera manger is running.
 *
 * \return List of names for all detected cameras
 */
std::vector<std::string> CameraManager::list() const
{
	std::vector<std::string> list;

	for (PipelineHandler *pipe : pipes_) {
		for (unsigned int i = 0; i < pipe->count(); i++) {
			Camera *cam = pipe->camera(i);
			list.push_back(cam->name());
		}
	}

	return list;
}

/**
 * \brief Get a camera based on name
 *
 * \param[in] name Name of camera to get
 *
 * Before calling this function the caller is responsible for ensuring that
 * the camera manger is running. A camera fetched this way shall be
 * released by the user with the put() method of the Camera object once
 * it is done using the camera.
 *
 * \return Pointer to Camera object or nullptr if camera not found
 */
Camera *CameraManager::get(const std::string &name)
{
	for (PipelineHandler *pipe : pipes_) {
		for (unsigned int i = 0; i < pipe->count(); i++) {
			Camera *cam = pipe->camera(i);
			if (cam->name() == name) {
				cam->get();
				return cam;
			}
		}
	}

	return nullptr;
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
 * \param dispatcher Pointer to the event dispatcher
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
void CameraManager::setEventDispatcher(EventDispatcher *dispatcher)
{
	if (dispatcher_) {
		LOG(Warning) << "Event dispatcher is already set";
		return;
	}

	dispatcher_ = dispatcher;
}

/**
 * \brief Retrieve the event dispatcher
 *
 * This function retrieves the event dispatcher set with setEventDispatcher().
 * If no dispatcher has been set, a default poll-based implementation is created
 * and returned, and no custom event dispatcher may be installed anymore.
 *
 * \return Pointer to the event dispatcher
 */
EventDispatcher *CameraManager::eventDispatcher()
{
	if (!dispatcher_)
		dispatcher_ = new EventDispatcherPoll();

	return dispatcher_;
}

} /* namespace libcamera */
