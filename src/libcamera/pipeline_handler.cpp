/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * pipeline_handler.cpp - Pipeline handler infrastructure
 */

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"

/**
 * \file pipeline_handler.h
 * \brief Create pipelines and cameras from a set of media devices
 *
 * Each pipeline supported by libcamera needs to be backed by a pipeline
 * handler implementation that operate on a set of media devices. The pipeline
 * handler is responsible for matching the media devices it requires with the
 * devices present in the system, and once all those devices can be acquired,
 * create corresponding Camera instances.
 *
 * Every subclass of PipelineHandler shall be registered with libcamera using
 * the REGISTER_PIPELINE_HANDLER() macro.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Pipeline)

/**
 * \class CameraData
 * \brief Base class for platform-specific data associated with a camera
 *
 * The CameraData base abstract class represents platform specific-data
 * a pipeline handler might want to associate with a Camera to access them
 * at a later time.
 *
 * Pipeline handlers are expected to extend this base class with platform
 * specific implementation, associate instances of the derived classes
 * using the setCameraData() method, and access them at a later time
 * with cameraData().
 */

/**
 * \class PipelineHandler
 * \brief Create and manage cameras based on a set of media devices
 *
 * The PipelineHandler matches the media devices provided by a DeviceEnumerator
 * with the pipelines it supports and creates corresponding Camera devices.
 *
 * Pipeline handler instances are reference-counted through std::shared_ptr<>.
 * They implement std::enable_shared_from_this<> in order to create new
 * std::shared_ptr<> in code paths originating from member functions of the
 * PipelineHandler class where only the 'this' pointer is available.
 */

/**
 * \brief Construct a PipelineHandler instance
 * \param[in] manager The camera manager
 *
 * In order to honour the std::enable_shared_from_this<> contract,
 * PipelineHandler instances shall never be constructed manually, but always
 * through the PipelineHandlerFactory::create() method implemented by the
 * respective factories.
 */
PipelineHandler::PipelineHandler(CameraManager *manager)
	: manager_(manager)
{
}

PipelineHandler::~PipelineHandler()
{
};

/**
 * \fn PipelineHandler::match(DeviceEnumerator *enumerator)
 * \brief Match media devices and create camera instances
 * \param enumerator The enumerator providing all media devices found in the
 * system
 *
 * This function is the main entry point of the pipeline handler. It is called
 * by the camera manager with the \a enumerator passed as an argument. It shall
 * acquire from the \a enumerator all the media devices it needs for a single
 * pipeline, create one or multiple Camera instances and register them with the
 * camera manager.
 *
 * If all media devices needed by the pipeline handler are found, they must all
 * be acquired by a call to MediaDevice::acquire(). This function shall then
 * create the corresponding Camera instances, store them internally, and return
 * true. Otherwise it shall not acquire any media device (or shall release all
 * the media devices is has acquired by calling MediaDevice::release()) and
 * return false.
 *
 * If multiple instances of a pipeline are available in the system, the
 * PipelineHandler class will be instanciated once per instance, and its match()
 * function called for every instance. Each call shall acquire media devices for
 * one pipeline instance, until all compatible media devices are exhausted.
 *
 * If this function returns true, a new instance of the pipeline handler will
 * be created and its match() function called,
 *
 * \return true if media devices have been acquired and camera instances
 * created, or false otherwise
 */

/**
 * \var PipelineHandler::manager_
 * \brief The Camera manager associated with the pipeline handler
 *
 * The camera manager pointer is stored in the pipeline handler for the
 * convenience of pipeline handler implementations. It remains valid and
 * constant for the whole lifetime of the pipeline handler.
 */

/**
 * \brief Register a camera to the camera manager and pipeline handler
 * \param[in] camera The camera to be added
 *
 * This function is called by pipeline handlers to register the cameras they
 * handle with the camera manager.
 */
void PipelineHandler::registerCamera(std::shared_ptr<Camera> camera)
{
	cameras_.push_back(camera);
	manager_->addCamera(std::move(camera));
}

/**
 * \brief Enable hotplug handling for a media device
 * \param[in] media The media device
 *
 * This function enables hotplug handling, and especially hot-unplug handling,
 * of the \a media device. It shall be called by pipeline handlers for all the
 * media devices that can be disconnected.
 *
 * When a media device passed to this function is later unplugged, the pipeline
 * handler gets notified and automatically disconnects all the cameras it has
 * registered without requiring any manual intervention.
 */
void PipelineHandler::hotplugMediaDevice(MediaDevice *media)
{
	media->disconnected.connect(this, &PipelineHandler::mediaDeviceDisconnected);
}

/**
 * \brief Device disconnection handler
 *
 * This virtual function is called to notify the pipeline handler that the
 * device it handles has been disconnected. It notifies all cameras created by
 * the pipeline handler that they have been disconnected, and unregisters them
 * from the camera manager.
 *
 * The function can be overloaded by pipeline handlers to perform custom
 * operations at disconnection time. Any overloaded version shall call the
 * PipelineHandler::disconnect() base function for proper hot-unplug operation.
 */
void PipelineHandler::disconnect()
{
	for (std::weak_ptr<Camera> ptr : cameras_) {
		std::shared_ptr<Camera> camera = ptr.lock();
		if (!camera)
			continue;

		camera->disconnect();
		manager_->removeCamera(camera.get());
	}

	cameras_.clear();
}

/**
 * \brief Slot for the MediaDevice disconnected signal
 */
void PipelineHandler::mediaDeviceDisconnected(MediaDevice *media)
{
	if (cameras_.empty())
		return;

	disconnect();
}

/**
 * \brief Retrieve the pipeline-specific data associated with a Camera
 * \param camera The camera data is associate with
 *
 * \return A pointer to the pipeline-specific data set with setCameraData().
 * The returned pointer lifetime is associated with the one of the pipeline
 * handler, and caller of this function shall never release it manually.
 */
CameraData *PipelineHandler::cameraData(const Camera *camera)
{
	if (!cameraData_.count(camera)) {
		LOG(Pipeline, Error)
			<< "Cannot get data associated with camera "
			<< camera->name();
		return nullptr;
	}

	return cameraData_[camera].get();
}

/**
 * \brief Set pipeline-specific data in the camera
 * \param camera The camera to associate data to
 * \param data The pipeline-specific data
 *
 * This method allows pipeline handlers to associate pipeline-specific
 * information with \a camera. The \a data lifetime gets associated with
 * the pipeline handler one, and gets released at deletion time.
 *
 * If pipeline-specific data has already been associated with the camera by a
 * previous call to this method, is it replaced by \a data and the previous data
 * are deleted, rendering all references to them invalid.
 *
 * The data can be retrieved by pipeline handlers using the cameraData() method.
 */
void PipelineHandler::setCameraData(const Camera *camera,
				    std::unique_ptr<CameraData> data)
{
	if (cameraData_.count(camera))
		LOG(Pipeline, Debug)
			<< "Replacing data associated with "
			<< camera->name();

	cameraData_[camera] = std::move(data);
}

/**
 * \class PipelineHandlerFactory
 * \brief Registration of PipelineHandler classes and creation of instances
 *
 * To facilitate discovery and instantiation of PipelineHandler classes, the
 * PipelineHandlerFactory class maintains a registry of pipeline handler
 * classes. Each PipelineHandler subclass shall register itself using the
 * REGISTER_PIPELINE_HANDLER() macro, which will create a corresponding
 * instance of a PipelineHandlerFactory subclass and register it with the
 * static list of factories.
 */

/**
 * \brief Construct a pipeline handler factory
 * \param[in] name Name of the pipeline handler class
 *
 * Creating an instance of the factory registers is with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used for debug purpose and shall be unique.
 */
PipelineHandlerFactory::PipelineHandlerFactory(const char *name)
	: name_(name)
{
	registerType(this);
}

/**
 * \fn PipelineHandlerFactory::create()
 * \brief Create an instance of the PipelineHandler corresponding to the factory
 * \param[in] manager The camera manager
 *
 * This virtual function is implemented by the REGISTER_PIPELINE_HANDLER()
 * macro. It creates a pipeline handler instance associated with the camera
 * \a manager.
 *
 * \return a pointer to a newly constructed instance of the PipelineHandler
 * subclass corresponding to the factory
 */

/**
 * \fn PipelineHandlerFactory::name()
 * \brief Retrieve the factory name
 * \return The factory name
 */

/**
 * \brief Add a pipeline handler class to the registry
 * \param[in] factory Factory to use to construct the pipeline handler
 *
 * The caller is responsible to guarantee the uniqueness of the pipeline handler
 * name.
 */
void PipelineHandlerFactory::registerType(PipelineHandlerFactory *factory)
{
	std::vector<PipelineHandlerFactory *> &factories = PipelineHandlerFactory::factories();

	factories.push_back(factory);

	LOG(Pipeline, Debug)
		<< "Registered pipeline handler \"" << factory->name() << "\"";
}

/**
 * \brief Retrieve the list of all pipeline handler factories
 *
 * The static factories map is defined inside the function to ensures it gets
 * initialized on first use, without any dependency on link order.
 *
 * \return the list of pipeline handler factories
 */
std::vector<PipelineHandlerFactory *> &PipelineHandlerFactory::factories()
{
	static std::vector<PipelineHandlerFactory *> factories;
	return factories;
}

/**
 * \def REGISTER_PIPELINE_HANDLER
 * \brief Register a pipeline handler with the pipeline handler factory
 * \param[in] handler Class name of PipelineHandler derived class to register
 *
 * Register a PipelineHandler subclass with the factory and make it available to
 * try and match devices.
 */

} /* namespace libcamera */
