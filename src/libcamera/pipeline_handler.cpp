/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * pipeline_handler.cpp - Pipeline handler infrastructure
 */

#include "device_enumerator.h"
#include "log.h"
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

/**
 * \class PipelineHandler
 * \brief Create and manage cameras based on a set of media devices
 *
 * The PipelineHandler matches the media devices provided by a DeviceEnumerator
 * with the pipelines it supports and creates corresponding Camera devices.
 */

/**
 * \fn PipelineHandler::match(DeviceEnumerator *enumerator)
 * \brief Match media devices and create camera instances
 * \param enumerator The enumerator providing all media devices found in the
 * system
 *
 * This function is the main entry point of the pipeline handler. It is called
 * by the device enumerator with the \a enumerator passed as an argument. It
 * shall acquire from the \a enumerator all the media devices it needs for a
 * single pipeline and create one or multiple Camera instances.
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
 * \fn PipelineHandler::count()
 * \brief Retrieve the number of cameras handled by this pipeline handler
 * \return the number of cameras that were created by the match() function
 */

/**
 * \fn PipelineHandler::camera(unsigned int id)
 * \brief Retrieve one of the cameras handled by this pipeline handler
 * \param[in] id the camera index
 * \return a pointer to the Camera identified by \a id
 */

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
 * \fn PipelineHandlerFactory::create()
 * \brief Create an instance of the PipelineHandler corresponding to the factory
 *
 * This virtual function is implemented by the REGISTER_PIPELINE_HANDLER() macro.
 *
 * \return a pointer to a newly constructed instance of the PipelineHandler
 * subclass corresponding to the factory
 */

/**
 * \brief Add a pipeline handler class to the registry
 * \param[in] name Name of the pipeline handler class
 * \param[in] factory Factory to use to construct the pipeline handler
 *
 * The caller is responsible to guarantee the uniqueness of the pipeline handler
 * name.
 */
void PipelineHandlerFactory::registerType(const std::string &name,
					  PipelineHandlerFactory *factory)
{
	std::map<std::string, PipelineHandlerFactory *> &factories = registry();

	if (factories.count(name)) {
		LOG(Error) <<  "Registering '" << name << "' pipeline twice";
		return;
	}

	factories[name] = factory;
}

/**
 * \brief Create an instance of a pipeline handler if it matches media devices
 * present in the system
 * \param[in] name Name of the pipeline handler to instantiate
 * \param[in] enumerator Device enumerator to search for a match for the handler
 *
 * This function matches the media devices required by pipeline \a name against
 * the devices enumerated by \a enumerator.
 *
 * \return the newly created pipeline handler instance if a match was found, or
 * nullptr otherwise
 */
PipelineHandler *PipelineHandlerFactory::create(const std::string &name,
						DeviceEnumerator *enumerator)
{
	std::map<std::string, PipelineHandlerFactory *> &factories = registry();

	auto it = factories.find(name);
	if (it == factories.end()) {
		LOG(Error) << "Trying to create non-existing pipeline handler "
			   << name;
		return nullptr;
	}

	PipelineHandler *pipe = it->second->create();

	if (pipe->match(enumerator))
		return pipe;

	delete pipe;
	return nullptr;
}

/**
 * \brief Retrieve the names of all pipeline handlers registered with the
 * factory
 *
 * \return a list of all registered pipeline handler names
 */
std::vector<std::string> PipelineHandlerFactory::handlers()
{
	std::map<std::string, PipelineHandlerFactory *> &factories = registry();
	std::vector<std::string> handlers;

	for (auto const &handler : factories)
		handlers.push_back(handler.first);

	return handlers;
}

/**
 * \brief Retrieve the list of all pipeline handler factories
 *
 * The static factories map is defined inside the function to ensures it gets
 * initialized on first use, without any dependency on link order.
 *
 * \return the list of pipeline handler factories
 */
std::map<std::string, PipelineHandlerFactory *> &PipelineHandlerFactory::registry()
{
	static std::map<std::string, PipelineHandlerFactory *> factories;
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
