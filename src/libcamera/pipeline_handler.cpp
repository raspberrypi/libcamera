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
 * \brief Create pipelines and cameras from one or more media devices
 *
 * Each pipeline supported by libcamera needs to be backed by a pipeline
 * handler implementation which describes the one or many media devices
 * needed for a pipeline to function properly.
 *
 * The pipeline handler is responsible for providing a description of the
 * media devices it requires to operate. Once all media devices can be
 * provided the pipeline handler can acquire them and create camera
 * devices that utilize the acquired media devices.
 *
 * To make it a bit less bit complicated to write pipe line handlers a
 * macro REGISTER_PIPELINE_HANDLER() is provided which allows a pipeline
 * handler implementation to register itself with the library with ease.
 */

namespace libcamera {

/**
 * \class PipelineHandler
 * \brief Find a set of media devices and provide cameras
 *
 * The responsibility of a PipelineHandler is to describe all media
 * devices it would need in order to provide cameras to the system.
 */

/**
 * \class PipelineHandlerFactory
 * \brief Keep a registry and create instances of available pipeline handlers
 *
 * The responsibility of the PipelineHandlerFactory is to keep a list
 * of all pipelines in the system. Each pipeline handler should register
 * it self with the factory using the REGISTER_PIPELINE_HANDLER() macro.
 */

/**
 * \brief Add a pipeline handler to the global list
 *
 * \param[in] name Name of the pipeline handler to add
 * \param[in] factory Factory to use to construct the pipeline
 *
 * The caller is responsible to guarantee the uniqueness of the pipeline name.
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
 * \brief Create a new pipeline handler and try to match the media devices it requires
 *
 * \param[in] name Name of the pipeline handler to try
 * \param[in] enumerator Device enumerator to search for a match for the handler
 *
 * Try to match the media devices pipeline \a name requires against the ones
 * registered in \a enumerator.
 *
 * \return Pipeline handler if a match was found, nullptr otherwise
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
 * \brief List all names of piepline handlers from the global list
 *
 * \return List of registerd pipeline handler names
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
 * \brief Static global list of pipeline handlers
 *
 * The static factories map is defined inside the function to ensures it gets
 * initialized on first use, without any dependency on link order.
 *
 * \return Global list of pipeline handlers
 */
std::map<std::string, PipelineHandlerFactory *> &PipelineHandlerFactory::registry()
{
	static std::map<std::string, PipelineHandlerFactory *> factories;
	return factories;
}

/**
 * \def REGISTER_PIPELINE_HANDLER
 * \brief Register a pipeline handler with the global list
 *
 * \param[in] handler Class name of PipelineHandler derived class to register
 *
 * Register a specific pipeline handler with the global list and make it
 * available to try and match devices.
 */

} /* namespace libcamera */
