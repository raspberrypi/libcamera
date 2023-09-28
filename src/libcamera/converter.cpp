/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2022 NXP
 *
 * converter.cpp - Generic format converter interface
 */

#include "libcamera/internal/converter.h"

#include <algorithm>

#include <libcamera/base/log.h>

#include "libcamera/internal/media_device.h"

/**
 * \file internal/converter.h
 * \brief Abstract converter
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Converter)

/**
 * \class Converter
 * \brief Abstract Base Class for converter
 *
 * The Converter class is an Abstract Base Class defining the interfaces of
 * converter implementations.
 *
 * Converters offer scaling and pixel format conversion services on an input
 * stream. The converter can output multiple streams with individual conversion
 * parameters from the same input stream.
 */

/**
 * \brief Construct a Converter instance
 * \param[in] media The media device implementing the converter
 *
 * This searches for the entity implementing the data streaming function in the
 * media graph entities and use its device node as the converter device node.
 */
Converter::Converter(MediaDevice *media)
{
	const std::vector<MediaEntity *> &entities = media->entities();
	auto it = std::find_if(entities.begin(), entities.end(),
			       [](MediaEntity *entity) {
				       return entity->function() == MEDIA_ENT_F_IO_V4L;
			       });
	if (it == entities.end()) {
		LOG(Converter, Error)
			<< "No entity suitable for implementing a converter in "
			<< media->driver() << " entities list.";
		return;
	}

	deviceNode_ = (*it)->deviceNode();
}

Converter::~Converter()
{
}

/**
 * \fn Converter::loadConfiguration()
 * \brief Load converter configuration from file
 * \param[in] filename The file name path
 *
 * Load converter dependent configuration parameters to apply on the hardware.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn Converter::isValid()
 * \brief Check if the converter configuration is valid
 * \return True is the converter is valid, false otherwise
 */

/**
 * \fn Converter::formats()
 * \brief Retrieve the list of supported pixel formats for an input pixel format
 * \param[in] input Input pixel format to retrieve output pixel format list for
 * \return The list of supported output pixel formats
 */

/**
 * \fn Converter::sizes()
 * \brief Retrieve the range of minimum and maximum output sizes for an input size
 * \param[in] input Input stream size to retrieve range for
 * \return A range of output image sizes
 */

/**
 * \fn Converter::strideAndFrameSize()
 * \brief Retrieve the output stride and frame size for an input configutation
 * \param[in] pixelFormat Input stream pixel format
 * \param[in] size Input stream size
 * \return A tuple indicating the stride and frame size or an empty tuple on error
 */

/**
 * \fn Converter::configure()
 * \brief Configure a set of output stream conversion from an input stream
 * \param[in] inputCfg Input stream configuration
 * \param[out] outputCfgs A list of output stream configurations
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn Converter::exportBuffers()
 * \brief Export buffers from the converter device
 * \param[in] output Output stream index exporting the buffers
 * \param[in] count Number of buffers to allocate
 * \param[out] buffers Vector to store allocated buffers
 *
 * This function operates similarly to V4L2VideoDevice::exportBuffers() on the
 * output stream indicated by the \a output index.
 *
 * \return The number of allocated buffers on success or a negative error code
 * otherwise
 */

/**
 * \fn Converter::start()
 * \brief Start the converter streaming operation
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn Converter::stop()
 * \brief Stop the converter streaming operation
 */

/**
 * \fn Converter::queueBuffers()
 * \brief Queue buffers to converter device
 * \param[in] input The frame buffer to apply the conversion
 * \param[out] outputs The container holding the output stream indexes and
 * their respective frame buffer outputs.
 *
 * This function queues the \a input frame buffer on the output streams of the
 * \a outputs map key and retrieve the output frame buffer indicated by the
 * buffer map value.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \var Converter::inputBufferReady
 * \brief A signal emitted when the input frame buffer completes
 */

/**
 * \var Converter::outputBufferReady
 * \brief A signal emitted on each frame buffer completion of the output queue
 */

/**
 * \fn Converter::deviceNode()
 * \brief The converter device node attribute accessor
 * \return The converter device node string
 */

/**
 * \class ConverterFactoryBase
 * \brief Base class for converter factories
 *
 * The ConverterFactoryBase class is the base of all specializations of the
 * ConverterFactory class template. It implements the factory registration,
 * maintains a registry of factories, and provides access to the registered
 * factories.
 */

/**
 * \brief Construct a converter factory base
 * \param[in] name Name of the converter class
 * \param[in] compatibles Name aliases of the converter class
 *
 * Creating an instance of the factory base registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used as unique identifier. If the converter
 * implementation fully relies on a generic framework, the name should be the
 * same as the framework. Otherwise, if the implementation is specialized, the
 * factory name should match the driver name implementing the function.
 *
 * The factory \a compatibles holds a list of driver names implementing a generic
 * subsystem without any personalizations.
 */
ConverterFactoryBase::ConverterFactoryBase(const std::string name, std::initializer_list<std::string> compatibles)
	: name_(name), compatibles_(compatibles)
{
	registerType(this);
}

/**
 * \fn ConverterFactoryBase::compatibles()
 * \return The list of compatible name aliases of the converter
 */

/**
 * \brief Create an instance of the converter corresponding to the media device
 * \param[in] media The media device to create the converter for
 *
 * The converter is created by matching the factory name or any of its
 * compatible aliases with the media device driver name.
 *
 * \return A new instance of the converter subclass corresponding to the media
 * device, or null if the media device driver name doesn't match anything
 */
std::unique_ptr<Converter> ConverterFactoryBase::create(MediaDevice *media)
{
	const std::vector<ConverterFactoryBase *> &factories =
		ConverterFactoryBase::factories();

	for (const ConverterFactoryBase *factory : factories) {
		const std::vector<std::string> &compatibles = factory->compatibles();
		auto it = std::find(compatibles.begin(), compatibles.end(), media->driver());

		if (it == compatibles.end() && media->driver() != factory->name_)
			continue;

		LOG(Converter, Debug)
			<< "Creating converter from "
			<< factory->name_ << " factory with "
			<< (it == compatibles.end() ? "no" : media->driver()) << " alias.";

		std::unique_ptr<Converter> converter = factory->createInstance(media);
		if (converter->isValid())
			return converter;
	}

	return nullptr;
}

/**
 * \brief Add a converter factory to the registry
 * \param[in] factory Factory to use to construct the converter class
 *
 * The caller is responsible to guarantee the uniqueness of the converter
 * factory name.
 */
void ConverterFactoryBase::registerType(ConverterFactoryBase *factory)
{
	std::vector<ConverterFactoryBase *> &factories =
		ConverterFactoryBase::factories();

	factories.push_back(factory);
}

/**
 * \brief Retrieve the list of all converter factory names
 * \return The list of all converter factory names
 */
std::vector<std::string> ConverterFactoryBase::names()
{
	std::vector<std::string> list;

	std::vector<ConverterFactoryBase *> &factories =
		ConverterFactoryBase::factories();

	for (ConverterFactoryBase *factory : factories) {
		list.push_back(factory->name_);
		for (auto alias : factory->compatibles())
			list.push_back(alias);
	}

	return list;
}

/**
 * \brief Retrieve the list of all converter factories
 * \return The list of converter factories
 */
std::vector<ConverterFactoryBase *> &ConverterFactoryBase::factories()
{
	/*
	 * The static factories map is defined inside the function to ensure
	 * it gets initialized on first use, without any dependency on link
	 * order.
	 */
	static std::vector<ConverterFactoryBase *> factories;
	return factories;
}

/**
 * \var ConverterFactoryBase::name_
 * \brief The name of the factory
 */

/**
 * \var ConverterFactoryBase::compatibles_
 * \brief The list holding the factory compatibles
 */

/**
 * \class ConverterFactory
 * \brief Registration of ConverterFactory classes and creation of instances
 * \param _Converter The converter class type for this factory
 *
 * To facilitate discovery and instantiation of Converter classes, the
 * ConverterFactory class implements auto-registration of converter helpers.
 * Each Converter subclass shall register itself using the REGISTER_CONVERTER()
 * macro, which will create a corresponding instance of a ConverterFactory
 * subclass and register it with the static list of factories.
 */

/**
 * \fn ConverterFactory::ConverterFactory(const char *name, std::initializer_list<std::string> compatibles)
 * \brief Construct a converter factory
 * \details \copydetails ConverterFactoryBase::ConverterFactoryBase
 */

/**
 * \fn ConverterFactory::createInstance() const
 * \brief Create an instance of the Converter corresponding to the factory
 * \param[in] media Media device pointer
 * \return A unique pointer to a newly constructed instance of the Converter
 * subclass corresponding to the factory
 */

/**
 * \def REGISTER_CONVERTER
 * \brief Register a converter with the Converter factory
 * \param[in] name Converter name used to register the class
 * \param[in] converter Class name of Converter derived class to register
 * \param[in] compatibles List of compatible names
 *
 * Register a Converter subclass with the factory and make it available to try
 * and match converters.
 */

} /* namespace libcamera */
