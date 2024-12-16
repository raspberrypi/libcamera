/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2022 NXP
 *
 * Generic format converter interface
 */

#include "libcamera/internal/converter.h"

#include <algorithm>

#include <libcamera/base/log.h>

#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"

/**
 * \file converter.h
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
 * \enum Converter::Feature
 * \brief Specify the features supported by the converter
 * \var Converter::Feature::None
 * \brief No extra features supported by the converter
 * \var Converter::Feature::InputCrop
 * \brief Cropping capability at input is supported by the converter
 */

/**
 * \typedef Converter::Features
 * \brief A bitwise combination of features supported by the converter
 */

/**
 * \enum Converter::Alignment
 * \brief The alignment mode specified when adjusting the converter input or
 * output sizes
 * \var Converter::Alignment::Down
 * \brief Adjust the Converter sizes to a smaller valid size
 * \var Converter::Alignment::Up
 * \brief Adjust the Converter sizes to a larger valid size
 */

/**
 * \brief Construct a Converter instance
 * \param[in] media The media device implementing the converter
 * \param[in] features Features flags representing supported features
 *
 * This searches for the entity implementing the data streaming function in the
 * media graph entities and use its device node as the converter device node.
 */
Converter::Converter(MediaDevice *media, Features features)
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
	features_ = features;
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
 * \fn Converter::adjustInputSize()
 * \brief Adjust the converter input \a size to a valid value
 * \param[in] pixFmt The pixel format of the converter input stream
 * \param[in] size The converter input size to adjust to a valid value
 * \param[in] align The desired alignment
 * \return The adjusted converter input size or a null Size if \a size cannot
 * be adjusted
 */

/**
 * \fn Converter::adjustOutputSize()
 * \brief Adjust the converter output \a size to a valid value
 * \param[in] pixFmt The pixel format of the converter output stream
 * \param[in] size The converter output size to adjust to a valid value
 * \param[in] align The desired alignment
 * \return The adjusted converter output size or a null Size if \a size cannot
 * be adjusted
 */

/**
 * \fn Converter::strideAndFrameSize()
 * \brief Retrieve the output stride and frame size for an input configutation
 * \param[in] pixelFormat Input stream pixel format
 * \param[in] size Input stream size
 * \return A tuple indicating the stride and frame size or an empty tuple on error
 */

/**
 * \fn Converter::validateOutput()
 * \brief Validate and possibily adjust \a cfg to a valid converter output
 * \param[inout] cfg The StreamConfiguration to validate and adjust
 * \param[out] adjusted Set to true if \a cfg has been adjusted
 * \param[in] align The desired alignment
 * \return 0 if \a cfg is valid or has been adjusted, a negative error code
 * otherwise if \a cfg cannot be adjusted
 */

/**
 * \fn Converter::configure()
 * \brief Configure a set of output stream conversion from an input stream
 * \param[in] inputCfg Input stream configuration
 * \param[out] outputCfgs A list of output stream configurations
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn Converter::isConfigured()
 * \brief Check if a given stream is configured
 * \param[in] stream The output stream
 * \return True if the \a stream is configured or false otherwise
 */

/**
 * \fn Converter::exportBuffers()
 * \brief Export buffers from the converter device
 * \param[in] stream Output stream pointer exporting the buffers
 * \param[in] count Number of buffers to allocate
 * \param[out] buffers Vector to store allocated buffers
 *
 * This function operates similarly to V4L2VideoDevice::exportBuffers() on the
 * output stream indicated by the \a output.
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
 * \param[out] outputs The container holding the output stream pointers and
 * their respective frame buffer outputs.
 *
 * This function queues the \a input frame buffer on the output streams of the
 * \a outputs map key and retrieve the output frame buffer indicated by the
 * buffer map value.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn Converter::setInputCrop()
 * \brief Set the crop rectangle \a rect for \a stream
 * \param[in] stream The output stream
 * \param[inout] rect The crop rectangle to apply and return the rectangle
 * that is actually applied
 *
 * Set the crop rectangle \a rect for \a stream provided the converter supports
 * cropping. The converter has the Feature::InputCrop flag in this case.
 *
 * The underlying hardware can adjust the rectangle supplied by the user
 * due to hardware constraints. The caller can inspect \a rect to determine the
 * actual rectangle that has been applied by the converter, after this function
 * returns.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn Converter::inputCropBounds()
 * \brief Retrieve the crop bounds of the converter
 *
 * Retrieve the minimum and maximum crop bounds of the converter. This can be
 * used to query the crop bounds before configuring a stream.
 *
 * \return A pair containing the minimum and maximum crop bound in that order
 */

/**
 * \fn Converter::inputCropBounds(const Stream *stream)
 * \brief Retrieve the crop bounds for \a stream
 * \param[in] stream The output stream
 *
 * Retrieve the minimum and maximum crop bounds for \a stream. The converter
 * should support cropping (Feature::InputCrop).
 *
 * The crop bounds depend on the configuration of the output stream and hence
 * this function should be called after the \a stream has been configured using
 * configure().
 *
 * When called with an unconfigured \a stream, this function returns a pair of
 * null rectangles.
 *
 * \return A pair containing the minimum and maximum crop bound in that order
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
 * \var Converter::features_
 * \brief Stores the features supported by the converter
 */

/**
 * \fn Converter::deviceNode()
 * \brief The converter device node attribute accessor
 * \return The converter device node string
 */

/**
 * \fn Converter::features()
 * \brief Retrieve the features supported by the converter
 * \return The converter Features flags
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

		const auto &compatibles = factory->compatibles();
		list.insert(list.end(), compatibles.begin(), compatibles.end());
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
