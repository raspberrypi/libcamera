/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_sensor.cpp - A camera sensor
 */

#include "libcamera/internal/camera_sensor.h"

#include <memory>
#include <vector>

#include <libcamera/base/log.h>

#include "libcamera/internal/media_object.h"

/**
 * \file camera_sensor.h
 * \brief A camera sensor
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraSensor)

/**
 * \class CameraSensor
 * \brief A abstract camera sensor
 *
 * The CameraSensor class eases handling of sensors for pipeline handlers by
 * hiding the details of the kernel API and caching sensor information.
 */

/**
 * \brief Destroy a CameraSensor
 */
CameraSensor::~CameraSensor() = default;

/**
 * \fn CameraSensor::model()
 * \brief Retrieve the sensor model name
 *
 * The sensor model name is a free-formed string that uniquely identifies the
 * sensor model.
 *
 * \return The sensor model name
 */

/**
 * \fn CameraSensor::id()
 * \brief Retrieve the sensor ID
 *
 * The sensor ID is a free-form string that uniquely identifies the sensor in
 * the system. The ID satisfies the requirements to be used as a camera ID.
 *
 * \return The sensor ID
 */

/**
 * \fn CameraSensor::entity()
 * \brief Retrieve the sensor media entity
 * \return The sensor media entity
 */

/**
 * \fn CameraSensor::focusLens()
 * \brief Retrieve the focus lens controller
 *
 * \return The focus lens controller. nullptr if no focus lens controller is
 * connected to the sensor
 */

/**
 * \fn CameraSensor::mbusCodes()
 * \brief Retrieve the media bus codes supported by the camera sensor
 *
 * Any Bayer formats are listed using the sensor's native Bayer order,
 * that is, with the effect of V4L2_CID_HFLIP and V4L2_CID_VFLIP undone
 * (where these controls exist).
 *
 * \return The supported media bus codes sorted in increasing order
 */

/**
 * \fn CameraSensor::sizes()
 * \brief Retrieve the supported frame sizes for a media bus code
 * \param[in] mbusCode The media bus code for which sizes are requested
 *
 * \return The supported frame sizes for \a mbusCode sorted in increasing order
 */

/**
 * \fn CameraSensor::resolution()
 * \brief Retrieve the camera sensor resolution
 *
 * The camera sensor resolution is the active pixel area size, clamped to the
 * maximum frame size the sensor can produce if it is smaller than the active
 * pixel area.
 *
 * \todo Consider if it desirable to distinguish between the maximum resolution
 * the sensor can produce (also including upscaled ones) and the actual pixel
 * array size by splitting this function in two.
 *
 * \return The camera sensor resolution in pixels
 */

/**
 * \fn CameraSensor::getFormat()
 * \brief Retrieve the best sensor format for a desired output
 * \param[in] mbusCodes The list of acceptable media bus codes
 * \param[in] size The desired size
 *
 * Media bus codes are selected from \a mbusCodes, which lists all acceptable
 * codes in decreasing order of preference. Media bus codes supported by the
 * sensor but not listed in \a mbusCodes are ignored. If none of the desired
 * codes is supported, it returns an error.
 *
 * \a size indicates the desired size at the output of the sensor. This function
 * selects the best media bus code and size supported by the sensor according
 * to the following criteria.
 *
 * - The desired \a size shall fit in the sensor output size to avoid the need
 *   to up-scale.
 * - The sensor output size shall match the desired aspect ratio to avoid the
 *   need to crop the field of view.
 * - The sensor output size shall be as small as possible to lower the required
 *   bandwidth.
 * - The desired \a size shall be supported by one of the media bus code listed
 *   in \a mbusCodes.
 *
 * When multiple media bus codes can produce the same size, the code at the
 * lowest position in \a mbusCodes is selected.
 *
 * The use of this function is optional, as the above criteria may not match the
 * needs of all pipeline handlers. Pipeline handlers may implement custom
 * sensor format selection when needed.
 *
 * The returned sensor output format is guaranteed to be acceptable by the
 * setFormat() function without any modification.
 *
 * \return The best sensor output format matching the desired media bus codes
 * and size on success, or an empty format otherwise.
 */

/**
 * \fn CameraSensor::setFormat()
 * \brief Set the sensor output format
 * \param[in] format The desired sensor output format
 * \param[in] transform The transform to be applied on the sensor.
 * Defaults to Identity.
 *
 * If flips are writable they are configured according to the desired Transform.
 * Transform::Identity always corresponds to H/V flip being disabled if the
 * controls are writable. Flips are set before the new format is applied as
 * they can effectively change the Bayer pattern ordering.
 *
 * The ranges of any controls associated with the sensor are also updated.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn CameraSensor::tryFormat()
 * \brief Try the sensor output format
 * \param[in] format The desired sensor output format
 *
 * The ranges of any controls associated with the sensor are not updated.
 *
 * \todo Add support for Transform by changing the format's Bayer ordering
 * before calling subdev_->setFormat().
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn CameraSensor::applyConfiguration()
 * \brief Apply a sensor configuration to the camera sensor
 * \param[in] config The sensor configuration
 * \param[in] transform The transform to be applied on the sensor.
 * Defaults to Identity
 * \param[out] sensorFormat Format applied to the sensor (optional)
 *
 * Apply to the camera sensor the configuration \a config.
 *
 * \todo The configuration shall be fully populated and if any of the fields
 * specified cannot be applied exactly, an error code is returned.
 *
 * \return 0 if \a config is applied correctly to the camera sensor, a negative
 * error code otherwise
 */

/**
 * \brief Retrieve the image source stream
 *
 * Sensors that produce multiple streams do not guarantee that the image stream
 * is always assigned number 0. This function allows callers to retrieve the
 * image stream on the sensor's source pad, in order to configure the receiving
 * side accordingly.
 *
 * \return The image source stream
 */
V4L2Subdevice::Stream CameraSensor::imageStream() const
{
	return { 0, 0 };
}

/**
 * \brief Retrieve the embedded data source stream
 *
 * Some sensors produce embedded data in a stream separate from the image
 * stream. This function indicates if the sensor supports this feature by
 * returning the embedded data stream on the sensor's source pad if available,
 * or an std::optional<> without a value otheriwse.
 *
 * \return The embedded data source stream
 */
std::optional<V4L2Subdevice::Stream> CameraSensor::embeddedDataStream() const
{
	return {};
}

/**
 * \brief Retrieve the format on the embedded data stream
 *
 * When an embedded data stream is available, this function returns the
 * corresponding format on the sensor's source pad. The format may vary with
 * the image stream format, and should therefore be retrieved after configuring
 * the image stream.
 *
 * If the sensor doesn't support embedded data, this function returns a
 * default-constructed format.
 *
 * \return The format on the embedded data stream
 */
V4L2SubdeviceFormat CameraSensor::embeddedDataFormat() const
{
	return {};
}

/**
 * \brief Enable or disable the embedded data stream
 * \param[in] enable True to enable the embedded data stream, false to disable it
 *
 * For sensors that support embedded data, this function enables or disables
 * generation of embedded data. Some of such sensors always produce embedded
 * data, in which case this function return -EISCONN if the caller attempts to
 * disable embedded data.
 *
 * If the sensor doesn't support embedded data, this function returns 0 when \a
 * enable is false, and -ENOSTR otherwise.
 *
 * \return 0 on success, or a negative error code otherwise
 */
int CameraSensor::setEmbeddedDataEnabled(bool enable)
{
	return enable ? -ENOSTR : 0;
}

/**
 * \fn CameraSensor::properties()
 * \brief Retrieve the camera sensor properties
 * \return The list of camera sensor properties
 */

/**
 * \fn CameraSensor::sensorInfo()
 * \brief Assemble and return the camera sensor info
 * \param[out] info The camera sensor info
 *
 * This function fills \a info with information that describes the camera sensor
 * and its current configuration. The information combines static data (such as
 * the the sensor model or active pixel array size) and data specific to the
 * current sensor configuration (such as the line length and pixel rate).
 *
 * Sensor information is only available for raw sensors. When called for a YUV
 * sensor, this function returns -EINVAL.
 *
 * \return 0 on success, a negative error code otherwise
 */

/**
 * \fn CameraSensor::computeTransform()
 * \brief Compute the Transform that gives the requested \a orientation
 * \param[inout] orientation The desired image orientation
 *
 * This function computes the Transform that the pipeline handler should apply
 * to the CameraSensor to obtain the requested \a orientation.
 *
 * The intended caller of this function is the validate() implementation of
 * pipeline handlers, that pass in the application requested
 * CameraConfiguration::orientation and obtain a Transform to apply to the
 * camera sensor, likely at configure() time.
 *
 * If the requested \a orientation cannot be obtained, the \a orientation
 * parameter is adjusted to report the current image orientation and
 * Transform::Identity is returned.
 *
 * If the requested \a orientation can be obtained, the function computes a
 * Transform and does not adjust \a orientation.
 *
 * Pipeline handlers are expected to verify if \a orientation has been
 * adjusted by this function and set the CameraConfiguration::status to
 * Adjusted accordingly.
 *
 * \return A Transform instance that applied to the CameraSensor produces images
 * with \a orientation
 */

/**
 * \fn CameraSensor::bayerOrder()
 * \brief Compute the Bayer order that results from the given Transform
 * \param[in] t The Transform to apply to the sensor
 *
 * Some sensors change their Bayer order when they are h-flipped or v-flipped.
 * This function computes and returns the Bayer order that would result from the
 * given transform applied to the sensor.
 *
 * This function is valid only when the sensor produces raw Bayer formats.
 *
 * \return The Bayer order produced by the sensor when the Transform is applied
 */

/**
 * \fn CameraSensor::controls()
 * \brief Retrieve the supported V4L2 controls and their information
 *
 * Control information is updated automatically to reflect the current sensor
 * configuration when the setFormat() function is called, without invalidating
 * any iterator on the ControlInfoMap.
 *
 * \return A map of the V4L2 controls supported by the sensor
 */

/**
 * \fn CameraSensor::getControls()
 * \brief Read V4L2 controls from the sensor
 * \param[in] ids The list of controls to read, specified by their ID
 *
 * This function reads the value of all controls contained in \a ids, and
 * returns their values as a ControlList. The control identifiers are defined by
 * the V4L2 specification (V4L2_CID_*).
 *
 * If any control in \a ids is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), or if any other error occurs
 * during validation of the requested controls, no control is read and this
 * function returns an empty control list.
 *
 * \sa V4L2Device::getControls()
 *
 * \return The control values in a ControlList on success, or an empty list on
 * error
 */

/**
 * \fn CameraSensor::setControls()
 * \brief Write V4L2 controls to the sensor
 * \param[in] ctrls The list of controls to write
 *
 * This function writes the value of all controls contained in \a ctrls, and
 * stores the values actually applied to the device in the corresponding \a
 * ctrls entry. The control identifiers are defined by the V4L2 specification
 * (V4L2_CID_*).
 *
 * If any control in \a ctrls is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), is read-only, or if any other
 * error occurs during validation of the requested controls, no control is
 * written and this function returns -EINVAL.
 *
 * If an error occurs while writing the controls, the index of the first
 * control that couldn't be written is returned. All controls below that index
 * are written and their values are updated in \a ctrls, while all other
 * controls are not written and their values are not changed.
 *
 * \sa V4L2Device::setControls()
 *
 * \return 0 on success or an error code otherwise
 * \retval -EINVAL One of the control is not supported or not accessible
 * \retval i The index of the control that failed
 */

/**
 * \fn CameraSensor::testPatternModes()
 * \brief Retrieve all the supported test pattern modes of the camera sensor
 * The test pattern mode values correspond to the controls::TestPattern control.
 *
 * \return The list of test pattern modes
 */

/**
 * \fn CameraSensor::setTestPatternMode()
 * \brief Set the test pattern mode for the camera sensor
 * \param[in] mode The test pattern mode
 *
 * The new \a mode is applied to the sensor if it differs from the active test
 * pattern mode. Otherwise, this function is a no-op. Setting the same test
 * pattern mode for every frame thus incurs no performance penalty.
 */

/**
 * \class CameraSensorFactoryBase
 * \brief Base class for camera sensor factories
 *
 * The CameraSensorFactoryBase class is the base of all specializations of
 * the CameraSensorFactory class template. It implements the factory
 * registration, maintains a registry of factories, and provides access to the
 * registered factories.
 */

/**
 * \brief Construct a camera sensor factory base
 * \param[in] priority Priority order for factory selection
 *
 * Creating an instance of the factory base registers it with the global list of
 * factories, accessible through the factories() function.
 */
CameraSensorFactoryBase::CameraSensorFactoryBase(int priority)
	: priority_(priority)
{
	registerFactory(this);
}

/**
 * \brief Create an instance of the CameraSensor corresponding to a media entity
 * \param[in] entity The media entity on the source end of the sensor
 *
 * When multiple factories match the same \a entity, this function selects the
 * matching factory with the highest priority as specified to the
 * REGISTER_CAMERA_SENSOR() macro at factory registration time. If multiple
 * matching factories have the same highest priority value, which factory gets
 * selected is undefined and may vary between runs.
 *
 * \return A unique pointer to a new instance of the CameraSensor subclass
 * matching the entity, or a null pointer if no such factory exists
 */
std::unique_ptr<CameraSensor> CameraSensorFactoryBase::create(MediaEntity *entity)
{
	const std::vector<CameraSensorFactoryBase *> &factories =
		CameraSensorFactoryBase::factories();

	for (const CameraSensorFactoryBase *factory : factories) {
		std::variant<std::unique_ptr<CameraSensor>, int> result =
			factory->match(entity);

		if (std::holds_alternative<std::unique_ptr<CameraSensor>>(result))
			return std::get<std::unique_ptr<CameraSensor>>(std::move(result));

		if (std::get<int>(result)) {
			LOG(CameraSensor, Error)
				<< "Failed to create sensor for '"
				<< entity->name() << ": " << std::get<int>(result);
			return nullptr;
		}
	}

	return nullptr;
}

/**
 * \fn CameraSensorFactoryBase::priority()
 * \brief Retrieve the priority value for the factory
 * \return The priority value for the factory
 */

/**
 * \brief Retrieve the list of all camera sensor factories
 *
 * The factories are sorted in decreasing priority order.
 *
 * \return The list of camera sensor factories
 */
std::vector<CameraSensorFactoryBase *> &CameraSensorFactoryBase::factories()
{
	/*
	 * The static factories map is defined inside the function to ensure
	 * it gets initialized on first use, without any dependency on link
	 * order.
	 */
	static std::vector<CameraSensorFactoryBase *> factories;
	return factories;
}

/**
 * \brief Add a camera sensor class to the registry
 * \param[in] factory Factory to use to construct the camera sensor
 */
void CameraSensorFactoryBase::registerFactory(CameraSensorFactoryBase *factory)
{
	std::vector<CameraSensorFactoryBase *> &factories =
		CameraSensorFactoryBase::factories();

	auto pos = std::upper_bound(factories.begin(), factories.end(), factory,
				    [](const CameraSensorFactoryBase *value,
				       const CameraSensorFactoryBase *elem) {
					    return value->priority() > elem->priority();
				    });
	factories.insert(pos, factory);
}

/**
 * \class CameraSensorFactory
 * \brief Registration of CameraSensorFactory classes and creation of instances
 * \tparam _CameraSensor The camera sensor class type for this factory
 *
 * To facilitate discovery and instantiation of CameraSensor classes, the
 * CameraSensorFactory class implements auto-registration of camera sensors.
 * Each CameraSensor subclass shall register itself using the
 * REGISTER_CAMERA_SENSOR() macro, which will create a corresponding instance
 * of a CameraSensorFactory subclass and register it with the static list of
 * factories.
 */

/**
 * \fn CameraSensorFactory::CameraSensorFactory()
 * \brief Construct a camera sensor factory
 *
 * Creating an instance of the factory registers it with the global list of
 * factories, accessible through the CameraSensorFactoryBase::factories()
 * function.
 */

/**
 * \def REGISTER_CAMERA_SENSOR(sensor, priority)
 * \brief Register a camera sensor type to the sensor factory
 * \param[in] sensor Class name of the CameraSensor derived class to register
 * \param[in] priority Priority order for factory selection
 *
 * Register a CameraSensor subclass with the factory and make it available to
 * try and match sensors. The subclass needs to implement a static match
 * function:
 *
 * \code{.cpp}
 * static std::variant<std::unique_ptr<CameraSensor>, int> match(MediaEntity *entity);
 * \endcode
 *
 * The function tests if the sensor class supports the camera sensor identified
 * by a MediaEntity. If so, it creates a new instance of the sensor class. The
 * return value is a variant that contains
 *
 * - A new instance of the camera sensor class if the entity matched and
 *   creation succeeded ;
 * - A non-zero error code if the entity matched and the creation failed ; or
 * - A zero error code if the entity didn't match.
 *
 * When multiple factories can support the same MediaEntity (as in the match()
 * function of multiple factories returning true for the same entity), the \a
 * priority argument selects which factory will be used. See
 * CameraSensorFactoryBase::create() for more information.
 */

} /* namespace libcamera */
