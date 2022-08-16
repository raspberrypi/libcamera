/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * device_enumerator.cpp - Enumeration and matching
 */

#include "libcamera/internal/device_enumerator.h"

#include <string.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/device_enumerator_sysfs.h"
#include "libcamera/internal/device_enumerator_udev.h"
#include "libcamera/internal/media_device.h"

/**
 * \file device_enumerator.h
 * \brief Enumeration and matching of media devices
 *
 * The purpose of device enumeration and matching is to find media devices in
 * the system and map them to pipeline handlers.
 *
 * At the core of the enumeration is the DeviceEnumerator class, responsible
 * for enumerating all media devices in the system. It handles all interactions
 * with the operating system in a platform-specific way. For each media device
 * found an instance of MediaDevice is created to store information about the
 * device gathered from the kernel through the Media Controller API.
 *
 * The DeviceEnumerator can enumerate all or specific media devices in the
 * system. When a new media device is added the enumerator creates a
 * corresponding MediaDevice instance.
 *
 * The enumerator supports searching among enumerated devices based on criteria
 * expressed in a DeviceMatch object.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(DeviceEnumerator)

/**
 * \class DeviceMatch
 * \brief Description of a media device search pattern
 *
 * The DeviceMatch class describes a media device using properties from the
 * Media Controller struct media_device_info, entity names in the media graph
 * or other properties that can be used to identify a media device.
 *
 * The description is meant to be filled by pipeline managers and passed to a
 * device enumerator to find matching media devices.
 *
 * A DeviceMatch is created with a specific Linux device driver in mind,
 * therefore the name of the driver is a required property. One or more Entity
 * names can be added as match criteria.
 *
 * Pipeline handlers are recommended to add entities to DeviceMatch as
 * appropriare to ensure that the media device they need can be uniquely
 * identified. This is useful when the corresponding kernel driver can produce
 * different graphs, for instance as a result of different driver versions or
 * hardware configurations, and not all those graphs are suitable for a pipeline
 * handler.
 */

/**
 * \brief Construct a media device search pattern
 * \param[in] driver The Linux device driver name that created the media device
 */
DeviceMatch::DeviceMatch(const std::string &driver)
	: driver_(driver)
{
}

/**
 * \brief Add a media entity name to the search pattern
 * \param[in] entity The name of the entity in the media graph
 */
void DeviceMatch::add(const std::string &entity)
{
	entities_.push_back(entity);
}

/**
 * \brief Compare a search pattern with a media device
 * \param[in] device The media device
 *
 * Matching is performed on the Linux device driver name and entity names from
 * the media graph. A match is found if both the driver name matches and the
 * media device contains all the entities listed in the search pattern.
 *
 * \return true if the media device matches the search pattern, false otherwise
 */
bool DeviceMatch::match(const MediaDevice *device) const
{
	if (driver_ != device->driver())
		return false;

	for (const std::string &name : entities_) {
		bool found = false;

		for (const MediaEntity *entity : device->entities()) {
			if (name == entity->name()) {
				found = true;
				break;
			}
		}

		if (!found)
			return false;
	}

	return true;
}

/**
 * \class DeviceEnumerator
 * \brief Enumerate, store and search media devices
 *
 * The DeviceEnumerator class is responsible for all interactions with the
 * operating system related to media devices. It enumerates all media devices
 * in the system, and for each device found creates an instance of the
 * MediaDevice class and stores it internally. The list of media devices can
 * then be searched using DeviceMatch search patterns.
 *
 * The enumerator also associates media device entities with device node paths.
 */

/**
 * \brief Create a new device enumerator matching the systems capabilities
 *
 * Depending on how the operating system handles device detection, hot-plug
 * notification and device node lookup, different device enumerator
 * implementations may be needed. This function creates the best enumerator for
 * the operating system based on the available resources. Not all different
 * enumerator types are guaranteed to support all features.
 *
 * \return A pointer to the newly created device enumerator on success, or
 * nullptr if an error occurs
 */
std::unique_ptr<DeviceEnumerator> DeviceEnumerator::create()
{
	std::unique_ptr<DeviceEnumerator> enumerator;

#ifdef HAVE_LIBUDEV
	enumerator = std::make_unique<DeviceEnumeratorUdev>();
	if (!enumerator->init())
		return enumerator;
#endif

	/*
	 * Either udev is not available or udev initialization failed. Fall back
	 * on the sysfs enumerator.
	 */
	enumerator = std::make_unique<DeviceEnumeratorSysfs>();
	if (!enumerator->init())
		return enumerator;

	return nullptr;
}

DeviceEnumerator::~DeviceEnumerator()
{
	for (const std::shared_ptr<MediaDevice> &media : devices_) {
		if (media->busy())
			LOG(DeviceEnumerator, Error)
				<< "Removing media device " << media->deviceNode()
				<< " while still in use";
	}
}

/**
 * \fn DeviceEnumerator::init()
 * \brief Initialize the enumerator
 * \return 0 on success or a negative error code otherwise
 * \retval -EBUSY the enumerator has already been initialized
 * \retval -ENODEV the enumerator can't enumerate devices
 */

/**
 * \fn DeviceEnumerator::enumerate()
 * \brief Enumerate all media devices in the system
 *
 * This function finds and add all media devices in the system to the
 * enumerator. It shall be implemented by all subclasses of DeviceEnumerator
 * using system-specific methods.
 *
 * Individual media devices that can't be properly enumerated shall be skipped
 * with a warning message logged, without returning an error. Only errors that
 * prevent enumeration altogether shall be fatal.
 *
 * \context This function is \threadbound.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \brief Create a media device instance
 * \param[in] deviceNode path to the media device to create
 *
 * Create a media device for the \a deviceNode, open it, and populate its
 * media graph. The device enumerator shall then populate the media device by
 * associating device nodes with entities using MediaEntity::setDeviceNode().
 * This process is specific to each device enumerator, and the device enumerator
 * shall ensure that device nodes are ready to be used (for instance, if
 * applicable, by waiting for device nodes to be created and access permissions
 * to be set by the system). Once done, it shall add the media device to the
 * system with addDevice().
 *
 * \return Created media device instance on success, or nullptr otherwise
 */
std::unique_ptr<MediaDevice> DeviceEnumerator::createDevice(const std::string &deviceNode)
{
	std::unique_ptr<MediaDevice> media = std::make_unique<MediaDevice>(deviceNode);

	int ret = media->populate();
	if (ret < 0) {
		LOG(DeviceEnumerator, Info)
			<< "Unable to populate media device " << deviceNode
			<< " (" << strerror(-ret) << "), skipping";
		return nullptr;
	}

	LOG(DeviceEnumerator, Debug)
		<< "New media device \"" << media->driver()
		<< "\" created from " << deviceNode;

	return media;
}

/**
* \var DeviceEnumerator::devicesAdded
* \brief Notify of new media devices being found
*
* This signal is emitted when the device enumerator finds new media devices in
* the system. It may be emitted for every newly detected device, or once for
* multiple devices, at the discretion of the device enumerator. Not all device
* enumerator types may support dynamic detection of new devices.
*/

/**
 * \brief Add a media device to the enumerator
 * \param[in] media media device instance to add
 *
 * Store the media device in the internal list for later matching with
 * pipeline handlers. \a media shall be created with createDevice() first.
 * This function shall be called after all members of the entities of the
 * media graph have been confirmed to be initialized.
 */
void DeviceEnumerator::addDevice(std::unique_ptr<MediaDevice> media)
{
	LOG(DeviceEnumerator, Debug)
		<< "Added device " << media->deviceNode() << ": " << media->driver();

	devices_.push_back(std::move(media));

	/* \todo To batch multiple additions, emit with a small delay here. */
	devicesAdded.emit();
}

/**
 * \brief Remove a media device from the enumerator
 * \param[in] deviceNode Path to the media device to remove
 *
 * Remove the media device identified by \a deviceNode previously added to the
 * enumerator with addDevice(). The media device's MediaDevice::disconnected
 * signal is emitted.
 */
void DeviceEnumerator::removeDevice(const std::string &deviceNode)
{
	std::shared_ptr<MediaDevice> media;

	for (auto iter = devices_.begin(); iter != devices_.end(); ++iter) {
		if ((*iter)->deviceNode() == deviceNode) {
			media = std::move(*iter);
			devices_.erase(iter);
			break;
		}
	}

	if (!media) {
		LOG(DeviceEnumerator, Warning)
			<< "Media device for node " << deviceNode
			<< " not found";
		return;
	}

	LOG(DeviceEnumerator, Debug)
		<< "Media device for node " << deviceNode << " removed.";

	media->disconnected.emit();
}

/**
 * \brief Search available media devices for a pattern match
 * \param[in] dm Search pattern
 *
 * Search in the enumerated media devices that are not already in use for a
 * match described in \a dm. If a match is found and the caller intends to use
 * it the caller is responsible for acquiring the MediaDevice object and
 * releasing it when done with it.
 *
 * \return pointer to the matching MediaDevice, or nullptr if no match is found
 */
std::shared_ptr<MediaDevice> DeviceEnumerator::search(const DeviceMatch &dm)
{
	for (std::shared_ptr<MediaDevice> &media : devices_) {
		if (media->busy())
			continue;

		if (dm.match(media.get())) {
			LOG(DeviceEnumerator, Debug)
				<< "Successful match for media device \""
				<< media->driver() << "\"";
			return media;
		}
	}

	return nullptr;
}

} /* namespace libcamera */
