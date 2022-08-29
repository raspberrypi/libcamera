/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * media_device.cpp - Media device handler
 */

#include "libcamera/internal/media_device.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

#include <linux/media.h>

#include <libcamera/base/log.h>

/**
 * \file media_device.h
 * \brief Provide a representation of a Linux kernel Media Controller device
 * that exposes the full graph topology.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(MediaDevice)

/**
 * \class MediaDevice
 * \brief The MediaDevice represents a Media Controller device with its full
 * graph of connected objects.
 *
 * A MediaDevice instance is associated with a media controller device node when
 * created, and that association is kept for the lifetime of the MediaDevice
 * instance.
 *
 * The instance is created with an empty media graph. Before performing any
 * other operation, it must be populate by calling populate(). Instances of
 * MediaEntity, MediaPad and MediaLink are created to model the media graph,
 * and stored in a map indexed by object id.
 *
 * The graph is valid once successfully populated, as reported by the isValid()
 * function. It can be queried to list all entities(), or entities can be
 * looked up by name with getEntityByName(). The graph can be traversed from
 * entity to entity through pads and links as exposed by the corresponding
 * classes.
 *
 * Media devices can be claimed for exclusive use with acquire(), released with
 * release() and tested with busy(). This mechanism is aimed at pipeline
 * managers to claim media devices they support during enumeration.
 */

/**
 * \brief Construct a MediaDevice
 * \param[in] deviceNode The media device node path
 *
 * Once constructed the media device is invalid, and must be populated with
 * populate() before the media graph can be queried.
 */
MediaDevice::MediaDevice(const std::string &deviceNode)
	: deviceNode_(deviceNode), valid_(false), acquired_(false)
{
}

MediaDevice::~MediaDevice()
{
	fd_.reset();
	clear();
}

std::string MediaDevice::logPrefix() const
{
	return deviceNode() + "[" + driver() + "]";
}

/**
 * \brief Claim a device for exclusive use
 *
 * The device claiming mechanism offers simple media device access arbitration
 * between multiple users. When the media device is created, it is available to
 * all users. Users can query the media graph to determine whether they can
 * support the device and, if they do, claim the device for exclusive use. Other
 * users are then expected to skip over media devices in use as reported by the
 * busy() function.
 *
 * Once claimed the device shall be released by its user when not needed anymore
 * by calling the release() function. Acquiring the media device opens a file
 * descriptor to the device which is kept open until release() is called.
 *
 * Exclusive access is only guaranteed if all users of the media device abide by
 * the device claiming mechanism, as it isn't enforced by the media device
 * itself.
 *
 * \return true if the device was successfully claimed, or false if it was
 * already in use
 * \sa release(), busy()
 */
bool MediaDevice::acquire()
{
	if (acquired_)
		return false;

	if (open())
		return false;

	acquired_ = true;
	return true;
}

/**
 * \brief Release a device previously claimed for exclusive use
 * \sa acquire(), busy()
 */
void MediaDevice::release()
{
	close();
	acquired_ = false;
}

/**
 * \brief Lock the device to prevent it from being used by other instances of
 * libcamera
 *
 * Multiple instances of libcamera might be running on the same system, at the
 * same time. To allow the different instances to coexist, system resources in
 * the form of media devices must be accessible for enumerating the cameras
 * they provide at all times, while still allowing an instance to lock a
 * resource while it prepares to actively use a camera from the resource.
 *
 * This function shall not be called from a pipeline handler implementation
 * directly, as the base PipelineHandler implementation handles this on the
 * behalf of the specified implementation.
 *
 * \return True if the device could be locked, false otherwise
 * \sa unlock()
 */
bool MediaDevice::lock()
{
	if (!fd_.isValid())
		return false;

	if (lockf(fd_.get(), F_TLOCK, 0))
		return false;

	return true;
}

/**
 * \brief Unlock the device and free it for use for libcamera instances
 *
 * This function shall not be called from a pipeline handler implementation
 * directly, as the base PipelineHandler implementation handles this on the
 * behalf of the specified implementation.
 *
 * \sa lock()
 */
void MediaDevice::unlock()
{
	if (!fd_.isValid())
		return;

	lockf(fd_.get(), F_ULOCK, 0);
}

/**
 * \fn MediaDevice::busy()
 * \brief Check if a device is in use
 * \return true if the device has been claimed for exclusive use, or false if it
 * is available
 * \sa acquire(), release()
 */

/**
 * \brief Populate the MediaDevice with device information and media objects
 *
 * This function retrieves the media device information and enumerates all
 * media objects in the media device graph and creates their MediaObject
 * representations. All entities, pads and links are stored as MediaEntity,
 * MediaPad and MediaLink respectively, with cross-references between objects.
 * Interfaces are not processed.
 *
 * Entities are stored in a separate list in the MediaDevice to ease lookup,
 * while pads are accessible from the entity they belong to and links from the
 * pads they connect.
 *
 * \return 0 on success or a negative error code otherwise
 */
int MediaDevice::populate()
{
	struct media_v2_topology topology = {};
	struct media_v2_entity *ents = nullptr;
	struct media_v2_interface *interfaces = nullptr;
	struct media_v2_link *links = nullptr;
	struct media_v2_pad *pads = nullptr;
	__u64 version = -1;
	int ret;

	clear();

	ret = open();
	if (ret)
		return ret;

	struct media_device_info info = {};
	ret = ioctl(fd_.get(), MEDIA_IOC_DEVICE_INFO, &info);
	if (ret) {
		ret = -errno;
		LOG(MediaDevice, Error)
			<< "Failed to get media device info " << strerror(-ret);
		goto done;
	}

	driver_ = info.driver;
	model_ = info.model;
	version_ = info.media_version;
	hwRevision_ = info.hw_revision;

	/*
	 * Keep calling G_TOPOLOGY until the version number stays stable.
	 */
	while (true) {
		topology.topology_version = 0;
		topology.ptr_entities = reinterpret_cast<uintptr_t>(ents);
		topology.ptr_interfaces = reinterpret_cast<uintptr_t>(interfaces);
		topology.ptr_links = reinterpret_cast<uintptr_t>(links);
		topology.ptr_pads = reinterpret_cast<uintptr_t>(pads);

		ret = ioctl(fd_.get(), MEDIA_IOC_G_TOPOLOGY, &topology);
		if (ret < 0) {
			ret = -errno;
			LOG(MediaDevice, Error)
				<< "Failed to enumerate topology: "
				<< strerror(-ret);
			goto done;
		}

		if (version == topology.topology_version)
			break;

		delete[] ents;
		delete[] interfaces;
		delete[] pads;
		delete[] links;

		ents = new struct media_v2_entity[topology.num_entities]();
		interfaces = new struct media_v2_interface[topology.num_interfaces]();
		links = new struct media_v2_link[topology.num_links]();
		pads = new struct media_v2_pad[topology.num_pads]();

		version = topology.topology_version;
	}

	/* Populate entities, pads and links. */
	if (populateEntities(topology) &&
	    populatePads(topology) &&
	    populateLinks(topology))
		valid_ = true;

	ret = 0;
done:
	close();

	delete[] ents;
	delete[] interfaces;
	delete[] pads;
	delete[] links;

	if (!valid_) {
		clear();
		return -EINVAL;
	}

	return ret;
}

/**
 * \fn MediaDevice::isValid()
 * \brief Query whether the media graph has been populated and is valid
 * \return true if the media graph is valid, false otherwise
 */

/**
 * \fn MediaDevice::driver()
 * \brief Retrieve the media device driver name
 * \return The name of the kernel driver that handles the MediaDevice
 */

/**
 * \fn MediaDevice::deviceNode()
 * \brief Retrieve the media device node path
 * \return The MediaDevice deviceNode path
 */

/**
 * \fn MediaDevice::model()
 * \brief Retrieve the media device model name
 * \return The MediaDevice model name
 */

/**
 * \fn MediaDevice::version()
 * \brief Retrieve the media device API version
 *
 * The version is formatted with the KERNEL_VERSION() macro.
 *
 * \return The MediaDevice API version
 */

/**
 * \fn MediaDevice::hwRevision()
 * \brief Retrieve the media device hardware revision
 *
 * The hardware revision is in a driver-specific format.
 *
 * \return The MediaDevice hardware revision
 */

/**
 * \fn MediaDevice::entities()
 * \brief Retrieve the list of entities in the media graph
 * \return The list of MediaEntities registered in the MediaDevice
 */

/**
 * \brief Return the MediaEntity with name \a name
 * \param[in] name The entity name
 * \return The entity with \a name, or nullptr if no such entity is found
 */
MediaEntity *MediaDevice::getEntityByName(const std::string &name) const
{
	for (MediaEntity *e : entities_)
		if (e->name() == name)
			return e;

	return nullptr;
}

/**
 * \brief Retrieve the MediaLink connecting two pads, identified by entity
 * names and pad indexes
 * \param[in] sourceName The source entity name
 * \param[in] sourceIdx The index of the source pad
 * \param[in] sinkName The sink entity name
 * \param[in] sinkIdx The index of the sink pad
 *
 * Find the link that connects the pads at index \a sourceIdx of the source
 * entity with name \a sourceName, to the pad at index \a sinkIdx of the
 * sink entity with name \a sinkName, if any.
 *
 * \sa link(const MediaEntity *source, unsigned int sourceIdx,
 *          const MediaEntity *sink, unsigned int sinkIdx)
 * \sa link(const MediaPad *source, const MediaPad *sink)
 *
 * \return The link that connects the two pads, or nullptr if no such a link
 * exists
 */
MediaLink *MediaDevice::link(const std::string &sourceName, unsigned int sourceIdx,
			     const std::string &sinkName, unsigned int sinkIdx)
{
	const MediaEntity *source = getEntityByName(sourceName);
	const MediaEntity *sink = getEntityByName(sinkName);
	if (!source || !sink)
		return nullptr;

	return link(source, sourceIdx, sink, sinkIdx);
}

/**
 * \brief Retrieve the MediaLink connecting two pads, identified by the
 * entities they belong to and pad indexes
 * \param[in] source The source entity
 * \param[in] sourceIdx The index of the source pad
 * \param[in] sink The sink entity
 * \param[in] sinkIdx The index of the sink pad
 *
 * Find the link that connects the pads at index \a sourceIdx of the source
 * entity \a source, to the pad at index \a sinkIdx of the sink entity \a
 * sink, if any.
 *
 * \sa link(const std::string &sourceName, unsigned int sourceIdx,
 *          const std::string &sinkName, unsigned int sinkIdx)
 * \sa link(const MediaPad *source, const MediaPad *sink)
 *
 * \return The link that connects the two pads, or nullptr if no such a link
 * exists
 */
MediaLink *MediaDevice::link(const MediaEntity *source, unsigned int sourceIdx,
			     const MediaEntity *sink, unsigned int sinkIdx)
{

	const MediaPad *sourcePad = source->getPadByIndex(sourceIdx);
	const MediaPad *sinkPad = sink->getPadByIndex(sinkIdx);
	if (!sourcePad || !sinkPad)
		return nullptr;

	return link(sourcePad, sinkPad);
}

/**
 * \brief Retrieve the MediaLink that connects two pads
 * \param[in] source The source pad
 * \param[in] sink The sink pad
 *
 * \sa link(const std::string &sourceName, unsigned int sourceIdx,
 *          const std::string &sinkName, unsigned int sinkIdx)
 * \sa link(const MediaEntity *source, unsigned int sourceIdx,
 *          const MediaEntity *sink, unsigned int sinkIdx)
 *
 * \return The link that connects the two pads, or nullptr if no such a link
 * exists
 */
MediaLink *MediaDevice::link(const MediaPad *source, const MediaPad *sink)
{
	for (MediaLink *link : source->links()) {
		if (link->sink()->id() == sink->id())
			return link;
	}

	return nullptr;
}

/**
 * \brief Disable all links in the media device
 *
 * Disable all the media device links, clearing the MEDIA_LNK_FL_ENABLED flag
 * on links which are not flagged as IMMUTABLE.
 *
 * \return 0 on success or a negative error code otherwise
 */
int MediaDevice::disableLinks()
{
	for (MediaEntity *entity : entities_) {
		for (MediaPad *pad : entity->pads()) {
			if (!(pad->flags() & MEDIA_PAD_FL_SOURCE))
				continue;

			for (MediaLink *link : pad->links()) {
				if (link->flags() & MEDIA_LNK_FL_IMMUTABLE)
					continue;

				int ret = link->setEnabled(false);
				if (ret)
					return ret;
			}
		}
	}

	return 0;
}

/**
 * \var MediaDevice::disconnected
 * \brief Signal emitted when the media device is disconnected from the system
 *
 * This signal is emitted when the device enumerator detects that the media
 * device has been removed from the system. For hot-pluggable devices this is
 * usually caused by physical device disconnection, but can also result from
 * driver unloading for most devices. The media device is passed as a parameter.
 */

/**
 * \brief Open the media device
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EBUSY Media device already open
 * \sa close()
 */
int MediaDevice::open()
{
	if (fd_.isValid()) {
		LOG(MediaDevice, Error) << "MediaDevice already open";
		return -EBUSY;
	}

	fd_ = UniqueFD(::open(deviceNode_.c_str(), O_RDWR));
	if (!fd_.isValid()) {
		int ret = -errno;
		LOG(MediaDevice, Error)
			<< "Failed to open media device at "
			<< deviceNode_ << ": " << strerror(-ret);
		return ret;
	}

	return 0;
}

/**
 * \brief Close the media device
 *
 * This function closes the media device node. It does not invalidate the media
 * graph and all cached media objects remain valid and can be accessed normally.
 * Once closed no operation interacting with the media device node can be
 * performed until the device is opened again.
 *
 * Closing an already closed device is allowed and will not perform any
 * operation.
 *
 * \sa open()
 */
void MediaDevice::close()
{
	fd_.reset();
}

/**
 * \var MediaDevice::objects_
 * \brief Global map of media objects (entities, pads, links) keyed by their
 * object id.
 */

/**
 * \brief Retrieve the media graph object specified by \a id
 * \return The graph object, or nullptr if no object with \a id is found
 */
MediaObject *MediaDevice::object(unsigned int id)
{
	auto it = objects_.find(id);
	return (it == objects_.end()) ? nullptr : it->second;
}

/**
 * \brief Add a media object to the media graph
 *
 * If the \a object has a unique id it is added to the media graph, and its
 * lifetime will be managed by the media device. Otherwise the object isn't
 * added to the graph and the caller must delete it.
 *
 * \return true if the object was successfully added to the graph and false
 * otherwise
 */
bool MediaDevice::addObject(MediaObject *object)
{

	if (objects_.find(object->id()) != objects_.end()) {
		LOG(MediaDevice, Error)
			<< "Element with id " << object->id()
			<< " already enumerated.";
		return false;
	}

	objects_[object->id()] = object;

	return true;
}

/**
 * \brief Delete all graph objects in the media device
 *
 * Clear the media graph and delete all the objects it contains. After this
 * function returns any previously obtained pointer to a media graph object
 * becomes invalid.
 *
 * The media device graph state is reset to invalid when the graph is cleared.
 *
 * \sa isValid()
 */
void MediaDevice::clear()
{
	for (auto const &o : objects_)
		delete o.second;

	objects_.clear();
	entities_.clear();
	valid_ = false;
}

/**
 * \var MediaDevice::entities_
 * \brief Global list of media entities in the media graph
 */

/**
 * \brief Find the interface associated with an entity
 * \param[in] topology The media topology as returned by MEDIA_IOC_G_TOPOLOGY
 * \param[in] entityId The entity id
 * \return A pointer to the interface if found, or nullptr otherwise
 */
struct media_v2_interface *MediaDevice::findInterface(const struct media_v2_topology &topology,
						      unsigned int entityId)
{
	struct media_v2_link *links = reinterpret_cast<struct media_v2_link *>
						      (topology.ptr_links);
	unsigned int ifaceId = 0;
	unsigned int i;

	for (i = 0; i < topology.num_links; ++i) {
		/* Search for the interface to entity link. */
		if (links[i].sink_id != entityId)
			continue;

		if ((links[i].flags & MEDIA_LNK_FL_LINK_TYPE) !=
		    MEDIA_LNK_FL_INTERFACE_LINK)
			continue;

		ifaceId = links[i].source_id;
		break;
	}
	if (i == topology.num_links)
		return nullptr;

	struct media_v2_interface *ifaces = reinterpret_cast<struct media_v2_interface *>
					    (topology.ptr_interfaces);
	for (i = 0; i < topology.num_interfaces; ++i) {
		if (ifaces[i].id == ifaceId)
			return &ifaces[i];
	}

	return nullptr;
}

/*
 * For each entity in the media graph create a MediaEntity and store a
 * reference in the media device objects map and entities list.
 */
bool MediaDevice::populateEntities(const struct media_v2_topology &topology)
{
	struct media_v2_entity *mediaEntities = reinterpret_cast<struct media_v2_entity *>
						(topology.ptr_entities);

	for (unsigned int i = 0; i < topology.num_entities; ++i) {
		struct media_v2_entity *ent = &mediaEntities[i];

		/*
		 * The media_v2_entity structure was missing the flag field before
		 * v4.19.
		 */
		if (!MEDIA_V2_ENTITY_HAS_FLAGS(version_))
			fixupEntityFlags(ent);

		/*
		 * Find the interface linked to this entity to get the device
		 * node major and minor numbers.
		 */
		struct media_v2_interface *iface =
			findInterface(topology, ent->id);
		MediaEntity *entity = new MediaEntity(this, ent, iface);

		if (!addObject(entity)) {
			delete entity;
			return false;
		}

		entities_.push_back(entity);
	}

	return true;
}

bool MediaDevice::populatePads(const struct media_v2_topology &topology)
{
	struct media_v2_pad *mediaPads = reinterpret_cast<struct media_v2_pad *>
					 (topology.ptr_pads);

	for (unsigned int i = 0; i < topology.num_pads; ++i) {
		unsigned int entity_id = mediaPads[i].entity_id;

		/* Store a reference to this MediaPad in entity. */
		MediaEntity *mediaEntity = dynamic_cast<MediaEntity *>
					   (object(entity_id));
		if (!mediaEntity) {
			LOG(MediaDevice, Error)
				<< "Failed to find entity with id: "
				<< entity_id;
			return false;
		}

		MediaPad *pad = new MediaPad(&mediaPads[i], mediaEntity);
		if (!addObject(pad)) {
			delete pad;
			return false;
		}

		mediaEntity->addPad(pad);
	}

	return true;
}

bool MediaDevice::populateLinks(const struct media_v2_topology &topology)
{
	struct media_v2_link *mediaLinks = reinterpret_cast<struct media_v2_link *>
					   (topology.ptr_links);

	for (unsigned int i = 0; i < topology.num_links; ++i) {
		if ((mediaLinks[i].flags & MEDIA_LNK_FL_LINK_TYPE) ==
		    MEDIA_LNK_FL_INTERFACE_LINK)
			continue;

		/* Look up the source and sink objects. */
		unsigned int source_id = mediaLinks[i].source_id;
		MediaObject *source = object(source_id);
		if (!source) {
			LOG(MediaDevice, Error)
				<< "Failed to find MediaObject with id "
				<< source_id;
			return false;
		}

		unsigned int sink_id = mediaLinks[i].sink_id;
		MediaObject *sink = object(sink_id);
		if (!sink) {
			LOG(MediaDevice, Error)
				<< "Failed to find MediaObject with id "
				<< sink_id;
			return false;
		}

		switch (mediaLinks[i].flags & MEDIA_LNK_FL_LINK_TYPE) {
		case MEDIA_LNK_FL_DATA_LINK: {
			MediaPad *sourcePad = dynamic_cast<MediaPad *>(source);
			MediaPad *sinkPad = dynamic_cast<MediaPad *>(sink);
			if (!source || !sink) {
				LOG(MediaDevice, Error)
					<< "Source or sink is not a pad";
				return false;
			}

			MediaLink *link = new MediaLink(&mediaLinks[i],
							sourcePad, sinkPad);
			if (!addObject(link)) {
				delete link;
				return false;
			}

			link->source()->addLink(link);
			link->sink()->addLink(link);

			break;
		}

		case MEDIA_LNK_FL_ANCILLARY_LINK: {
			MediaEntity *primary = dynamic_cast<MediaEntity *>(source);
			MediaEntity *ancillary = dynamic_cast<MediaEntity *>(sink);
			if (!primary || !ancillary) {
				LOG(MediaDevice, Error)
					<< "Source or sink is not an entity";
				return false;
			}

			primary->addAncillaryEntity(ancillary);

			break;
		}

		default:
			LOG(MediaDevice, Warning)
				<< "Unknown media link type";

			break;
		}
	}

	return true;
}

/**
 * \brief Fixup entity flags using the legacy API
 * \param[in] entity The entity
 *
 * This function is used as a fallback to query entity flags using the legacy
 * MEDIA_IOC_ENUM_ENTITIES ioctl when running on a kernel version that doesn't
 * provide them through the MEDIA_IOC_G_TOPOLOGY ioctl.
 */
void MediaDevice::fixupEntityFlags(struct media_v2_entity *entity)
{
	struct media_entity_desc desc = {};
	desc.id = entity->id;

	int ret = ioctl(fd_.get(), MEDIA_IOC_ENUM_ENTITIES, &desc);
	if (ret < 0) {
		ret = -errno;
		LOG(MediaDevice, Debug)
			<< "Failed to retrieve information for entity "
			<< entity->id << ": " << strerror(-ret);
		return;
	}

	entity->flags = desc.flags;
}

/**
 * \brief Apply \a flags to a link between two pads
 * \param[in] link The link to apply flags to
 * \param[in] flags The flags to apply to the link
 *
 * This function applies the link \a flags (as defined by the MEDIA_LNK_FL_*
 * macros from the Media Controller API) to the given \a link. It implements
 * low-level link setup as it performs no checks on the validity of the \a
 * flags, and assumes that the supplied \a flags are valid for the link (e.g.
 * immutable links cannot be disabled).
*
 * \sa MediaLink::setEnabled(bool enable)
 *
 * \return 0 on success or a negative error code otherwise
 */
int MediaDevice::setupLink(const MediaLink *link, unsigned int flags)
{
	struct media_link_desc linkDesc = {};
	MediaPad *source = link->source();
	MediaPad *sink = link->sink();

	linkDesc.source.entity = source->entity()->id();
	linkDesc.source.index = source->index();
	linkDesc.source.flags = MEDIA_PAD_FL_SOURCE;

	linkDesc.sink.entity = sink->entity()->id();
	linkDesc.sink.index = sink->index();
	linkDesc.sink.flags = MEDIA_PAD_FL_SINK;

	linkDesc.flags = flags;

	int ret = ioctl(fd_.get(), MEDIA_IOC_SETUP_LINK, &linkDesc);
	if (ret) {
		ret = -errno;
		LOG(MediaDevice, Error)
			<< "Failed to setup link "
			<< source->entity()->name() << "["
			<< source->index() << "] -> "
			<< sink->entity()->name() << "["
			<< sink->index() << "]: "
			<< strerror(-ret);
		return ret;
	}

	LOG(MediaDevice, Debug)
		<< source->entity()->name() << "["
		<< source->index() << "] -> "
		<< sink->entity()->name() << "["
		<< sink->index() << "]: " << flags;

	return 0;
}

} /* namespace libcamera */
