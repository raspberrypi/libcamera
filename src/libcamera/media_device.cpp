/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * media_device.cpp - Media device handler
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <string>
#include <vector>

#include <linux/media.h>

#include "log.h"
#include "media_device.h"

/**
 * \file media_device.h
 * \brief Provide a representation of a Linux kernel Media Controller device
 * that exposes the full graph topology.
 */

namespace libcamera {

/**
 * \class MediaDevice
 * \brief The MediaDevice represents a Media Controller device with its full
 * graph of connected objects.
 *
 * Media devices are created with an empty graph, which must be populated from
 *
 *
 * The caller is responsible for opening the MediaDevice explicitly before
 * operating on it, and shall close it when not needed anymore, as access
 * to the MediaDevice is exclusive.
 *
 * A MediaDevice is created empty and gets populated by inspecting the media
 * graph topology using the MEDIA_IOC_G_TOPOLOGY ioctls. Representation
 * of each entity, pad and link described are created using MediaObject
 * derived classes.
 *
 * All MediaObject are stored in a global pool, where they could be retrieved
 * from by their globally unique id.
 *
 * References to MediaEntity registered in the graph are stored in a vector
 * to allow easier by-name lookup, and the list of MediaEntities is accessible.
 */

/**
 * \brief Construct a MediaDevice
 * \param devnode The media device node path
 */
MediaDevice::MediaDevice(const std::string &devnode)
	: devnode_(devnode), fd_(-1), valid_(false)
{
}

/**
 * \brief Close the media device file descriptor and delete all object
 */
MediaDevice::~MediaDevice()
{
	if (fd_ != -1)
		::close(fd_);
	clear();
}

/**
 * \fn MediaDevice::driver()
 * \brief Retrieve the media device driver name
 * \return The name of the kernel driver that handles the MediaDevice
 */

/**
 * \fn MediaDevice::devnode()
 * \brief Retrieve the media device device node path
 * \return The MediaDevice devnode path
 */

/**
 * \brief Delete all media objects in the MediaDevice.
 *
 * Delete all MediaEntities; entities will then delete their pads,
 * and each source pad will delete links.
 *
 * After this function has been called, the media graph will be unpopulated
 * and its media objects deleted. The media device has to be populated
 * before it could be used again.
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
 * \brief Open a media device and retrieve informations from it
 *
 * The function fails if the media device is already open or if either
 * open or the media device information retrieval operations fail.
 * \return 0 for success or a negative error number otherwise
 */
int MediaDevice::open()
{
	if (fd_ != -1) {
		LOG(Error) << "MediaDevice already open";
		return -EBUSY;
	}

	int ret = ::open(devnode_.c_str(), O_RDWR);
	if (ret < 0) {
		ret = -errno;
		LOG(Error) << "Failed to open media device at " << devnode_
			   << ": " << strerror(-ret);
		return ret;
	}
	fd_ = ret;

	struct media_device_info info = { };
	ret = ioctl(fd_, MEDIA_IOC_DEVICE_INFO, &info);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Failed to get media device info "
			   << ": " << strerror(-ret);
		return ret;
	}

	driver_ = info.driver;

	return 0;
}

/**
 * \brief Close the file descriptor associated with the media device.
 *
 * After this function has been called, for the MediaDevice to be operated on,
 * the caller shall open it again.
 */
void MediaDevice::close()
{
	if (fd_ == -1)
		return;

	::close(fd_);
	fd_ = -1;
}

/**
 * \fn MediaDevice::entities()
 * \brief Retrieve the list of entities in the media graph
 * \return The list of MediaEntities registered in the MediaDevice
 */

/*
 * Add a new object to the global objects pool and fail if the object
 * has already been registered.
 */
bool MediaDevice::addObject(MediaObject *obj)
{

	if (objects_.find(obj->id()) != objects_.end()) {
		LOG(Error) << "Element with id " << obj->id()
			   << " already enumerated.";
		return false;
	}

	objects_[obj->id()] = obj;

	return true;
}

/*
 * MediaObject pool lookup by id.
 */
MediaObject *MediaDevice::object(unsigned int id)
{
	auto it = objects_.find(id);
	return (it == objects_.end()) ? nullptr : it->second;
}

/**
 * \brief Return the MediaEntity with name \a name
 * \param name The entity name
 * \return The entity with \a name
 * \return nullptr if no entity with \a name is found
 */
MediaEntity *MediaDevice::getEntityByName(const std::string &name)
{
	for (MediaEntity *e : entities_)
		if (e->name() == name)
			return e;

	return nullptr;
}

bool MediaDevice::populateLinks(const struct media_v2_topology &topology)
{
	media_v2_link *mediaLinks = reinterpret_cast<media_v2_link *>
				    (topology.ptr_links);

	for (unsigned int i = 0; i < topology.num_links; ++i) {
		/*
		 * Skip links between entities and interfaces: we only care
		 * about pad-2-pad links here.
		 */
		if ((mediaLinks[i].flags & MEDIA_LNK_FL_LINK_TYPE) ==
		    MEDIA_LNK_FL_INTERFACE_LINK)
			continue;

		/* Store references to source and sink pads in the link. */
		unsigned int source_id = mediaLinks[i].source_id;
		MediaPad *source = dynamic_cast<MediaPad *>
				   (object(source_id));
		if (!source) {
			LOG(Error) << "Failed to find pad with id: "
				   << source_id;
			return false;
		}

		unsigned int sink_id = mediaLinks[i].sink_id;
		MediaPad *sink = dynamic_cast<MediaPad *>
				 (object(sink_id));
		if (!sink) {
			LOG(Error) << "Failed to find pad with id: "
				   << sink_id;
			return false;
		}

		MediaLink *link = new MediaLink(&mediaLinks[i], source, sink);
		if (!addObject(link)) {
			delete link;
			return false;
		}

		source->addLink(link);
		sink->addLink(link);
	}

	return true;
}

bool MediaDevice::populatePads(const struct media_v2_topology &topology)
{
	media_v2_pad *mediaPads = reinterpret_cast<media_v2_pad *>
				  (topology.ptr_pads);

	for (unsigned int i = 0; i < topology.num_pads; ++i) {
		unsigned int entity_id = mediaPads[i].entity_id;

		/* Store a reference to this MediaPad in entity. */
		MediaEntity *mediaEntity = dynamic_cast<MediaEntity *>
					   (object(entity_id));
		if (!mediaEntity) {
			LOG(Error) << "Failed to find entity with id: "
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

/*
 * For each entity in the media graph create a MediaEntity and store a
 * reference in the MediaObject global pool and in the global vector of
 * entities.
 */
bool MediaDevice::populateEntities(const struct media_v2_topology &topology)
{
	media_v2_entity *mediaEntities = reinterpret_cast<media_v2_entity *>
					 (topology.ptr_entities);

	for (unsigned int i = 0; i < topology.num_entities; ++i) {
		MediaEntity *entity = new MediaEntity(&mediaEntities[i]);
		if (!addObject(entity)) {
			delete entity;
			return false;
		}

		entities_.push_back(entity);
	}

	return true;
}

/**
 * \brief Populate the media graph with media objects
 *
 * This function enumerates all media objects in the media device graph and
 * creates their MediaObject representations. All entities, pads and links are
 * stored as MediaEntity, MediaPad and MediaLink respectively, with cross-
 * references between objects. Interfaces are not processed.
 *
 * MediaEntities are stored in a global list in the MediaDevice itself to ease
 * lookup, while MediaPads are accessible from the MediaEntity they belong
 * to only and MediaLinks from the MediaPad they connect.
 *
 * \return 0 on success, a negative error code otherwise
 */
int MediaDevice::populate()
{
	struct media_v2_topology topology = { };
	struct media_v2_entity *ents = nullptr;
	struct media_v2_link *links = nullptr;
	struct media_v2_pad *pads = nullptr;
	__u64 version = -1;
	int ret;

	clear();

	/*
	 * Keep calling G_TOPOLOGY until the version number stays stable.
	 */
	while (true) {
		topology.topology_version = 0;
		topology.ptr_entities = reinterpret_cast<__u64>(ents);
		topology.ptr_links = reinterpret_cast<__u64>(links);
		topology.ptr_pads = reinterpret_cast<__u64>(pads);

		ret = ioctl(fd_, MEDIA_IOC_G_TOPOLOGY, &topology);
		if (ret < 0) {
			ret = -errno;
			LOG(Error) << "Failed to enumerate topology: "
				   << strerror(-ret);
			return ret;
		}

		if (version == topology.topology_version)
			break;

		delete[] links;
		delete[] ents;
		delete[] pads;

		ents = new media_v2_entity[topology.num_entities];
		links = new media_v2_link[topology.num_links];
		pads = new media_v2_pad[topology.num_pads];

		version = topology.topology_version;
	}

	/* Populate entities, pads and links. */
	if (populateEntities(topology) &&
	    populatePads(topology) &&
	    populateLinks(topology))
		valid_ = true;

	delete[] links;
	delete[] ents;
	delete[] pads;

	if (!valid_) {
		clear();
		return -EINVAL;
	}

	return 0;
}

/**
 * \fn MediaDevice::valid()
 * \brief Query whether the media graph is valid
 * \return true if the media graph is valid, false otherwise
 */

/**
 * \var MediaDevice::objects_
 * \brief Global map of media objects (entities, pads, links) keyed by their
 * object id.
 */

/**
 * \var MediaDevice::entities_
 * \brief Global list of media entities in the media graph
 */

} /* namespace libcamera */
