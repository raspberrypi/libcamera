/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * Media Device objects: entities, pads and links.
 */

#pragma once

#include <string>
#include <vector>

#include <linux/media.h>

#include <libcamera/base/class.h>

namespace libcamera {

class MediaDevice;
class MediaEntity;
class MediaPad;

class MediaObject
{
public:
	MediaDevice *device() { return dev_; }
	const MediaDevice *device() const { return dev_; }
	unsigned int id() const { return id_; }

protected:
	friend class MediaDevice;

	MediaObject(MediaDevice *dev, unsigned int id)
		: dev_(dev), id_(id)
	{
	}
	virtual ~MediaObject() = default;

	MediaDevice *dev_;
	unsigned int id_;
};

class MediaLink : public MediaObject
{
public:
	MediaPad *source() const { return source_; }
	MediaPad *sink() const { return sink_; }
	unsigned int flags() const { return flags_; }
	int setEnabled(bool enable);

	std::string toString() const;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(MediaLink)

	friend class MediaDevice;

	MediaLink(const struct media_v2_link *link,
		  MediaPad *source, MediaPad *sink);

	MediaPad *source_;
	MediaPad *sink_;
	unsigned int flags_;
};

std::ostream &operator<<(std::ostream &out, const MediaLink &link);

class MediaPad : public MediaObject
{
public:
	unsigned int index() const { return index_; }
	MediaEntity *entity() const { return entity_; }
	unsigned int flags() const { return flags_; }
	const std::vector<MediaLink *> &links() const { return links_; }

	void addLink(MediaLink *link);

	std::string toString() const;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(MediaPad)

	friend class MediaDevice;

	MediaPad(const struct media_v2_pad *pad, MediaEntity *entity);

	unsigned int index_;
	MediaEntity *entity_;
	unsigned int flags_;

	std::vector<MediaLink *> links_;
};

std::ostream &operator<<(std::ostream &out, const MediaPad &pad);

class MediaEntity : public MediaObject
{
public:
	enum class Type {
		Invalid,
		MediaEntity,
		V4L2Subdevice,
		V4L2VideoDevice,
	};

	const std::string &name() const { return name_; }
	unsigned int function() const { return function_; }
	unsigned int flags() const { return flags_; }
	Type type() const { return type_; }
	const std::string &deviceNode() const { return deviceNode_; }
	unsigned int deviceMajor() const { return major_; }
	unsigned int deviceMinor() const { return minor_; }

	const std::vector<MediaPad *> &pads() const { return pads_; }
	const std::vector<MediaEntity *> &ancillaryEntities() const { return ancillaryEntities_; }

	const MediaPad *getPadByIndex(unsigned int index) const;
	const MediaPad *getPadById(unsigned int id) const;

	int setDeviceNode(const std::string &deviceNode);

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(MediaEntity)

	friend class MediaDevice;

	MediaEntity(MediaDevice *dev, const struct media_v2_entity *entity,
		    const struct media_v2_interface *iface);

	void addPad(MediaPad *pad);

	void addAncillaryEntity(MediaEntity *ancillaryEntity);

	std::string name_;
	unsigned int function_;
	unsigned int flags_;
	Type type_;
	std::string deviceNode_;
	unsigned int major_;
	unsigned int minor_;

	std::vector<MediaPad *> pads_;
	std::vector<MediaEntity *> ancillaryEntities_;
};

} /* namespace libcamera */
