/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * media_object.h - Media Device objects: entities, pads and links.
 */
#ifndef __LIBCAMERA_MEDIA_OBJECT_H__
#define __LIBCAMERA_MEDIA_OBJECT_H__

#include <string>
#include <vector>

#include <linux/media.h>

namespace libcamera {

class MediaDevice;
class MediaEntity;
class MediaPad;

class MediaObject
{
public:
	unsigned int id() const { return id_; }

protected:
	friend class MediaDevice;

	MediaObject(unsigned int id) : id_(id) { }
	virtual ~MediaObject() { }

	unsigned int id_;
};

class MediaLink : public MediaObject
{
public:
	MediaPad *source() const { return source_; }
	MediaPad *sink() const { return sink_; }
	unsigned int flags() const { return flags_; }

private:
	friend class MediaDevice;

	MediaLink(const struct media_v2_link *link,
		  MediaPad *source, MediaPad *sink);
	MediaLink(const MediaLink &) = delete;
	~MediaLink() { }

	MediaPad *source_;
	MediaPad *sink_;
	unsigned int flags_;
};

class MediaPad : public MediaObject
{
public:
	unsigned int index() const { return index_; }
	MediaEntity *entity() const { return entity_; }
	unsigned int flags() const { return flags_; }
	const std::vector<MediaLink *> &links() const { return links_; }

	void addLink(MediaLink *link);

private:
	friend class MediaDevice;

	MediaPad(const struct media_v2_pad *pad, MediaEntity *entity);
	MediaPad(const MediaPad &) = delete;
	~MediaPad();

	unsigned int index_;
	MediaEntity *entity_;
	unsigned int flags_;

	std::vector<MediaLink *> links_;
};

class MediaEntity : public MediaObject
{
public:
	const std::string &name() const { return name_; }

	const std::vector<MediaPad *> &pads() const { return pads_; }

	const MediaPad *getPadByIndex(unsigned int index) const;
	const MediaPad *getPadById(unsigned int id) const;

private:
	friend class MediaDevice;

	MediaEntity(const struct media_v2_entity *entity);
	MediaEntity(const MediaEntity &) = delete;
	~MediaEntity();

	std::string name_;
	std::string devnode_;

	std::vector<MediaPad *> pads_;

	void addPad(MediaPad *pad);
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_MEDIA_OBJECT_H__ */
