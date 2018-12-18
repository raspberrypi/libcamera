/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * media_device.h - Media device handler
 */
#ifndef __LIBCAMERA_MEDIA_DEVICE_H__
#define __LIBCAMERA_MEDIA_DEVICE_H__

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <linux/media.h>

#include "media_object.h"

namespace libcamera {

class MediaDevice
{
public:
	MediaDevice(const std::string &devnode);
	~MediaDevice();

	int open();
	void close();

	int populate();

	const std::string driver() const { return driver_; }
	const std::string devnode() const { return devnode_; }
	const std::vector<MediaEntity *> &entities() const { return entities_; }

private:
	std::string driver_;
	std::string devnode_;
	int fd_;

	std::map<unsigned int, MediaObject *> objects_;
	MediaObject *object(unsigned int id);
	int addObject(MediaObject *obj);
	void clear();

	std::vector<MediaEntity *> entities_;
	MediaEntity *getEntityByName(const std::string &name);

	void populateEntities(const struct media_v2_topology &topology);
	int populatePads(const struct media_v2_topology &topology);
	int populateLinks(const struct media_v2_topology &topology);
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_MEDIA_DEVICE_H__ */
