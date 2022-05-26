/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * media_device.h - Media device handler
 */

#pragma once

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <linux/media.h>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/unique_fd.h>

#include "libcamera/internal/media_object.h"

namespace libcamera {

class MediaDevice : protected Loggable
{
public:
	MediaDevice(const std::string &deviceNode);
	~MediaDevice();

	bool acquire();
	void release();
	bool busy() const { return acquired_; }

	bool lock();
	void unlock();

	int populate();
	bool isValid() const { return valid_; }

	const std::string &driver() const { return driver_; }
	const std::string &deviceNode() const { return deviceNode_; }
	const std::string &model() const { return model_; }
	unsigned int version() const { return version_; }
	unsigned int hwRevision() const { return hwRevision_; }

	const std::vector<MediaEntity *> &entities() const { return entities_; }
	MediaEntity *getEntityByName(const std::string &name) const;

	MediaLink *link(const std::string &sourceName, unsigned int sourceIdx,
			const std::string &sinkName, unsigned int sinkIdx);
	MediaLink *link(const MediaEntity *source, unsigned int sourceIdx,
			const MediaEntity *sink, unsigned int sinkIdx);
	MediaLink *link(const MediaPad *source, const MediaPad *sink);
	int disableLinks();

	Signal<> disconnected;

protected:
	std::string logPrefix() const override;

private:
	int open();
	void close();

	MediaObject *object(unsigned int id);
	bool addObject(MediaObject *object);
	void clear();

	struct media_v2_interface *findInterface(const struct media_v2_topology &topology,
						 unsigned int entityId);
	bool populateEntities(const struct media_v2_topology &topology);
	bool populatePads(const struct media_v2_topology &topology);
	bool populateLinks(const struct media_v2_topology &topology);
	void fixupEntityFlags(struct media_v2_entity *entity);

	friend int MediaLink::setEnabled(bool enable);
	int setupLink(const MediaLink *link, unsigned int flags);

	std::string driver_;
	std::string deviceNode_;
	std::string model_;
	unsigned int version_;
	unsigned int hwRevision_;

	UniqueFD fd_;
	bool valid_;
	bool acquired_;

	std::map<unsigned int, MediaObject *> objects_;
	std::vector<MediaEntity *> entities_;
};

} /* namespace libcamera */
