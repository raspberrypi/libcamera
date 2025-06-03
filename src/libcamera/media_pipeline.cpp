/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Media pipeline support
 */

#include "libcamera/internal/media_pipeline.h"

#include <algorithm>
#include <errno.h>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <linux/media.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"
#include "libcamera/internal/v4l2_subdevice.h"

/**
 * \file media_pipeline.h
 * \brief Provide a representation of a pipeline of devices using the Media
 * Controller.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(MediaPipeline)

/**
 * \class MediaPipeline
 * \brief The MediaPipeline represents a set of entities that together form a
 * data path for stream data.
 *
 * A MediaPipeline instance is constructed from a sink and a source between
 * two entities in a media graph.
 */

/**
 * \brief Retrieve all source pads connected to a sink pad through active routes
 *
 * Examine the entity using the V4L2 Subdevice Routing API to collect all the
 * source pads which are connected with an active route to the sink pad.
 *
 * \return A vector of source MediaPads
 */
static std::vector<const MediaPad *> routedSourcePads(MediaPad *sink)
{
	MediaEntity *entity = sink->entity();
	std::unique_ptr<V4L2Subdevice> subdev =
		std::make_unique<V4L2Subdevice>(entity);

	int ret = subdev->open();
	if (ret < 0)
		return {};

	V4L2Subdevice::Routing routing = {};
	ret = subdev->getRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret < 0)
		return {};

	std::vector<const MediaPad *> pads;

	for (const V4L2Subdevice::Route &route : routing) {
		if (sink->index() != route.sink.pad ||
		    !(route.flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		const MediaPad *pad = entity->getPadByIndex(route.source.pad);
		if (!pad) {
			LOG(MediaPipeline, Error)
				<< "Entity " << entity->name()
				<< " has invalid route source pad "
				<< route.source.pad;
			return {};
		}

		pads.push_back(pad);
	}

	return pads;
}

/**
 * \brief Find the path from source to sink
 *
 * Starting from a source entity, determine the shortest path to the target
 * described by \a sink.
 *
 * If \a sink can not be found, or a route from source to sink can not be
 * achieved an error of -ENOLINK will be returned.
 *
 * When successful, the MediaPipeline will internally store the representation
 * of entities and links to describe the path between the two entities.
 *
 * \return 0 on success, a negative errno otherwise
 */
int MediaPipeline::init(MediaEntity *source, std::string_view sink)
{
	/*
	 * Find the shortest path between from the Camera Sensor and the
	 * target entity.
	 */
	std::unordered_set<MediaEntity *> visited;
	std::queue<std::tuple<MediaEntity *, MediaPad *>> queue;

	/* Remember at each entity where we came from. */
	std::unordered_map<MediaEntity *, Entity> parents;
	MediaEntity *entity = nullptr;
	MediaEntity *target = nullptr;
	MediaPad *sinkPad;

	queue.push({ source, nullptr });

	while (!queue.empty()) {
		std::tie(entity, sinkPad) = queue.front();
		queue.pop();

		/* Found the target device. */
		if (entity->name() == sink) {
			LOG(MediaPipeline, Debug)
				<< "Found Pipeline target " << entity->name();
			target = entity;
			break;
		}

		visited.insert(entity);

		/*
		 * Add direct downstream entities to the search queue. If the
		 * current entity supports the subdev internal routing API,
		 * restrict the search to downstream entities reachable through
		 * active routes.
		 */

		std::vector<const MediaPad *> pads;
		bool supportsRouting = false;

		if (sinkPad) {
			pads = routedSourcePads(sinkPad);
			if (!pads.empty())
				supportsRouting = true;
		}

		if (pads.empty()) {
			for (const MediaPad *pad : entity->pads()) {
				if (!(pad->flags() & MEDIA_PAD_FL_SOURCE))
					continue;
				pads.push_back(pad);
			}
		}

		for (const MediaPad *pad : pads) {
			for (MediaLink *link : pad->links()) {
				MediaEntity *next = link->sink()->entity();
				if (visited.find(next) == visited.end()) {
					queue.push({ next, link->sink() });

					Entity e{ entity, supportsRouting, sinkPad, pad, link };
					parents.insert({ next, e });
				}
			}
		}
	}

	if (!target) {
		LOG(MediaPipeline, Error)
			<< "Failed to connect " << source->name()
			<< " to " << sink;
		return -ENOLINK;
	}

	/*
	 * With the parents, we can follow back our way from the capture device
	 * to the sensor. Store all the entities in the pipeline, from the
	 * camera sensor to the video node, in entities_.
	 */
	entities_.push_front({ entity, false, sinkPad, nullptr, nullptr });

	for (auto it = parents.find(entity); it != parents.end();
	     it = parents.find(entity)) {
		const Entity &e = it->second;
		entities_.push_front(e);
		entity = e.entity;
	}

	LOG(MediaPipeline, Info)
		<< "Found pipeline: "
		<< utils::join(entities_, " -> ",
			       [](const Entity &e) {
				       std::string s = "[";
				       if (e.sink)
					       s += std::to_string(e.sink->index()) + "|";
				       s += e.entity->name();
				       if (e.source)
					       s += "|" + std::to_string(e.source->index());
				       s += "]";
				       return s;
			       });

	return 0;
}

/**
 * \brief Initialise and enable all links through the MediaPipeline
 * \return 0 on success, or a negative errno otherwise
 */
int MediaPipeline::initLinks()
{
	int ret = 0;

	MediaLink *sinkLink = nullptr;
	for (Entity &e : entities_) {
		/* Sensor entities have no connected sink. */
		if (!sinkLink) {
			sinkLink = e.sourceLink;
			continue;
		}

		LOG(MediaPipeline, Debug) << "Enabling : " << *sinkLink;

		if (!(sinkLink->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = sinkLink->setEnabled(true);
			if (ret < 0)
				return ret;
		}

		sinkLink = e.sourceLink;
	}

	return ret;
}

/**
 * \brief Configure the entities of this MediaPipeline
 *
 * Propagate formats through each of the entities of the Pipeline, validating
 * that each one was not adjusted by the driver from the desired format.
 *
 * \return 0 on success or a negative errno otherwise
 */
int MediaPipeline::configure(CameraSensor *sensor, V4L2SubdeviceFormat *format)
{
	int ret;

	for (const Entity &e : entities_) {
		/* The sensor is configured through the CameraSensor */
		if (!e.sourceLink)
			break;

		MediaLink *link = e.sourceLink;
		MediaPad *source = link->source();
		MediaPad *sink = link->sink();

		/* 'format' already contains the sensor configuration */
		if (source->entity() != sensor->entity()) {
			/* \todo Add MediaDevice cache to reduce FD pressure */
			V4L2Subdevice subdev(source->entity());
			ret = subdev.open();
			if (ret)
				return ret;

			ret = subdev.getFormat(source->index(), format);
			if (ret < 0)
				return ret;
		}

		V4L2SubdeviceFormat sourceFormat = *format;
		/* \todo Add MediaDevice cache to reduce FD pressure */
		V4L2Subdevice subdev(sink->entity());
		ret = subdev.open();
		if (ret)
			return ret;

		ret = subdev.setFormat(sink->index(), format);
		if (ret < 0)
			return ret;

		if (format->code != sourceFormat.code ||
		    format->size != sourceFormat.size) {
			LOG(MediaPipeline, Debug)
				<< "Source '" << *source
				<< " produces " << sourceFormat
				<< ", sink '" << *sink
				<< " requires " << *format;
			return -EINVAL;
		}

		LOG(MediaPipeline, Debug)
			<< "Link " << *link << " configured with format "
			<< *format;
	}

	return 0;
}

} /* namespace libcamera */
