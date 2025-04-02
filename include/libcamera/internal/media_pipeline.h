/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Media pipeline support
 */

#pragma once

#include <list>
#include <string>

#include <libcamera/base/log.h>

namespace libcamera {

class CameraSensor;
class MediaEntity;
class MediaLink;
class MediaPad;
struct V4L2SubdeviceFormat;

class MediaPipeline
{
public:
	int init(MediaEntity *source, std::string_view sink);
	int initLinks();
	int configure(CameraSensor *sensor, V4L2SubdeviceFormat *);

private:
	struct Entity {
		/* The media entity, always valid. */
		MediaEntity *entity;
		/*
		 * Whether or not the entity is a subdev that supports the
		 * routing API.
		 */
		bool supportsRouting;
		/*
		 * The local sink pad connected to the upstream entity, null for
		 * the camera sensor at the beginning of the pipeline.
		 */
		const MediaPad *sink;
		/*
		 * The local source pad connected to the downstream entity, null
		 * for the video node at the end of the pipeline.
		 */
		const MediaPad *source;
		/*
		 * The link on the source pad, to the downstream entity, null
		 * for the video node at the end of the pipeline.
		 */
		MediaLink *sourceLink;
	};

	std::list<Entity> entities_;
};

} /* namespace libcamera */
