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
	struct Entity {
		MediaEntity *entity;
		bool supportsRouting;
		const MediaPad *sink;
		const MediaPad *source;
		MediaLink *sourceLink;
	};

	int init(MediaEntity *source, std::string_view sink);
	int initLinks();
	int configure(CameraSensor *sensor, V4L2SubdeviceFormat *);

	const std::list<Entity> &entities() const { return entities_; }

private:
	std::list<Entity> entities_;
};

} /* namespace libcamera */
