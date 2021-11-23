/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * frames.h - Intel IPU3 Frames helper
 */

#pragma once

#include <map>
#include <memory>
#include <queue>
#include <vector>

#include <libcamera/base/signal.h>

#include <libcamera/controls.h>

namespace libcamera {

class FrameBuffer;
class IPAProxy;
class PipelineHandler;
class Request;
class V4L2VideoDevice;
struct IPABuffer;

class IPU3Frames
{
public:
	struct Info {
		unsigned int id;
		Request *request;

		FrameBuffer *rawBuffer;
		FrameBuffer *paramBuffer;
		FrameBuffer *statBuffer;

		ControlList effectiveSensorControls;

		bool paramDequeued;
		bool metadataProcessed;
	};

	IPU3Frames();

	void init(const std::vector<std::unique_ptr<FrameBuffer>> &paramBuffers,
		  const std::vector<std::unique_ptr<FrameBuffer>> &statBuffers);
	void clear();

	Info *create(Request *request);
	void remove(Info *info);
	bool tryComplete(Info *info);

	Info *find(unsigned int id);
	Info *find(FrameBuffer *buffer);

	Signal<> bufferAvailable;

private:
	std::queue<FrameBuffer *> availableParamBuffers_;
	std::queue<FrameBuffer *> availableStatBuffers_;

	std::map<unsigned int, std::unique_ptr<Info>> frameInfo_;
};

} /* namespace libcamera */
