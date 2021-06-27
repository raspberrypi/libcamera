/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * frames.cpp - Intel IPU3 Frames helper
 */

#include "frames.h"

#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPU3)

IPU3Frames::IPU3Frames()
{
}

void IPU3Frames::init(const std::vector<std::unique_ptr<FrameBuffer>> &paramBuffers,
		      const std::vector<std::unique_ptr<FrameBuffer>> &statBuffers)
{
	for (const std::unique_ptr<FrameBuffer> &buffer : paramBuffers)
		availableParamBuffers_.push(buffer.get());

	for (const std::unique_ptr<FrameBuffer> &buffer : statBuffers)
		availableStatBuffers_.push(buffer.get());

	frameInfo_.clear();
}

void IPU3Frames::clear()
{
	availableParamBuffers_ = {};
	availableStatBuffers_ = {};
}

IPU3Frames::Info *IPU3Frames::create(Request *request)
{
	unsigned int id = request->sequence();

	if (availableParamBuffers_.empty()) {
		LOG(IPU3, Debug) << "Parameters buffer underrun";
		return nullptr;
	}

	if (availableStatBuffers_.empty()) {
		LOG(IPU3, Debug) << "Statistics buffer underrun";
		return nullptr;
	}

	FrameBuffer *paramBuffer = availableParamBuffers_.front();
	FrameBuffer *statBuffer = availableStatBuffers_.front();

	paramBuffer->_d()->setRequest(request);
	statBuffer->_d()->setRequest(request);

	availableParamBuffers_.pop();
	availableStatBuffers_.pop();

	/* \todo Remove the dynamic allocation of Info */
	std::unique_ptr<Info> info = std::make_unique<Info>();

	info->id = id;
	info->request = request;
	info->rawBuffer = nullptr;
	info->paramBuffer = paramBuffer;
	info->statBuffer = statBuffer;
	info->paramDequeued = false;
	info->metadataProcessed = false;

	frameInfo_[id] = std::move(info);

	return frameInfo_[id].get();
}

void IPU3Frames::remove(IPU3Frames::Info *info)
{
	/* Return params and stat buffer for reuse. */
	availableParamBuffers_.push(info->paramBuffer);
	availableStatBuffers_.push(info->statBuffer);

	/* Delete the extended frame information. */
	frameInfo_.erase(info->id);
}

bool IPU3Frames::tryComplete(IPU3Frames::Info *info)
{
	Request *request = info->request;

	if (request->hasPendingBuffers())
		return false;

	if (!info->metadataProcessed)
		return false;

	if (!info->paramDequeued)
		return false;

	remove(info);

	bufferAvailable.emit();

	return true;
}

IPU3Frames::Info *IPU3Frames::find(unsigned int id)
{
	const auto &itInfo = frameInfo_.find(id);

	if (itInfo != frameInfo_.end())
		return itInfo->second.get();

	LOG(IPU3, Fatal) << "Can't find tracking information for frame " << id;

	return nullptr;
}

IPU3Frames::Info *IPU3Frames::find(FrameBuffer *buffer)
{
	for (auto const &itInfo : frameInfo_) {
		Info *info = itInfo.second.get();

		for (auto const itBuffers : info->request->buffers())
			if (itBuffers.second == buffer)
				return info;

		if (info->rawBuffer == buffer || info->paramBuffer == buffer ||
		    info->statBuffer == buffer)
			return info;
	}

	LOG(IPU3, Fatal) << "Can't find tracking information from buffer";

	return nullptr;
}

} /* namespace libcamera */
