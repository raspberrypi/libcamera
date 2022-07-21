/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * fc_queue.h - IPA Frame context queue
 */

#pragma once

#include <stdint.h>
#include <vector>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(FCQueue)

namespace ipa {

template<typename FrameContext>
class FCQueue;

struct FrameContext {
private:
	template<typename T> friend class FCQueue;
	uint32_t frame;
};

template<typename FrameContext>
class FCQueue
{
public:
	FCQueue(unsigned int size)
		: contexts_(size)
	{
	}

	void clear()
	{
		for (FrameContext &ctx : contexts_)
			ctx.frame = 0;
	}

	FrameContext &alloc(const uint32_t frame)
	{
		FrameContext &frameContext = contexts_[frame % contexts_.size()];

		/*
		 * Do not re-initialise if a get() call has already fetched this
		 * frame context to preseve the context.
		 *
		 * \todo If the the sequence number of the context to initialise
		 * is smaller than the sequence number of the queue slot to use,
		 * it means that we had a serious request underrun and more
		 * frames than the queue size has been produced since the last
		 * time the application has queued a request. Does this deserve
		 * an error condition ?
		 */
		if (frame != 0 && frame <= frameContext.frame)
			LOG(FCQueue, Warning)
				<< "Frame " << frame << " already initialised";
		else
			init(frameContext, frame);

		return frameContext;
	}

	FrameContext &get(uint32_t frame)
	{
		FrameContext &frameContext = contexts_[frame % contexts_.size()];

		/*
		 * If the IPA algorithms try to access a frame context slot which
		 * has been already overwritten by a newer context, it means the
		 * frame context queue has overflowed and the desired context
		 * has been forever lost. The pipeline handler shall avoid
		 * queueing more requests to the IPA than the frame context
		 * queue size.
		 */
		if (frame < frameContext.frame)
			LOG(FCQueue, Fatal) << "Frame context for " << frame
					    << " has been overwritten by "
					    << frameContext.frame;

		if (frame == frameContext.frame)
			return frameContext;

		/*
		 * The frame context has been retrieved before it was
		 * initialised through the initialise() call. This indicates an
		 * algorithm attempted to access a Frame context before it was
		 * queued to the IPA. Controls applied for this request may be
		 * left unhandled.
		 *
		 * \todo Set an error flag for per-frame control errors.
		 */
		LOG(FCQueue, Warning)
			<< "Obtained an uninitialised FrameContext for " << frame;

		init(frameContext, frame);

		return frameContext;
	}

private:
	void init(FrameContext &frameContext, const uint32_t frame)
	{
		frameContext = {};
		frameContext.frame = frame;
	}

	std::vector<FrameContext> contexts_;
};

} /* namespace ipa */

} /* namespace libcamera */
