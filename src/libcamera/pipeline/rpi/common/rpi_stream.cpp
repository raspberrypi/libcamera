/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * Raspberry Pi device stream abstraction class.
 */
#include "rpi_stream.h"

#include <algorithm>
#include <tuple>
#include <utility>

#include <libcamera/base/log.h>

/* Maximum number of buffer slots to allocate in the V4L2 device driver. */
static constexpr unsigned int maxV4L2BufferCount = 32;

namespace libcamera {

LOG_DEFINE_CATEGORY(RPISTREAM)

namespace RPi {

const BufferObject Stream::errorBufferObject{ nullptr, false };

void Stream::setFlags(StreamFlags flags)
{
	/* We don't want dynamic mmapping. */
	ASSERT(!(flags & StreamFlag::RequiresMmap));

	flags_ |= flags;

	/* Import streams cannot be external. */
	ASSERT(!(flags_ & StreamFlag::External) || !(flags_ & StreamFlag::ImportOnly));
}

void Stream::clearFlags(StreamFlags flags)
{
	/* We don't want dynamic mmapping. */
	ASSERT(!(flags & StreamFlag::RequiresMmap));

	flags_ &= ~flags;
}

RPi::Stream::StreamFlags Stream::getFlags() const
{
	return flags_;
}

V4L2VideoDevice *Stream::dev() const
{
	return dev_.get();
}

const std::string &Stream::name() const
{
	return name_;
}

unsigned int Stream::swDownscale() const
{
	return swDownscale_;
}

void Stream::setSwDownscale(unsigned int swDownscale)
{
	swDownscale_ = swDownscale;
}

void Stream::resetBuffers()
{
	/* Add all internal buffers to the queue of usable buffers. */
	availableBuffers_ = {};
	for (auto const &buffer : internalBuffers_)
		availableBuffers_.push(buffer.get());
}

void Stream::setExportedBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	for (auto const &buffer : *buffers)
		bufferEmplace(++id_, buffer.get());
}

const BufferMap &Stream::getBuffers() const
{
	return bufferMap_;
}

unsigned int Stream::getBufferId(FrameBuffer *buffer) const
{
	if (flags_ & StreamFlag::ImportOnly)
		return 0;

	/* Find the buffer in the map, and return the buffer id. */
	auto it = std::find_if(bufferMap_.begin(), bufferMap_.end(),
			       [&buffer](auto const &p) { return p.second.buffer == buffer; });

	if (it == bufferMap_.end())
		return 0;

	return it->first;
}

void Stream::setExportedBuffer(FrameBuffer *buffer)
{
	bufferEmplace(++id_, buffer);
}

int Stream::prepareBuffers(unsigned int count)
{
	int ret;

	if (!(flags_ & StreamFlag::ImportOnly)) {
		/* Export some frame buffers for internal use. */
		ret = dev_->exportBuffers(count, &internalBuffers_);
		if (ret < 0)
			return ret;

		/* Add these exported buffers to the internal/external buffer list. */
		setExportedBuffers(&internalBuffers_);
		resetBuffers();
	}

	return dev_->importBuffers(maxV4L2BufferCount);
}

int Stream::queueBuffer(FrameBuffer *buffer)
{
	/*
	 * A nullptr buffer implies an external stream, but no external
	 * buffer has been supplied in the Request. So, pick one from the
	 * availableBuffers_ queue.
	 */
	if (!buffer) {
		if (availableBuffers_.empty()) {
			LOG(RPISTREAM, Debug) << "No buffers available for "
					      << name_;
			/*
			 * Note that we need to queue an internal buffer as soon
			 * as one becomes available.
			 */
			requestBuffers_.push(nullptr);
			return 0;
		}

		buffer = availableBuffers_.front();
		availableBuffers_.pop();
	}

	/*
	 * If no earlier requests are pending to be queued we can go ahead and
	 * queue this buffer into the device.
	 */
	if (requestBuffers_.empty())
		return queueToDevice(buffer);

	/*
	 * There are earlier Request buffers to be queued, so this buffer must go
	 * on the waiting list.
	 */
	requestBuffers_.push(buffer);

	return 0;
}

void Stream::returnBuffer(FrameBuffer *buffer)
{
	if (!(flags_ & StreamFlag::External) && !(flags_ & StreamFlag::Recurrent)) {
		/* For internal buffers, simply requeue back to the device. */
		queueToDevice(buffer);
		return;
	}

	/* Push this buffer back into the queue to be used again. */
	availableBuffers_.push(buffer);

	/*
	 * Do we have any Request buffers that are waiting to be queued?
	 * If so, do it now as availableBuffers_ will not be empty.
	 */
	while (!requestBuffers_.empty()) {
		FrameBuffer *requestBuffer = requestBuffers_.front();

		if (!requestBuffer) {
			/*
			 * We want to queue an internal buffer, but none
			 * are available. Can't do anything, quit the loop.
			 */
			if (availableBuffers_.empty())
				break;

			/*
			 * We want to queue an internal buffer, and at least one
			 * is available.
			 */
			requestBuffer = availableBuffers_.front();
			availableBuffers_.pop();
		}

		requestBuffers_.pop();
		queueToDevice(requestBuffer);
	}
}

const BufferObject &Stream::getBuffer(unsigned int id)
{
	auto const &it = bufferMap_.find(id);
	if (it == bufferMap_.end())
		return errorBufferObject;

	return it->second;
}

const BufferObject &Stream::acquireBuffer()
{
	/* No id provided, so pick up the next available buffer if possible. */
	if (availableBuffers_.empty())
		return errorBufferObject;

	unsigned int id = getBufferId(availableBuffers_.front());
	availableBuffers_.pop();

	return getBuffer(id);
}

int Stream::queueAllBuffers()
{
	int ret;

	if ((flags_ & StreamFlag::External) || (flags_ & StreamFlag::Recurrent))
		return 0;

	while (!availableBuffers_.empty()) {
		ret = queueBuffer(availableBuffers_.front());
		if (ret < 0)
			return ret;

		availableBuffers_.pop();
	}

	return 0;
}

void Stream::releaseBuffers()
{
	dev_->releaseBuffers();
	clearBuffers();
}

void Stream::bufferEmplace(unsigned int id, FrameBuffer *buffer)
{
	if (flags_ & StreamFlag::RequiresMmap)
		bufferMap_.emplace(std::piecewise_construct, std::forward_as_tuple(id),
				   std::forward_as_tuple(buffer, true));
	else
		bufferMap_.emplace(std::piecewise_construct, std::forward_as_tuple(id),
				   std::forward_as_tuple(buffer, false));
}

void Stream::clearBuffers()
{
	availableBuffers_ = std::queue<FrameBuffer *>{};
	requestBuffers_ = std::queue<FrameBuffer *>{};
	internalBuffers_.clear();
	bufferMap_.clear();
	id_ = 0;
}

int Stream::queueToDevice(FrameBuffer *buffer)
{
	LOG(RPISTREAM, Debug) << "Queuing buffer " << getBufferId(buffer)
			      << " for " << name_;

	int ret = dev_->queueBuffer(buffer);
	if (ret)
		LOG(RPISTREAM, Error) << "Failed to queue buffer for "
				      << name_;
	return ret;
}

} /* namespace RPi */

} /* namespace libcamera */
