/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * rpi_stream.cpp - Raspberry Pi device stream abstraction class.
 */
#include "rpi_stream.h"

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(RPISTREAM)

namespace RPi {

V4L2VideoDevice *Stream::dev() const
{
	return dev_.get();
}

std::string Stream::name() const
{
	return name_;
}

void Stream::resetBuffers()
{
	/* Add all internal buffers to the queue of usable buffers. */
	availableBuffers_ = {};
	for (auto const &buffer : internalBuffers_)
		availableBuffers_.push(buffer.get());
}

void Stream::setExternal(bool external)
{
	/* Import streams cannot be external. */
	ASSERT(!external || !importOnly_);
	external_ = external;
}

bool Stream::isExternal() const
{
	return external_;
}

void Stream::setExportedBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	for (auto const &buffer : *buffers)
		bufferMap_.emplace(id_.get(), buffer.get());
}

const BufferMap &Stream::getBuffers() const
{
	return bufferMap_;
}

int Stream::getBufferId(FrameBuffer *buffer) const
{
	if (importOnly_)
		return -1;

	/* Find the buffer in the map, and return the buffer id. */
	auto it = std::find_if(bufferMap_.begin(), bufferMap_.end(),
			       [&buffer](auto const &p) { return p.second == buffer; });

	if (it == bufferMap_.end())
		return -1;

	return it->first;
}

void Stream::setExternalBuffer(FrameBuffer *buffer)
{
	bufferMap_.emplace(BufferMask::MaskExternalBuffer | id_.get(), buffer);
}

void Stream::removeExternalBuffer(FrameBuffer *buffer)
{
	int id = getBufferId(buffer);

	/* Ensure we have this buffer in the stream, and it is marked external. */
	ASSERT(id != -1 && (id & BufferMask::MaskExternalBuffer));
	bufferMap_.erase(id);
}

int Stream::prepareBuffers(unsigned int count)
{
	int ret;

	if (!importOnly_) {
		if (count) {
			/* Export some frame buffers for internal use. */
			ret = dev_->exportBuffers(count, &internalBuffers_);
			if (ret < 0)
				return ret;

			/* Add these exported buffers to the internal/external buffer list. */
			setExportedBuffers(&internalBuffers_);
			resetBuffers();
		}

		/* We must import all internal/external exported buffers. */
		count = bufferMap_.size();
	}

	/*
	 * If this is an external stream, we must allocate slots for buffers that
	 * might be externally allocated. We have no indication of how many buffers
	 * may be used, so this might overallocate slots in the buffer cache.
	 * Similarly, if this stream is only importing buffers, we do the same.
	 *
	 * \todo Find a better heuristic, or, even better, an exact solution to
	 * this issue.
	 */
	if (isExternal() || importOnly_)
		count = count * 2;

	return dev_->importBuffers(count);
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
	if (!external_) {
		/* For internal buffers, simply requeue back to the device. */
		queueToDevice(buffer);
		return;
	}

	/* Push this buffer back into the queue to be used again. */
	availableBuffers_.push(buffer);

	/* Allow the buffer id to be reused. */
	id_.release(getBufferId(buffer));

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

int Stream::queueAllBuffers()
{
	int ret;

	if (external_)
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

void Stream::clearBuffers()
{
	availableBuffers_ = std::queue<FrameBuffer *>{};
	requestBuffers_ = std::queue<FrameBuffer *>{};
	internalBuffers_.clear();
	bufferMap_.clear();
	id_.reset();
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
