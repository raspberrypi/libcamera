/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpi_stream.cpp - Raspberry Pi device stream abstraction class.
 */
#include "rpi_stream.h"

#include "libcamera/internal/log.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(RPISTREAM)

namespace RPi {

V4L2VideoDevice *RPiStream::dev() const
{
	return dev_.get();
}

std::string RPiStream::name() const
{
	return name_;
}

void RPiStream::reset()
{
	external_ = false;
	internalBuffers_.clear();
}

bool RPiStream::isImporter() const
{
	return importOnly_;
}

void RPiStream::setExternal(bool external)
{
	external_ = external;
}

bool RPiStream::isExternal() const
{
	/*
	 * Import streams cannot be external.
	 *
	 * RAW capture is a special case where we simply copy the RAW
	 * buffer out of the request. All other buffer handling happens
	 * as if the stream is internal.
	 */
	return external_ && !importOnly_;
}

void RPiStream::setExternalBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	externalBuffers_ = buffers;
}

const std::vector<std::unique_ptr<FrameBuffer>> *RPiStream::getBuffers() const
{
	return external_ ? externalBuffers_ : &internalBuffers_;
}

bool RPiStream::findFrameBuffer(FrameBuffer *buffer) const
{
	auto start = external_ ? externalBuffers_->begin() : internalBuffers_.begin();
	auto end = external_ ? externalBuffers_->end() : internalBuffers_.end();

	if (importOnly_)
		return false;

	if (std::find_if(start, end,
			 [buffer](std::unique_ptr<FrameBuffer> const &ref) { return ref.get() == buffer; }) != end)
		return true;

	return false;
}

int RPiStream::importBuffers(unsigned int count)
{
	return dev_->importBuffers(count);
}

int RPiStream::allocateBuffers(unsigned int count)
{
	return dev_->allocateBuffers(count, &internalBuffers_);
}

int RPiStream::queueBuffers()
{
	if (external_)
		return 0;

	for (auto &b : internalBuffers_) {
		int ret = dev_->queueBuffer(b.get());
		if (ret) {
			LOG(RPISTREAM, Error) << "Failed to queue buffers for "
					      << name_;
			return ret;
		}
	}

	return 0;
}

void RPiStream::releaseBuffers()
{
	dev_->releaseBuffers();
	if (!external_ && !importOnly_)
		internalBuffers_.clear();
}

} /* namespace RPi */

} /* namespace libcamera */
