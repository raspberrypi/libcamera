/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * framebuffer.h - Internal frame buffer handling
 */

#pragma once

#include <memory>
#include <utility>

#include <libcamera/base/class.h>

#include <libcamera/fence.h>
#include <libcamera/framebuffer.h>

namespace libcamera {

class FrameBuffer::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(FrameBuffer)

public:
	Private(const std::vector<Plane> &planes, uint64_t cookie = 0);
	virtual ~Private();

	void setRequest(Request *request) { request_ = request; }
	bool isContiguous() const { return isContiguous_; }

	Fence *fence() const { return fence_.get(); }
	void setFence(std::unique_ptr<Fence> fence) { fence_ = std::move(fence); }

	void cancel() { metadata_.status = FrameMetadata::FrameCancelled; }

	FrameMetadata &metadata() { return metadata_; }

private:
	std::vector<Plane> planes_;
	FrameMetadata metadata_;
	uint64_t cookie_;

	std::unique_ptr<Fence> fence_;
	Request *request_;
	bool isContiguous_;
};

} /* namespace libcamera */
