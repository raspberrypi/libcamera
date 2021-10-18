/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * request.h - Request class private data
 */
#ifndef __LIBCAMERA_INTERNAL_REQUEST_H__
#define __LIBCAMERA_INTERNAL_REQUEST_H__

#include <memory>

#include <libcamera/request.h>

namespace libcamera {

class Camera;
class FrameBuffer;

class Request::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(Request)

public:
	Private(Camera *camera);
	~Private();

	Camera *camera() const { return camera_; }
	bool hasPendingBuffers() const;

	bool completeBuffer(FrameBuffer *buffer);
	void complete();
	void cancel();
	void reuse();

private:
	friend class PipelineHandler;

	void doCancelRequest();

	Camera *camera_;
	bool cancelled_;
	uint32_t sequence_ = 0;

	std::unordered_set<FrameBuffer *> pending_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_REQUEST_H__ */
