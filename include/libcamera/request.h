/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * request.h - Capture request handling
 */
#ifndef __LIBCAMERA_REQUEST_H__
#define __LIBCAMERA_REQUEST_H__

#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <unordered_set>

#include <libcamera/base/class.h>
#include <libcamera/base/signal.h>

#include <libcamera/controls.h>

namespace libcamera {

class Camera;
class CameraControlValidator;
class FrameBuffer;
class Stream;

class Request
{
public:
	enum Status {
		RequestPending,
		RequestComplete,
		RequestCancelled,
	};

	enum ReuseFlag {
		Default = 0,
		ReuseBuffers = (1 << 0),
	};

	using BufferMap = std::map<const Stream *, FrameBuffer *>;

	Request(Camera *camera, uint64_t cookie = 0);
	~Request();

	void reuse(ReuseFlag flags = Default);

	ControlList &controls() { return *controls_; }
	ControlList &metadata() { return *metadata_; }
	const BufferMap &buffers() const { return bufferMap_; }
	int addBuffer(const Stream *stream, FrameBuffer *buffer);
	FrameBuffer *findBuffer(const Stream *stream) const;

	uint32_t sequence() const { return sequence_; }
	uint64_t cookie() const { return cookie_; }
	Status status() const { return status_; }

	bool hasPendingBuffers() const { return !pending_.empty(); }

	std::string toString() const;

private:
	LIBCAMERA_DISABLE_COPY(Request)

	friend class PipelineHandler;

	void complete();
	void cancel();

	bool completeBuffer(FrameBuffer *buffer);

	Camera *camera_;
	CameraControlValidator *validator_;
	ControlList *controls_;
	ControlList *metadata_;
	BufferMap bufferMap_;
	std::unordered_set<FrameBuffer *> pending_;

	uint32_t sequence_;
	const uint64_t cookie_;
	Status status_;
	bool cancelled_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_REQUEST_H__ */
