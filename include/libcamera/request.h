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
#include <unordered_set>

#include <libcamera/controls.h>
#include <libcamera/signal.h>

namespace libcamera {

class Buffer;
class Camera;
class CameraControlValidator;
class Stream;

class Request
{
public:
	enum Status {
		RequestPending,
		RequestComplete,
		RequestCancelled,
	};

	Request(Camera *camera, uint64_t cookie = 0);
	Request(const Request &) = delete;
	Request &operator=(const Request &) = delete;
	~Request();

	ControlList &controls() { return *controls_; }
	const std::map<Stream *, Buffer *> &buffers() const { return bufferMap_; }
	int addBuffer(std::unique_ptr<Buffer> buffer);
	Buffer *findBuffer(Stream *stream) const;

	uint64_t cookie() const { return cookie_; }
	Status status() const { return status_; }

	bool hasPendingBuffers() const { return !pending_.empty(); }

private:
	friend class Camera;
	friend class PipelineHandler;

	int prepare();
	void complete();

	bool completeBuffer(Buffer *buffer);

	Camera *camera_;
	CameraControlValidator *validator_;
	ControlList *controls_;
	std::map<Stream *, Buffer *> bufferMap_;
	std::unordered_set<Buffer *> pending_;

	const uint64_t cookie_;
	Status status_;
	bool cancelled_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_REQUEST_H__ */
