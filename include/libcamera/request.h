/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * request.h - Capture request handling
 */
#ifndef __LIBCAMERA_REQUEST_H__
#define __LIBCAMERA_REQUEST_H__

#include <map>
#include <unordered_set>

#include <libcamera/signal.h>

namespace libcamera {

class Buffer;
class Camera;
class Stream;

class Request
{
public:
	explicit Request(Camera *camera);
	Request(const Request &) = delete;
	Request &operator=(const Request &) = delete;

	int setBuffers(const std::map<Stream *, Buffer *> &streamMap);
	Buffer *findBuffer(Stream *stream) const;

private:
	friend class Camera;

	int prepare();
	void bufferCompleted(Buffer *buffer);

	Camera *camera_;
	std::map<Stream *, Buffer *> bufferMap_;
	std::unordered_set<Buffer *> pending_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_REQUEST_H__ */
