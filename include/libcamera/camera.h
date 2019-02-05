/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.h - Camera object interface
 */
#ifndef __LIBCAMERA_CAMERA_H__
#define __LIBCAMERA_CAMERA_H__

#include <map>
#include <memory>
#include <string>

#include <libcamera/request.h>
#include <libcamera/signal.h>

namespace libcamera {

class Buffer;
class PipelineHandler;
class Request;
class Stream;
class StreamConfiguration;

class Camera final
{
public:
	static std::shared_ptr<Camera> create(PipelineHandler *pipe,
					      const std::string &name,
					      const std::vector<Stream *> &streams);

	Camera(const Camera &) = delete;
	Camera &operator=(const Camera &) = delete;

	const std::string &name() const;

	Signal<Request *, const std::map<Stream *, Buffer *> &> requestCompleted;
	Signal<Camera *> disconnected;

	int acquire();
	void release();

	const std::vector<Stream *> &streams() const;
	std::map<Stream *, StreamConfiguration>
	streamConfiguration(std::vector<Stream *> &streams);
	int configureStreams(std::map<Stream *, StreamConfiguration> &config);

	int allocateBuffers();
	void freeBuffers();

	Request *createRequest();
	int queueRequest(Request *request);

	int start();
	int stop();

private:
	Camera(PipelineHandler *pipe, const std::string &name);
	~Camera();

	friend class PipelineHandler;
	void disconnect();
	int exclusiveAccess();

	std::shared_ptr<PipelineHandler> pipe_;
	std::string name_;
	std::vector<Stream *> streams_;
	std::vector<Stream *> activeStreams_;

	bool acquired_;
	bool disconnected_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CAMERA_H__ */
