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
#include <set>
#include <string>

#include <libcamera/request.h>
#include <libcamera/signal.h>
#include <libcamera/stream.h>

namespace libcamera {

class Buffer;
class PipelineHandler;
class Request;

class CameraConfiguration
{
public:
	using iterator = std::vector<Stream *>::iterator;
	using const_iterator = std::vector<Stream *>::const_iterator;

	CameraConfiguration();

	iterator begin();
	iterator end();
	const_iterator begin() const;
	const_iterator end() const;

	bool isValid() const;
	bool isEmpty() const;
	std::size_t size() const;

	Stream *front();
	const Stream *front() const;

	Stream *operator[](unsigned int index) const;
	StreamConfiguration &operator[](Stream *stream);
	const StreamConfiguration &operator[](Stream *stream) const;

private:
	std::vector<Stream *> order_;
	std::map<Stream *, StreamConfiguration> config_;
};

class Camera final
{
public:
	static std::shared_ptr<Camera> create(PipelineHandler *pipe,
					      const std::string &name,
					      const std::set<Stream *> &streams);

	Camera(const Camera &) = delete;
	Camera &operator=(const Camera &) = delete;

	const std::string &name() const;

	Signal<Request *, Buffer *> bufferCompleted;
	Signal<Request *, const std::map<Stream *, Buffer *> &> requestCompleted;
	Signal<Camera *> disconnected;

	int acquire();
	int release();

	const std::set<Stream *> &streams() const;
	CameraConfiguration generateConfiguration(const StreamRoles &roles);
	int configure(const CameraConfiguration &config);

	int allocateBuffers();
	int freeBuffers();

	Request *createRequest();
	int queueRequest(Request *request);

	int start();
	int stop();

private:
	enum State {
		CameraAvailable,
		CameraAcquired,
		CameraConfigured,
		CameraPrepared,
		CameraRunning,
	};

	Camera(PipelineHandler *pipe, const std::string &name);
	~Camera();

	bool stateBetween(State low, State high) const;
	bool stateIs(State state) const;

	friend class PipelineHandler;
	void disconnect();

	void requestComplete(Request *request);

	std::shared_ptr<PipelineHandler> pipe_;
	std::string name_;
	std::set<Stream *> streams_;
	std::set<Stream *> activeStreams_;

	bool disconnected_;
	State state_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CAMERA_H__ */
