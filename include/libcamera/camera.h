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
	enum Status {
		Valid,
		Adjusted,
		Invalid,
	};

	using iterator = std::vector<StreamConfiguration>::iterator;
	using const_iterator = std::vector<StreamConfiguration>::const_iterator;

	virtual ~CameraConfiguration();

	void addConfiguration(const StreamConfiguration &cfg);
	virtual Status validate() = 0;

	StreamConfiguration &at(unsigned int index);
	const StreamConfiguration &at(unsigned int index) const;
	StreamConfiguration &operator[](unsigned int index)
	{
		return at(index);
	}
	const StreamConfiguration &operator[](unsigned int index) const
	{
		return at(index);
	}

	iterator begin();
	const_iterator begin() const;
	iterator end();
	const_iterator end() const;

	bool empty() const;
	std::size_t size() const;

protected:
	CameraConfiguration();

	std::vector<StreamConfiguration> config_;
};

class Camera final : public std::enable_shared_from_this<Camera>
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
	std::unique_ptr<CameraConfiguration> generateConfiguration(const StreamRoles &roles);
	int configure(CameraConfiguration *config);

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
