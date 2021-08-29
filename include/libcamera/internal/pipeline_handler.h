/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * pipeline_handler.h - Pipeline handler infrastructure
 */

#pragma once

#include <memory>
#include <queue>
#include <set>
#include <string>
#include <sys/types.h>
#include <vector>

#include <libcamera/base/mutex.h>
#include <libcamera/base/object.h>

#include <libcamera/controls.h>
#include <libcamera/stream.h>

#include "libcamera/internal/ipa_proxy.h"

namespace libcamera {

class Camera;
class CameraConfiguration;
class CameraManager;
class DeviceEnumerator;
class DeviceMatch;
class FrameBuffer;
class MediaDevice;
class PipelineHandler;
class Request;

class PipelineHandler : public std::enable_shared_from_this<PipelineHandler>,
			public Object
{
public:
	PipelineHandler(CameraManager *manager);
	virtual ~PipelineHandler();

	virtual bool match(DeviceEnumerator *enumerator) = 0;
	MediaDevice *acquireMediaDevice(DeviceEnumerator *enumerator,
					const DeviceMatch &dm);

	bool acquire();
	void release();

	virtual CameraConfiguration *generateConfiguration(Camera *camera,
		const StreamRoles &roles) = 0;
	virtual int configure(Camera *camera, CameraConfiguration *config) = 0;

	virtual int exportFrameBuffers(Camera *camera, Stream *stream,
				       std::vector<std::unique_ptr<FrameBuffer>> *buffers) = 0;

	virtual int start(Camera *camera, const ControlList *controls) = 0;
	void stop(Camera *camera);
	bool hasPendingRequests(const Camera *camera) const;

	void registerRequest(Request *request);
	void queueRequest(Request *request);

	bool completeBuffer(Request *request, FrameBuffer *buffer);
	void completeRequest(Request *request);

	const char *name() const { return name_; }

protected:
	void registerCamera(std::shared_ptr<Camera> camera);
	void hotplugMediaDevice(MediaDevice *media);

	virtual int queueRequestDevice(Camera *camera, Request *request) = 0;
	virtual void stopDevice(Camera *camera) = 0;

	CameraManager *manager_;

private:
	void unlockMediaDevices();

	void mediaDeviceDisconnected(MediaDevice *media);
	virtual void disconnect();

	void doQueueRequest(Request *request);
	void doQueueRequests();

	std::vector<std::shared_ptr<MediaDevice>> mediaDevices_;
	std::vector<std::weak_ptr<Camera>> cameras_;

	std::queue<Request *> waitingRequests_;

	const char *name_;

	Mutex lock_;
	unsigned int useCount_ LIBCAMERA_TSA_GUARDED_BY(lock_);

	friend class PipelineHandlerFactory;
};

class PipelineHandlerFactory
{
public:
	PipelineHandlerFactory(const char *name);
	virtual ~PipelineHandlerFactory() = default;

	std::shared_ptr<PipelineHandler> create(CameraManager *manager);

	const std::string &name() const { return name_; }

	static void registerType(PipelineHandlerFactory *factory);
	static std::vector<PipelineHandlerFactory *> &factories();

private:
	virtual PipelineHandler *createInstance(CameraManager *manager) = 0;

	std::string name_;
};

#define REGISTER_PIPELINE_HANDLER(handler)				\
class handler##Factory final : public PipelineHandlerFactory		\
{									\
public:									\
	handler##Factory() : PipelineHandlerFactory(#handler) {}	\
									\
private:								\
	PipelineHandler *createInstance(CameraManager *manager)		\
	{								\
		return new handler(manager);				\
	}								\
};									\
static handler##Factory global_##handler##Factory;

} /* namespace libcamera */
