/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * pipeline_handler.h - Pipeline handler infrastructure
 */
#ifndef __LIBCAMERA_PIPELINE_HANDLER_H__
#define __LIBCAMERA_PIPELINE_HANDLER_H__

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <libcamera/stream.h>

namespace libcamera {

class Buffer;
class BufferPool;
class Camera;
class CameraConfiguration;
class CameraManager;
class DeviceEnumerator;
class DeviceMatch;
class MediaDevice;
class PipelineHandler;
class Request;

class CameraData
{
public:
	explicit CameraData(PipelineHandler *pipe)
		: pipe_(pipe)
	{
	}
	virtual ~CameraData() {}

	Camera *camera_;
	PipelineHandler *pipe_;
	std::list<Request *> queuedRequests_;

private:
	CameraData(const CameraData &) = delete;
	CameraData &operator=(const CameraData &) = delete;
};

class PipelineHandler : public std::enable_shared_from_this<PipelineHandler>
{
public:
	PipelineHandler(CameraManager *manager);
	virtual ~PipelineHandler();

	virtual bool match(DeviceEnumerator *enumerator) = 0;
	MediaDevice *acquireMediaDevice(DeviceEnumerator *enumerator,
					const DeviceMatch &dm);

	bool lock();
	void unlock();

	virtual CameraConfiguration *generateConfiguration(Camera *camera,
		const StreamRoles &roles) = 0;
	virtual int configure(Camera *camera, CameraConfiguration *config) = 0;

	virtual int allocateBuffers(Camera *camera,
				    const std::set<Stream *> &streams) = 0;
	virtual int freeBuffers(Camera *camera,
				const std::set<Stream *> &streams) = 0;

	virtual int start(Camera *camera) = 0;
	virtual void stop(Camera *camera);

	virtual int queueRequest(Camera *camera, Request *request);

	bool completeBuffer(Camera *camera, Request *request, Buffer *buffer);
	void completeRequest(Camera *camera, Request *request);

protected:
	void registerCamera(std::shared_ptr<Camera> camera,
			    std::unique_ptr<CameraData> data);
	void hotplugMediaDevice(MediaDevice *media);

	CameraData *cameraData(const Camera *camera);

	CameraManager *manager_;

private:
	void mediaDeviceDisconnected(MediaDevice *media);
	virtual void disconnect();

	std::vector<std::shared_ptr<MediaDevice>> mediaDevices_;
	std::vector<std::weak_ptr<Camera>> cameras_;
	std::map<const Camera *, std::unique_ptr<CameraData>> cameraData_;
};

class PipelineHandlerFactory
{
public:
	PipelineHandlerFactory(const char *name);
	virtual ~PipelineHandlerFactory() { };

	virtual std::shared_ptr<PipelineHandler> create(CameraManager *manager) = 0;

	const std::string &name() const { return name_; }

	static void registerType(PipelineHandlerFactory *factory);
	static std::vector<PipelineHandlerFactory *> &factories();

private:
	std::string name_;
};

#define REGISTER_PIPELINE_HANDLER(handler)				\
class handler##Factory final : public PipelineHandlerFactory		\
{									\
public:									\
	handler##Factory() : PipelineHandlerFactory(#handler) {}	\
	std::shared_ptr<PipelineHandler> create(CameraManager *manager)	\
	{								\
		return std::make_shared<handler>(manager);		\
	}								\
};									\
static handler##Factory global_##handler##Factory;

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_HANDLER_H__ */
