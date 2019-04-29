/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "utils.h"
#include "v4l2_device.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(UVC)

class PipelineHandlerUVC : public PipelineHandler
{
public:
	PipelineHandlerUVC(CameraManager *manager);
	~PipelineHandlerUVC();

	CameraConfiguration
	streamConfiguration(Camera *camera,
			    const std::vector<StreamUsage> &usages) override;
	int configureStreams(Camera *camera,
			     const CameraConfiguration &config) override;

	int allocateBuffers(Camera *camera,
			    const std::set<Stream *> &streams) override;
	int freeBuffers(Camera *camera,
			const std::set<Stream *> &streams) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequest(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	class UVCCameraData : public CameraData
	{
	public:
		UVCCameraData(PipelineHandler *pipe)
			: CameraData(pipe), video_(nullptr)
		{
		}

		~UVCCameraData()
		{
			delete video_;
		}

		void bufferReady(Buffer *buffer);

		V4L2Device *video_;
		Stream stream_;
	};

	UVCCameraData *cameraData(const Camera *camera)
	{
		return static_cast<UVCCameraData *>(
			PipelineHandler::cameraData(camera));
	}

	std::shared_ptr<MediaDevice> media_;
};

PipelineHandlerUVC::PipelineHandlerUVC(CameraManager *manager)
	: PipelineHandler(manager), media_(nullptr)
{
}

PipelineHandlerUVC::~PipelineHandlerUVC()
{
	if (media_)
		media_->release();
}

CameraConfiguration
PipelineHandlerUVC::streamConfiguration(Camera *camera,
					const std::vector<StreamUsage> &usages)
{
	UVCCameraData *data = cameraData(camera);
	CameraConfiguration configs;
	StreamConfiguration config{};

	config.width = 640;
	config.height = 480;
	config.pixelFormat = V4L2_PIX_FMT_YUYV;
	config.bufferCount = 4;

	configs[&data->stream_] = config;

	return configs;
}

int PipelineHandlerUVC::configureStreams(Camera *camera,
					 const CameraConfiguration &config)
{
	UVCCameraData *data = cameraData(camera);
	const StreamConfiguration *cfg = &config[&data->stream_];
	int ret;

	V4L2DeviceFormat format = {};
	format.width = cfg->width;
	format.height = cfg->height;
	format.fourcc = cfg->pixelFormat;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.width != cfg->width ||
	    format.height != cfg->height ||
	    format.fourcc != cfg->pixelFormat)
		return -EINVAL;

	return 0;
}

int PipelineHandlerUVC::allocateBuffers(Camera *camera,
					const std::set<Stream *> &streams)
{
	UVCCameraData *data = cameraData(camera);
	Stream *stream = *streams.begin();
	const StreamConfiguration &cfg = stream->configuration();

	LOG(UVC, Debug) << "Requesting " << cfg.bufferCount << " buffers";

	return data->video_->exportBuffers(&stream->bufferPool());
}

int PipelineHandlerUVC::freeBuffers(Camera *camera,
				    const std::set<Stream *> &streams)
{
	UVCCameraData *data = cameraData(camera);
	return data->video_->releaseBuffers();
}

int PipelineHandlerUVC::start(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	return data->video_->streamOn();
}

void PipelineHandlerUVC::stop(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	data->video_->streamOff();
	PipelineHandler::stop(camera);
}

int PipelineHandlerUVC::queueRequest(Camera *camera, Request *request)
{
	UVCCameraData *data = cameraData(camera);
	Buffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(UVC, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	int ret = data->video_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	PipelineHandler::queueRequest(camera, request);

	return 0;
}

bool PipelineHandlerUVC::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("uvcvideo");

	media_ = enumerator->search(dm);
	if (!media_)
		return false;

	media_->acquire();

	std::unique_ptr<UVCCameraData> data = utils::make_unique<UVCCameraData>(this);

	/* Locate and open the default video node. */
	for (MediaEntity *entity : media_->entities()) {
		if (entity->flags() & MEDIA_ENT_FL_DEFAULT) {
			data->video_ = new V4L2Device(entity);
			break;
		}
	}

	if (!data->video_) {
		LOG(UVC, Error) << "Could not find a default video device";
		return false;
	}

	if (data->video_->open())
		return false;

	data->video_->bufferReady.connect(data.get(), &UVCCameraData::bufferReady);

	/* Create and register the camera. */
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, media_->model(), streams);
	registerCamera(std::move(camera), std::move(data));

	/* Enable hot-unplug notifications. */
	hotplugMediaDevice(media_.get());

	return true;
}

void PipelineHandlerUVC::UVCCameraData::bufferReady(Buffer *buffer)
{
	Request *request = queuedRequests_.front();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
