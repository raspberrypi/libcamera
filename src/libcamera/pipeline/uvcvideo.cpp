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

	std::map<Stream *, StreamConfiguration>
	streamConfiguration(Camera *camera,
			    std::set<Stream *> &streams) override;
	int configureStreams(Camera *camera,
			     std::map<Stream *, StreamConfiguration> &config) override;

	int allocateBuffers(Camera *camera, Stream *stream) override;
	int freeBuffers(Camera *camera, Stream *stream) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequest(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator);

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

std::map<Stream *, StreamConfiguration>
PipelineHandlerUVC::streamConfiguration(Camera *camera,
					std::set<Stream *> &streams)
{
	UVCCameraData *data = cameraData(camera);

	std::map<Stream *, StreamConfiguration> configs;
	StreamConfiguration config{};

	LOG(UVC, Debug) << "Retrieving default format";
	config.width = 640;
	config.height = 480;
	config.pixelFormat = V4L2_PIX_FMT_YUYV;
	config.bufferCount = 4;

	configs[&data->stream_] = config;

	return configs;
}

int PipelineHandlerUVC::configureStreams(Camera *camera,
					 std::map<Stream *, StreamConfiguration> &config)
{
	UVCCameraData *data = cameraData(camera);
	StreamConfiguration *cfg = &config[&data->stream_];
	int ret;

	LOG(UVC, Debug) << "Configure the camera for resolution "
			<< cfg->width << "x" << cfg->height;

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

int PipelineHandlerUVC::allocateBuffers(Camera *camera, Stream *stream)
{
	UVCCameraData *data = cameraData(camera);
	const StreamConfiguration &cfg = stream->configuration();

	LOG(UVC, Debug) << "Requesting " << cfg.bufferCount << " buffers";

	return data->video_->exportBuffers(&stream->bufferPool());
}

int PipelineHandlerUVC::freeBuffers(Camera *camera, Stream *stream)
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

	data->video_->queueBuffer(buffer);

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

	/* Create and register the camera. */
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, media_->model(), streams);

	setCameraData(camera.get(), std::move(data));
	registerCamera(std::move(camera));

	/* Enable hot-unplug notifications. */
	hotplugMediaDevice(media_.get());

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
