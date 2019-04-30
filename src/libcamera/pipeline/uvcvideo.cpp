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

class UVCCameraConfiguration : public CameraConfiguration
{
public:
	UVCCameraConfiguration();

	Status validate() override;
};

class PipelineHandlerUVC : public PipelineHandler
{
public:
	PipelineHandlerUVC(CameraManager *manager);

	CameraConfiguration *generateConfiguration(Camera *camera,
		const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int allocateBuffers(Camera *camera,
			    const std::set<Stream *> &streams) override;
	int freeBuffers(Camera *camera,
			const std::set<Stream *> &streams) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequest(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	UVCCameraData *cameraData(const Camera *camera)
	{
		return static_cast<UVCCameraData *>(
			PipelineHandler::cameraData(camera));
	}
};

UVCCameraConfiguration::UVCCameraConfiguration()
	: CameraConfiguration()
{
}

CameraConfiguration::Status UVCCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];

	/* \todo: Validate the configuration against the device capabilities. */
	const unsigned int pixelFormat = cfg.pixelFormat;
	const Size size = cfg.size;

	cfg.pixelFormat = V4L2_PIX_FMT_YUYV;
	cfg.size = { 640, 480 };

	if (cfg.pixelFormat != pixelFormat || cfg.size != size) {
		LOG(UVC, Debug)
			<< "Adjusting configuration from " << cfg.toString()
			<< " to " << cfg.size.toString() << "-YUYV";
		status = Adjusted;
	}

	cfg.bufferCount = 4;

	return status;
}

PipelineHandlerUVC::PipelineHandlerUVC(CameraManager *manager)
	: PipelineHandler(manager)
{
}

CameraConfiguration *PipelineHandlerUVC::generateConfiguration(Camera *camera,
	const StreamRoles &roles)
{
	CameraConfiguration *config = new UVCCameraConfiguration();

	if (roles.empty())
		return config;

	StreamConfiguration cfg{};
	cfg.pixelFormat = V4L2_PIX_FMT_YUYV;
	cfg.size = { 640, 480 };
	cfg.bufferCount = 4;

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int PipelineHandlerUVC::configure(Camera *camera, CameraConfiguration *config)
{
	UVCCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	int ret;

	V4L2DeviceFormat format = {};
	format.fourcc = cfg.pixelFormat;
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != cfg.pixelFormat)
		return -EINVAL;

	cfg.setStream(&data->stream_);

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
	MediaDevice *media;
	DeviceMatch dm("uvcvideo");

	media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	std::unique_ptr<UVCCameraData> data = utils::make_unique<UVCCameraData>(this);

	/* Locate and open the default video node. */
	for (MediaEntity *entity : media->entities()) {
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
	std::shared_ptr<Camera> camera = Camera::create(this, media->model(), streams);
	registerCamera(std::move(camera), std::move(data));

	/* Enable hot-unplug notifications. */
	hotplugMediaDevice(media);

	return true;
}

void UVCCameraData::bufferReady(Buffer *buffer)
{
	Request *request = queuedRequests_.front();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
