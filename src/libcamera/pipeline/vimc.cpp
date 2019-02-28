/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vimc.cpp - Pipeline handler for the vimc device
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

LOG_DEFINE_CATEGORY(VIMC)

class PipelineHandlerVimc : public PipelineHandler
{
public:
	PipelineHandlerVimc(CameraManager *manager);
	~PipelineHandlerVimc();

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
	class VimcCameraData : public CameraData
	{
	public:
		VimcCameraData(PipelineHandler *pipe)
			: CameraData(pipe)
		{
		}

		~VimcCameraData()
		{
			delete video_;
		}

		V4L2Device *video_;
		Stream stream_;
	};

	VimcCameraData *cameraData(const Camera *camera)
	{
		return static_cast<VimcCameraData *>(
			PipelineHandler::cameraData(camera));
	}

	std::shared_ptr<MediaDevice> media_;
};

PipelineHandlerVimc::PipelineHandlerVimc(CameraManager *manager)
	: PipelineHandler(manager), media_(nullptr)
{
}

PipelineHandlerVimc::~PipelineHandlerVimc()
{
	if (media_)
		media_->release();
}

std::map<Stream *, StreamConfiguration>
PipelineHandlerVimc::streamConfiguration(Camera *camera,
					 std::set<Stream *> &streams)
{
	VimcCameraData *data = cameraData(camera);
	std::map<Stream *, StreamConfiguration> configs;

	StreamConfiguration config{};

	LOG(VIMC, Debug) << "Retrieving default format";
	config.width = 640;
	config.height = 480;
	config.pixelFormat = V4L2_PIX_FMT_RGB24;
	config.bufferCount = 4;

	configs[&data->stream_] = config;

	return configs;
}

int PipelineHandlerVimc::configureStreams(Camera *camera,
				      std::map<Stream *, StreamConfiguration> &config)
{
	VimcCameraData *data = cameraData(camera);
	StreamConfiguration *cfg = &config[&data->stream_];
	int ret;

	LOG(VIMC, Debug) << "Configure the camera for resolution "
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

int PipelineHandlerVimc::allocateBuffers(Camera *camera, Stream *stream)
{
	VimcCameraData *data = cameraData(camera);
	const StreamConfiguration &cfg = stream->configuration();

	LOG(VIMC, Debug) << "Requesting " << cfg.bufferCount << " buffers";

	return data->video_->exportBuffers(&stream->bufferPool());
}

int PipelineHandlerVimc::freeBuffers(Camera *camera, Stream *stream)
{
	VimcCameraData *data = cameraData(camera);
	return data->video_->releaseBuffers();
}

int PipelineHandlerVimc::start(Camera *camera)
{
	VimcCameraData *data = cameraData(camera);
	return data->video_->streamOn();
}

void PipelineHandlerVimc::stop(Camera *camera)
{
	VimcCameraData *data = cameraData(camera);
	data->video_->streamOff();
}

int PipelineHandlerVimc::queueRequest(Camera *camera, Request *request)
{
	VimcCameraData *data = cameraData(camera);
	Buffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(VIMC, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	data->video_->queueBuffer(buffer);

	return 0;
}

bool PipelineHandlerVimc::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("vimc");

	dm.add("Raw Capture 0");
	dm.add("Raw Capture 1");
	dm.add("RGB/YUV Capture");
	dm.add("Sensor A");
	dm.add("Sensor B");
	dm.add("Debayer A");
	dm.add("Debayer B");
	dm.add("RGB/YUV Input");
	dm.add("Scaler");

	media_ = enumerator->search(dm);
	if (!media_)
		return false;

	media_->acquire();

	std::unique_ptr<VimcCameraData> data = utils::make_unique<VimcCameraData>(this);

	/* Locate and open the capture video node. */
	data->video_ = new V4L2Device(media_->getEntityByName("Raw Capture 1"));
	if (data->video_->open())
		return false;

	/* Create and register the camera. */
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, "VIMC Sensor B",
							streams);

	setCameraData(camera.get(), std::move(data));
	registerCamera(std::move(camera));

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerVimc);

} /* namespace libcamera */
