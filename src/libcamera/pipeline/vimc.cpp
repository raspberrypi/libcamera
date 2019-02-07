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
			    std::vector<Stream *> &streams) override;
	int configureStreams(Camera *camera,
			     std::map<Stream *, StreamConfiguration> &config) override;

	int allocateBuffers(Camera *camera, Stream *stream) override;
	int freeBuffers(Camera *camera, Stream *stream) override;

	int start(const Camera *camera) override;
	void stop(const Camera *camera) override;

	int queueRequest(const Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator);

private:
	std::shared_ptr<MediaDevice> media_;
	V4L2Device *video_;
	Stream stream_;
};

PipelineHandlerVimc::PipelineHandlerVimc(CameraManager *manager)
	: PipelineHandler(manager), media_(nullptr), video_(nullptr)
{
}

PipelineHandlerVimc::~PipelineHandlerVimc()
{
	delete video_;

	if (media_)
		media_->release();
}

std::map<Stream *, StreamConfiguration>
PipelineHandlerVimc::streamConfiguration(Camera *camera,
				     std::vector<Stream *> &streams)
{
	std::map<Stream *, StreamConfiguration> configs;

	StreamConfiguration config{};

	LOG(VIMC, Debug) << "Retrieving default format";
	config.width = 640;
	config.height = 480;
	config.pixelFormat = V4L2_PIX_FMT_RGB24;
	config.bufferCount = 4;

	configs[&stream_] = config;

	return configs;
}

int PipelineHandlerVimc::configureStreams(Camera *camera,
				      std::map<Stream *, StreamConfiguration> &config)
{
	StreamConfiguration *cfg = &config[&stream_];

	LOG(VIMC, Debug) << "Configure the camera for resolution "
			 << cfg->width << "x" << cfg->height;

	V4L2DeviceFormat format = {};
	format.width = cfg->width;
	format.height = cfg->height;
	format.fourcc = cfg->pixelFormat;

	return video_->setFormat(&format);
}

int PipelineHandlerVimc::allocateBuffers(Camera *camera, Stream *stream)
{
	const StreamConfiguration &cfg = stream->configuration();

	LOG(VIMC, Debug) << "Requesting " << cfg.bufferCount << " buffers";

	return video_->exportBuffers(&stream->bufferPool());
}

int PipelineHandlerVimc::freeBuffers(Camera *camera, Stream *stream)
{
	return video_->releaseBuffers();
}

int PipelineHandlerVimc::start(const Camera *camera)
{
	return video_->streamOn();
}

void PipelineHandlerVimc::stop(const Camera *camera)
{
	video_->streamOff();
}

int PipelineHandlerVimc::queueRequest(const Camera *camera, Request *request)
{
	Buffer *buffer = request->findBuffer(&stream_);
	if (!buffer) {
		LOG(VIMC, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	video_->queueBuffer(buffer);

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

	media_ = std::move(enumerator->search(dm));
	if (!media_)
		return false;

	media_->acquire();

	video_ = new V4L2Device(media_->getEntityByName("Raw Capture 1"));

	if (video_->open()) {
		media_->release();
		return false;
	}

	std::vector<Stream *> streams{ &stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, "VIMC Sensor B",
							streams);
	registerCamera(std::move(camera));

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerVimc);

} /* namespace libcamera */
