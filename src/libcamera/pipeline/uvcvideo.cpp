/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <libcamera/camera.h>
#include <libcamera/stream.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
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
			    std::vector<Stream *> &streams) override;
	int configureStreams(Camera *camera,
			     std::map<Stream *, StreamConfiguration> &config) override;

	bool match(DeviceEnumerator *enumerator);

private:
	std::shared_ptr<MediaDevice> media_;
	V4L2Device *video_;
	Stream stream_;
};

PipelineHandlerUVC::PipelineHandlerUVC(CameraManager *manager)
	: PipelineHandler(manager), media_(nullptr), video_(nullptr)
{
}

PipelineHandlerUVC::~PipelineHandlerUVC()
{
	if (video_)
		delete video_;

	if (media_)
		media_->release();
}

std::map<Stream *, StreamConfiguration>
PipelineHandlerUVC::streamConfiguration(Camera *camera,
					std::vector<Stream *> &streams)
{
	std::map<Stream *, StreamConfiguration> configs;

	StreamConfiguration config{};

	LOG(UVC, Debug) << "Retrieving default format";
	config.width = 640;
	config.height = 480;
	config.pixelFormat = V4L2_PIX_FMT_YUYV;
	config.bufferCount = 4;

	configs[&stream_] = config;

	return configs;
}

int PipelineHandlerUVC::configureStreams(Camera *camera,
					 std::map<Stream *, StreamConfiguration> &config)
{
	StreamConfiguration *cfg = &config[&stream_];

	LOG(UVC, Info) << "TODO: Configure the camera for resolution "
		       << cfg->width << "x" << cfg->height;

	return 0;
}

bool PipelineHandlerUVC::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("uvcvideo");

	media_ = std::move(enumerator->search(dm));

	if (!media_)
		return false;

	media_->acquire();

	for (MediaEntity *entity : media_->entities()) {
		if (entity->flags() & MEDIA_ENT_FL_DEFAULT) {
			video_ = new V4L2Device(entity);
			break;
		}
	}

	if (!video_ || video_->open()) {
		if (!video_)
			LOG(UVC, Error) << "Could not find a default video device";

		media_->release();
		return false;
	}

	std::vector<Stream *> streams{ &stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, media_->model(), streams);
	registerCamera(std::move(camera));
	hotplugMediaDevice(media_.get());

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
