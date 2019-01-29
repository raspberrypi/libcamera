/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vimc.cpp - Pipeline handler for the vimc device
 */

#include <libcamera/camera.h>
#include <libcamera/stream.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "v4l2_device.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(VIMC)

class PipeHandlerVimc : public PipelineHandler
{
public:
	PipeHandlerVimc(CameraManager *manager);
	~PipeHandlerVimc();

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

PipeHandlerVimc::PipeHandlerVimc(CameraManager *manager)
	: PipelineHandler(manager), media_(nullptr), video_(nullptr)
{
}

PipeHandlerVimc::~PipeHandlerVimc()
{
	delete video_;

	if (media_)
		media_->release();
}

std::map<Stream *, StreamConfiguration>
PipeHandlerVimc::streamConfiguration(Camera *camera,
				     std::vector<Stream *> &streams)
{
	std::map<Stream *, StreamConfiguration> configs;

	StreamConfiguration config{};

	LOG(VIMC, Info) << "TODO: Return a good default format";

	configs[&stream_] = config;

	return configs;
}

int PipeHandlerVimc::configureStreams(Camera *camera,
				      std::map<Stream *, StreamConfiguration> &config)
{
	StreamConfiguration *cfg = &config[&stream_];

	LOG(VIMC, Info) << "TODO: Configure the camera for resolution "
			<< cfg->width << "x" << cfg->height;

	return 0;
}

bool PipeHandlerVimc::match(DeviceEnumerator *enumerator)
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

	video_ = new V4L2Device(*media_->getEntityByName("Raw Capture 1"));

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

REGISTER_PIPELINE_HANDLER(PipeHandlerVimc);

} /* namespace libcamera */
