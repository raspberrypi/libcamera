/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vimc.cpp - Pipeline handler for the vimc device
 */

#include <libcamera/camera.h>
#include <libcamera/stream.h>

#include "device_enumerator.h"
#include "media_device.h"
#include "pipeline_handler.h"

namespace libcamera {

class PipeHandlerVimc : public PipelineHandler
{
public:
	PipeHandlerVimc(CameraManager *manager);
	~PipeHandlerVimc();

	bool match(DeviceEnumerator *enumerator);

private:
	std::shared_ptr<MediaDevice> media_;
	Stream stream_;
};

PipeHandlerVimc::PipeHandlerVimc(CameraManager *manager)
	: PipelineHandler(manager), media_(nullptr)
{
}

PipeHandlerVimc::~PipeHandlerVimc()
{
	if (media_)
		media_->release();
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

	std::vector<Stream *> streams{ &stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, "Dummy VIMC Camera", streams);
	registerCamera(std::move(camera));

	return true;
}

REGISTER_PIPELINE_HANDLER(PipeHandlerVimc);

} /* namespace libcamera */
