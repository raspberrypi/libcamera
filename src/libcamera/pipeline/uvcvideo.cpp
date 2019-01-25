/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <libcamera/camera.h>

#include "device_enumerator.h"
#include "media_device.h"
#include "pipeline_handler.h"

namespace libcamera {

class PipelineHandlerUVC : public PipelineHandler
{
public:
	PipelineHandlerUVC(CameraManager *manager);
	~PipelineHandlerUVC();

	bool match(DeviceEnumerator *enumerator);

private:
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

bool PipelineHandlerUVC::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("uvcvideo");

	media_ = std::move(enumerator->search(dm));

	if (!media_)
		return false;

	media_->acquire();

	std::shared_ptr<Camera> camera = Camera::create(this, media_->model());
	registerCamera(std::move(camera));
	hotplugMediaDevice(media_.get());

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
