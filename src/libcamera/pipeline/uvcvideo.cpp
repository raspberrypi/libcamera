/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include "device_enumerator.h"
#include "media_device.h"
#include "pipeline_handler.h"

namespace libcamera {

class PipelineHandlerUVC : public PipelineHandler
{
public:
	PipelineHandlerUVC();
	~PipelineHandlerUVC();

	bool match(CameraManager *manager, DeviceEnumerator *enumerator);

private:
	MediaDevice *dev_;
};

PipelineHandlerUVC::PipelineHandlerUVC()
	: dev_(nullptr)
{
}

PipelineHandlerUVC::~PipelineHandlerUVC()
{
	if (dev_)
		dev_->release();
}

bool PipelineHandlerUVC::match(CameraManager *manager, DeviceEnumerator *enumerator)
{
	DeviceMatch dm("uvcvideo");

	dev_ = enumerator->search(dm);

	if (!dev_)
		return false;

	dev_->acquire();

	std::shared_ptr<Camera> camera = Camera::create(dev_->model());
	manager->addCamera(std::move(camera));

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
