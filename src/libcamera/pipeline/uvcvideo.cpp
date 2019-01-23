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
	PipelineHandlerUVC(CameraManager *manager);
	~PipelineHandlerUVC();

	bool match(DeviceEnumerator *enumerator);

private:
	std::shared_ptr<MediaDevice> dev_;
};

PipelineHandlerUVC::PipelineHandlerUVC(CameraManager *manager)
	: PipelineHandler(manager), dev_(nullptr)
{
}

PipelineHandlerUVC::~PipelineHandlerUVC()
{
	if (dev_)
		dev_->release();
}

bool PipelineHandlerUVC::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("uvcvideo");

	dev_ = std::move(enumerator->search(dm));

	if (!dev_)
		return false;

	dev_->acquire();

	std::shared_ptr<Camera> camera = Camera::create(this, dev_->model());
	manager_->addCamera(std::move(camera));

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
