/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vimc.cpp - Pipeline handler for the vimc device
 */

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

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
	MediaDevice *dev_;
};

PipeHandlerVimc::PipeHandlerVimc(CameraManager *manager)
	: PipelineHandler(manager), dev_(nullptr)
{
}

PipeHandlerVimc::~PipeHandlerVimc()
{
	if (dev_)
		dev_->release();
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

	dev_ = enumerator->search(dm);
	if (!dev_)
		return false;

	dev_->acquire();

	std::shared_ptr<Camera> camera = Camera::create(this, "Dummy VIMC Camera");
	manager_->addCamera(std::move(camera));

	return true;
}

REGISTER_PIPELINE_HANDLER(PipeHandlerVimc);

} /* namespace libcamera */
