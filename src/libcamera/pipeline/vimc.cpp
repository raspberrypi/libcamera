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
	PipeHandlerVimc();
	~PipeHandlerVimc();

	bool match(CameraManager *manager, DeviceEnumerator *enumerator);

private:
	MediaDevice *dev_;
};

PipeHandlerVimc::PipeHandlerVimc()
	: dev_(nullptr)
{
}

PipeHandlerVimc::~PipeHandlerVimc()
{
	if (dev_)
		dev_->release();
}

bool PipeHandlerVimc::match(CameraManager *manager, DeviceEnumerator *enumerator)
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

	/*
	 * NOTE: A more complete Camera implementation could
	 * be passed the MediaDevice(s) it controls here or
	 * a reference to the PipelineHandler. Which method
	 * will be chosen depends on how the Camera
	 * object is modeled.
	 */
	Camera *camera = new Camera("Dummy VIMC Camera");
	manager->addCamera(camera);

	return true;
}

REGISTER_PIPELINE_HANDLER(PipeHandlerVimc);

} /* namespace libcamera */
