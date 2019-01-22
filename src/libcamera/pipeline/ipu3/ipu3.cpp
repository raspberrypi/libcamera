/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipu3.cpp - Pipeline handler for Intel IPU3
 */

#include <memory>
#include <vector>

#include <libcamera/camera.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPU3)

class PipelineHandlerIPU3 : public PipelineHandler
{
public:
	PipelineHandlerIPU3(CameraManager *manager);
	~PipelineHandlerIPU3();

	bool match(DeviceEnumerator *enumerator);

private:
	std::shared_ptr<MediaDevice> cio2_;
	std::shared_ptr<MediaDevice> imgu_;

	void registerCameras();
};

PipelineHandlerIPU3::PipelineHandlerIPU3(CameraManager *manager)
	: PipelineHandler(manager), cio2_(nullptr), imgu_(nullptr)
{
}

PipelineHandlerIPU3::~PipelineHandlerIPU3()
{
	if (cio2_)
		cio2_->release();

	if (imgu_)
		imgu_->release();
}

bool PipelineHandlerIPU3::match(DeviceEnumerator *enumerator)
{
	DeviceMatch cio2_dm("ipu3-cio2");
	cio2_dm.add("ipu3-csi2 0");
	cio2_dm.add("ipu3-cio2 0");
	cio2_dm.add("ipu3-csi2 1");
	cio2_dm.add("ipu3-cio2 1");
	cio2_dm.add("ipu3-csi2 2");
	cio2_dm.add("ipu3-cio2 2");
	cio2_dm.add("ipu3-csi2 3");
	cio2_dm.add("ipu3-cio2 3");

	DeviceMatch imgu_dm("ipu3-imgu");
	imgu_dm.add("ipu3-imgu 0");
	imgu_dm.add("ipu3-imgu 0 input");
	imgu_dm.add("ipu3-imgu 0 parameters");
	imgu_dm.add("ipu3-imgu 0 output");
	imgu_dm.add("ipu3-imgu 0 viewfinder");
	imgu_dm.add("ipu3-imgu 0 3a stat");
	imgu_dm.add("ipu3-imgu 1");
	imgu_dm.add("ipu3-imgu 1 input");
	imgu_dm.add("ipu3-imgu 1 parameters");
	imgu_dm.add("ipu3-imgu 1 output");
	imgu_dm.add("ipu3-imgu 1 viewfinder");
	imgu_dm.add("ipu3-imgu 1 3a stat");

	cio2_ = std::move(enumerator->search(cio2_dm));
	if (!cio2_)
		return false;

	imgu_ = std::move(enumerator->search(imgu_dm));
	if (!imgu_)
		return false;

	/*
	 * It is safe to acquire both media devices at this point as
	 * DeviceEnumerator::search() skips the busy ones for us.
	 */
	cio2_->acquire();
	imgu_->acquire();

	/*
	 * Disable all links that are enabled by default on CIO2, as camera
	 * creation enables all valid links it finds.
	 *
	 * Close the CIO2 media device after, as links are enabled and should
	 * not need to be changed after.
	 */
	if (cio2_->open())
		goto error_release_mdev;

	if (cio2_->disableLinks())
		goto error_close_cio2;

	registerCameras();

	cio2_->close();

	return true;

error_close_cio2:
	cio2_->close();

error_release_mdev:
	cio2_->release();
	imgu_->release();

	return false;
}

/*
 * Cameras are created associating an image sensor (represented by a
 * media entity with function MEDIA_ENT_F_CAM_SENSOR) to one of the four
 * CIO2 CSI-2 receivers.
 */
void PipelineHandlerIPU3::registerCameras()
{
	/*
	 * For each CSI-2 receiver on the IPU3, create a Camera if an
	 * image sensor is connected to it.
	 */
	unsigned int numCameras = 0;
	for (unsigned int id = 0; id < 4; ++id) {
		std::string csi2Name = "ipu3-csi2 " + std::to_string(id);
		MediaEntity *csi2 = cio2_->getEntityByName(csi2Name);

		/*
		 * This shall not happen, as the device enumerator matched
		 * all entities described in the cio2_dm DeviceMatch.
		 *
		 * As this check is basically free, better stay safe than sorry.
		 */
		if (!csi2)
			continue;

		const std::vector<MediaPad *> &pads = csi2->pads();
		if (pads.empty())
			continue;

		/* IPU3 CSI-2 receivers have a single sink pad at index 0. */
		MediaPad *sink = pads[0];
		const std::vector<MediaLink *> &links = sink->links();
		if (links.empty())
			continue;

		/*
		 * Verify that the receiver is connected to a sensor, enable
		 * the media link between the two, and create a Camera with
		 * a unique name.
		 */
		MediaLink *link = links[0];
		MediaEntity *sensor = link->source()->entity();
		if (sensor->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		if (link->setEnabled(true))
			continue;

		std::string cameraName = sensor->name() + " " + std::to_string(id);
		std::shared_ptr<Camera> camera = Camera::create(this, cameraName);
		registerCamera(std::move(camera));

		LOG(IPU3, Info)
			<< "Registered Camera[" << numCameras << "] \""
			<< cameraName << "\""
			<< " connected to CSI-2 receiver " << id;

		numCameras++;
	}
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU3);

} /* namespace libcamera */
