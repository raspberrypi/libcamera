/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipu3.cpp - Pipeline handler for Intel IPU3
 */

#include <memory>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "utils.h"
#include "v4l2_device.h"
#include "v4l2_subdevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPU3)

class PipelineHandlerIPU3 : public PipelineHandler
{
public:
	PipelineHandlerIPU3(CameraManager *manager);
	~PipelineHandlerIPU3();

	std::map<Stream *, StreamConfiguration>
	streamConfiguration(Camera *camera,
			    std::set<Stream *> &streams) override;
	int configureStreams(Camera *camera,
			     std::map<Stream *, StreamConfiguration> &config) override;

	int allocateBuffers(Camera *camera, Stream *stream) override;
	int freeBuffers(Camera *camera, Stream *stream) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequest(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator);

private:
	class IPU3CameraData : public CameraData
	{
	public:
		IPU3CameraData(PipelineHandler *pipe)
			: CameraData(pipe), cio2_(nullptr), csi2_(nullptr),
			  sensor_(nullptr)
		{
		}

		~IPU3CameraData()
		{
			delete cio2_;
			delete csi2_;
			delete sensor_;
		}

		void bufferReady(Buffer *buffer);

		V4L2Device *cio2_;
		V4L2Subdevice *csi2_;
		V4L2Subdevice *sensor_;

		Stream stream_;
	};

	IPU3CameraData *cameraData(const Camera *camera)
	{
		return static_cast<IPU3CameraData *>(
			PipelineHandler::cameraData(camera));
	}

	void registerCameras();

	std::shared_ptr<MediaDevice> cio2_;
	std::shared_ptr<MediaDevice> imgu_;
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

std::map<Stream *, StreamConfiguration>
PipelineHandlerIPU3::streamConfiguration(Camera *camera,
					 std::set<Stream *> &streams)
{
	IPU3CameraData *data = cameraData(camera);
	std::map<Stream *, StreamConfiguration> configs;
	V4L2SubdeviceFormat format = {};

	/*
	 * FIXME: As of now, return the image format reported by the sensor.
	 * In future good defaults should be provided for each stream.
	 */
	if (data->sensor_->getFormat(0, &format)) {
		LOG(IPU3, Error) << "Failed to create stream configurations";
		return configs;
	}

	StreamConfiguration config = {};
	config.width = format.width;
	config.height = format.height;
	config.pixelFormat = V4L2_PIX_FMT_IPU3_SGRBG10;
	config.bufferCount = 4;

	configs[&data->stream_] = config;

	return configs;
}

int PipelineHandlerIPU3::configureStreams(Camera *camera,
					  std::map<Stream *, StreamConfiguration> &config)
{
	IPU3CameraData *data = cameraData(camera);
	StreamConfiguration *cfg = &config[&data->stream_];
	V4L2Subdevice *sensor = data->sensor_;
	V4L2Subdevice *csi2 = data->csi2_;
	V4L2Device *cio2 = data->cio2_;
	V4L2SubdeviceFormat subdevFormat = {};
	V4L2DeviceFormat devFormat = {};
	int ret;

	/*
	 * FIXME: as of now, the format gets applied to the sensor and is
	 * propagated along the pipeline. It should instead be applied on the
	 * capture device and the sensor format calculated accordingly.
	 */

	ret = sensor->getFormat(0, &subdevFormat);
	if (ret)
		return ret;

	subdevFormat.width = cfg->width;
	subdevFormat.height = cfg->height;
	ret = sensor->setFormat(0, &subdevFormat);
	if (ret)
		return ret;

	/* Return error if the requested format cannot be applied to sensor. */
	if (subdevFormat.width != cfg->width ||
	    subdevFormat.height != cfg->height) {
		LOG(IPU3, Error)
			<< "Failed to apply image format "
			<< subdevFormat.width << "x" << subdevFormat.height
			<< " - got: " << cfg->width << "x" << cfg->height;
		return -EINVAL;
	}

	ret = csi2->setFormat(0, &subdevFormat);
	if (ret)
		return ret;

	ret = cio2->getFormat(&devFormat);
	if (ret)
		return ret;

	devFormat.width = subdevFormat.width;
	devFormat.height = subdevFormat.height;
	devFormat.fourcc = cfg->pixelFormat;

	ret = cio2->setFormat(&devFormat);
	if (ret)
		return ret;

	LOG(IPU3, Info) << cio2->driverName() << ": "
			<< devFormat.width << "x" << devFormat.height
			<< "- 0x" << std::hex << devFormat.fourcc << " planes: "
			<< devFormat.planes;

	return 0;
}

int PipelineHandlerIPU3::allocateBuffers(Camera *camera, Stream *stream)
{
	IPU3CameraData *data = cameraData(camera);
	const StreamConfiguration &cfg = stream->configuration();

	if (!cfg.bufferCount)
		return -EINVAL;

	int ret = data->cio2_->exportBuffers(&stream->bufferPool());
	if (ret) {
		LOG(IPU3, Error) << "Failed to request memory";
		return ret;
	}

	return 0;
}

int PipelineHandlerIPU3::freeBuffers(Camera *camera, Stream *stream)
{
	IPU3CameraData *data = cameraData(camera);

	int ret = data->cio2_->releaseBuffers();
	if (ret) {
		LOG(IPU3, Error) << "Failed to release memory";
		return ret;
	}

	return 0;
}

int PipelineHandlerIPU3::start(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	int ret;

	ret = data->cio2_->streamOn();
	if (ret) {
		LOG(IPU3, Info) << "Failed to start camera " << camera->name();
		return ret;
	}

	return 0;
}

void PipelineHandlerIPU3::stop(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);

	if (data->cio2_->streamOff())
		LOG(IPU3, Info) << "Failed to stop camera " << camera->name();

	PipelineHandler::stop(camera);
}

int PipelineHandlerIPU3::queueRequest(Camera *camera, Request *request)
{
	IPU3CameraData *data = cameraData(camera);
	Stream *stream = &data->stream_;

	Buffer *buffer = request->findBuffer(stream);
	if (!buffer) {
		LOG(IPU3, Error)
			<< "Attempt to queue request with invalid stream";
		return -ENOENT;
	}

	int ret = data->cio2_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	PipelineHandler::queueRequest(camera, request);

	return 0;
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

	/*
	 * It is safe to acquire both media devices at this point as
	 * DeviceEnumerator::search() skips the busy ones for us.
	 */
	cio2_ = enumerator->search(cio2_dm);
	if (!cio2_)
		return false;

	cio2_->acquire();

	imgu_ = enumerator->search(imgu_dm);
	if (!imgu_)
		return false;

	imgu_->acquire();

	/*
	 * Disable all links that are enabled by default on CIO2, as camera
	 * creation enables all valid links it finds.
	 *
	 * Close the CIO2 media device after, as links are enabled and should
	 * not need to be changed after.
	 */
	if (cio2_->open())
		return false;

	if (cio2_->disableLinks()) {
		cio2_->close();
		return false;
	}

	registerCameras();

	cio2_->close();

	return true;
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
		int ret;

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

		std::unique_ptr<IPU3CameraData> data = utils::make_unique<IPU3CameraData>(this);

		std::string cameraName = sensor->name() + " " + std::to_string(id);
		std::set<Stream *> streams{ &data->stream_ };
		std::shared_ptr<Camera> camera = Camera::create(this, cameraName, streams);

		/*
		 * Create and open video devices and subdevices associated with
		 * the camera.
		 *
		 * If any of these operations fails, the Camera instance won't
		 * be registered. The 'camera' shared pointer and the 'data'
		 * unique pointers go out of scope and delete the objects they
		 * manage.
		 */
		std::string cio2Name = "ipu3-cio2 " + std::to_string(id);
		MediaEntity *cio2 = cio2_->getEntityByName(cio2Name);
		if (!cio2) {
			LOG(IPU3, Error)
				<< "Failed to get entity '" << cio2Name << "'";
			continue;
		}

		data->cio2_ = new V4L2Device(cio2);
		ret = data->cio2_->open();
		if (ret)
			continue;

		data->cio2_->bufferReady.connect(data.get(), &IPU3CameraData::bufferReady);

		data->sensor_ = new V4L2Subdevice(sensor);
		ret = data->sensor_->open();
		if (ret)
			continue;

		data->csi2_ = new V4L2Subdevice(csi2);
		ret = data->csi2_->open();
		if (ret)
			continue;

		registerCamera(std::move(camera), std::move(data));

		LOG(IPU3, Info)
			<< "Registered Camera[" << numCameras << "] \""
			<< cameraName << "\""
			<< " connected to CSI-2 receiver " << id;

		numCameras++;
	}
}

void PipelineHandlerIPU3::IPU3CameraData::bufferReady(Buffer *buffer)
{
	Request *request = queuedRequests_.front();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU3);

} /* namespace libcamera */
