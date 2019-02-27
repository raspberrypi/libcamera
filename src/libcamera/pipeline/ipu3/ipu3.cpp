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

class CIO2Device
{
public:
	CIO2Device()
		: output_(nullptr), csi2_(nullptr), sensor_(nullptr)
	{
	}

	~CIO2Device()
	{
		delete output_;
		delete csi2_;
		delete sensor_;
	}

	int init(const MediaDevice *media, unsigned int index);

	V4L2Device *output_;
	V4L2Subdevice *csi2_;
	V4L2Subdevice *sensor_;
};

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
			: CameraData(pipe)
		{
		}

		void bufferReady(Buffer *buffer);

		CIO2Device cio2_;

		Stream stream_;
	};

	IPU3CameraData *cameraData(const Camera *camera)
	{
		return static_cast<IPU3CameraData *>(
			PipelineHandler::cameraData(camera));
	}

	void registerCameras();

	std::shared_ptr<MediaDevice> cio2MediaDev_;
	std::shared_ptr<MediaDevice> imguMediaDev_;
};

PipelineHandlerIPU3::PipelineHandlerIPU3(CameraManager *manager)
	: PipelineHandler(manager), cio2MediaDev_(nullptr), imguMediaDev_(nullptr)
{
}

PipelineHandlerIPU3::~PipelineHandlerIPU3()
{
	if (cio2MediaDev_)
		cio2MediaDev_->release();

	if (imguMediaDev_)
		imguMediaDev_->release();
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
	if (data->cio2_.sensor_->getFormat(0, &format)) {
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
	V4L2Subdevice *sensor = data->cio2_.sensor_;
	V4L2Subdevice *csi2 = data->cio2_.csi2_;
	V4L2Device *cio2 = data->cio2_.output_;
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
	const StreamConfiguration &cfg = stream->configuration();
	IPU3CameraData *data = cameraData(camera);
	V4L2Device *cio2 = data->cio2_.output_;

	if (!cfg.bufferCount)
		return -EINVAL;

	int ret = cio2->exportBuffers(&stream->bufferPool());
	if (ret) {
		LOG(IPU3, Error) << "Failed to request memory";
		return ret;
	}

	return 0;
}

int PipelineHandlerIPU3::freeBuffers(Camera *camera, Stream *stream)
{
	IPU3CameraData *data = cameraData(camera);
	V4L2Device *cio2 = data->cio2_.output_;

	int ret = cio2->releaseBuffers();
	if (ret) {
		LOG(IPU3, Error) << "Failed to release memory";
		return ret;
	}

	return 0;
}

int PipelineHandlerIPU3::start(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	V4L2Device *cio2 = data->cio2_.output_;
	int ret;

	ret = cio2->streamOn();
	if (ret) {
		LOG(IPU3, Info) << "Failed to start camera " << camera->name();
		return ret;
	}

	return 0;
}

void PipelineHandlerIPU3::stop(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	V4L2Device *cio2 = data->cio2_.output_;

	if (cio2->streamOff())
		LOG(IPU3, Info) << "Failed to stop camera " << camera->name();

	PipelineHandler::stop(camera);
}

int PipelineHandlerIPU3::queueRequest(Camera *camera, Request *request)
{
	IPU3CameraData *data = cameraData(camera);
	V4L2Device *cio2 = data->cio2_.output_;
	Stream *stream = &data->stream_;

	Buffer *buffer = request->findBuffer(stream);
	if (!buffer) {
		LOG(IPU3, Error)
			<< "Attempt to queue request with invalid stream";
		return -ENOENT;
	}

	int ret = cio2->queueBuffer(buffer);
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
	cio2MediaDev_ = enumerator->search(cio2_dm);
	if (!cio2MediaDev_)
		return false;

	cio2MediaDev_->acquire();

	imguMediaDev_ = enumerator->search(imgu_dm);
	if (!imguMediaDev_)
		return false;

	imguMediaDev_->acquire();

	/*
	 * Disable all links that are enabled by default on CIO2, as camera
	 * creation enables all valid links it finds.
	 *
	 * Close the CIO2 media device after, as links are enabled and should
	 * not need to be changed after.
	 */
	if (cio2MediaDev_->open())
		return false;

	if (cio2MediaDev_->disableLinks()) {
		cio2MediaDev_->close();
		return false;
	}

	registerCameras();

	cio2MediaDev_->close();

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
	 * image sensor is connected to it and the sensor can produce images
	 * in a compatible format.
	 */
	unsigned int numCameras = 0;
	for (unsigned int id = 0; id < 4; ++id) {
		std::unique_ptr<IPU3CameraData> data =
			utils::make_unique<IPU3CameraData>(this);
		std::set<Stream *> streams{ &data->stream_ };
		CIO2Device *cio2 = &data->cio2_;

		int ret = cio2->init(cio2MediaDev_.get(), id);
		if (ret)
			continue;

		std::string cameraName = cio2->sensor_->entityName() + " "
				       + std::to_string(id);
		std::shared_ptr<Camera> camera = Camera::create(this,
								cameraName,
								streams);

		cio2->output_->bufferReady.connect(data.get(),
						   &IPU3CameraData::bufferReady);

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

/*------------------------------------------------------------------------------
 * CIO2 Device
 */

/**
 * \brief Initialize components of the CIO2 device with \a index
 * \param[in] media The CIO2 media device
 * \param[in] index The CIO2 device index
 *
 * Create and open the video device and subdevices in the CIO2 instance at \a
 * index, if a supported image sensor is connected to the CSI-2 receiver of
 * this CIO2 instance.  Enable the media links connecting the CIO2 components
 * to prepare for capture operations.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV No supported image sensor is connected to this CIO2 instance
 */
int CIO2Device::init(const MediaDevice *media, unsigned int index)
{
	int ret;

	/*
	 * Verify that a sensor subdevice is connected to this CIO2 instance
	 * and enable the media link between the two.
	 */
	std::string csi2Name = "ipu3-csi2 " + std::to_string(index);
	MediaEntity *csi2Entity = media->getEntityByName(csi2Name);
	const std::vector<MediaPad *> &pads = csi2Entity->pads();
	if (pads.empty())
		return -ENODEV;

	/* IPU3 CSI-2 receivers have a single sink pad at index 0. */
	MediaPad *sink = pads[0];
	const std::vector<MediaLink *> &links = sink->links();
	if (links.empty())
		return -ENODEV;

	MediaLink *link = links[0];
	MediaEntity *sensorEntity = link->source()->entity();
	if (sensorEntity->function() != MEDIA_ENT_F_CAM_SENSOR)
		return -ENODEV;

	ret = link->setEnabled(true);
	if (ret)
		return ret;

	/*
	 * \todo Define when to open and close video device nodes, as they
	 * might impact on power consumption.
	 */
	sensor_ = new V4L2Subdevice(sensorEntity);
	ret = sensor_->open();
	if (ret)
		return ret;

	csi2_ = new V4L2Subdevice(csi2Entity);
	ret = csi2_->open();
	if (ret)
		return ret;

	std::string cio2Name = "ipu3-cio2 " + std::to_string(index);
	output_ = V4L2Device::fromEntityName(media, cio2Name);
	ret = output_->open();
	if (ret)
		return ret;

	return 0;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU3);

} /* namespace libcamera */
