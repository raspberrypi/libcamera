/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vimc.cpp - Pipeline handler for the vimc device
 */

#include <algorithm>
#include <array>

#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "utils.h"
#include "v4l2_device.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(VIMC)

class VimcCameraData : public CameraData
{
public:
	VimcCameraData(PipelineHandler *pipe)
		: CameraData(pipe)
	{
	}

	~VimcCameraData()
	{
		delete video_;
	}

	void bufferReady(Buffer *buffer);

	V4L2Device *video_;
	Stream stream_;
};

class VimcCameraConfiguration : public CameraConfiguration
{
public:
	VimcCameraConfiguration();

	Status validate() override;
};

class PipelineHandlerVimc : public PipelineHandler
{
public:
	PipelineHandlerVimc(CameraManager *manager);

	CameraConfiguration *generateConfiguration(Camera *camera,
		const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int allocateBuffers(Camera *camera,
			    const std::set<Stream *> &streams) override;
	int freeBuffers(Camera *camera,
			const std::set<Stream *> &streams) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequest(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	VimcCameraData *cameraData(const Camera *camera)
	{
		return static_cast<VimcCameraData *>(
			PipelineHandler::cameraData(camera));
	}
};

VimcCameraConfiguration::VimcCameraConfiguration()
	: CameraConfiguration()
{
}

CameraConfiguration::Status VimcCameraConfiguration::validate()
{
	static const std::array<unsigned int, 3> formats{
		V4L2_PIX_FMT_BGR24,
		V4L2_PIX_FMT_RGB24,
		V4L2_PIX_FMT_ARGB32,
	};

	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];

	/* Adjust the pixel format. */
	if (std::find(formats.begin(), formats.end(), cfg.pixelFormat) ==
	    formats.end()) {
		LOG(VIMC, Debug) << "Adjusting format to RGB24";
		cfg.pixelFormat = V4L2_PIX_FMT_RGB24;
		status = Adjusted;
	}

	/* Clamp the size based on the device limits. */
	const Size size = cfg.size;

	cfg.size.width = std::max(16U, std::min(4096U, cfg.size.width));
	cfg.size.height = std::max(16U, std::min(2160U, cfg.size.height));

	if (cfg.size != size) {
		LOG(VIMC, Debug)
			<< "Adjusting size to " << cfg.size.toString();
		status = Adjusted;
	}

	cfg.bufferCount = 4;

	return status;
}

PipelineHandlerVimc::PipelineHandlerVimc(CameraManager *manager)
	: PipelineHandler(manager)
{
}

CameraConfiguration *PipelineHandlerVimc::generateConfiguration(Camera *camera,
	const StreamRoles &roles)
{
	CameraConfiguration *config = new VimcCameraConfiguration();

	if (roles.empty())
		return config;

	StreamConfiguration cfg{};
	cfg.pixelFormat = V4L2_PIX_FMT_RGB24;
	cfg.size = { 640, 480 };
	cfg.bufferCount = 4;

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int PipelineHandlerVimc::configure(Camera *camera, CameraConfiguration *config)
{
	VimcCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	int ret;

	V4L2DeviceFormat format = {};
	format.fourcc = cfg.pixelFormat;
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != cfg.pixelFormat)
		return -EINVAL;

	cfg.setStream(&data->stream_);

	return 0;
}

int PipelineHandlerVimc::allocateBuffers(Camera *camera,
					 const std::set<Stream *> &streams)
{
	VimcCameraData *data = cameraData(camera);
	Stream *stream = *streams.begin();
	const StreamConfiguration &cfg = stream->configuration();

	LOG(VIMC, Debug) << "Requesting " << cfg.bufferCount << " buffers";

	return data->video_->exportBuffers(&stream->bufferPool());
}

int PipelineHandlerVimc::freeBuffers(Camera *camera,
				     const std::set<Stream *> &streams)
{
	VimcCameraData *data = cameraData(camera);
	return data->video_->releaseBuffers();
}

int PipelineHandlerVimc::start(Camera *camera)
{
	VimcCameraData *data = cameraData(camera);
	return data->video_->streamOn();
}

void PipelineHandlerVimc::stop(Camera *camera)
{
	VimcCameraData *data = cameraData(camera);
	data->video_->streamOff();
	PipelineHandler::stop(camera);
}

int PipelineHandlerVimc::queueRequest(Camera *camera, Request *request)
{
	VimcCameraData *data = cameraData(camera);
	Buffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(VIMC, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	int ret = data->video_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	PipelineHandler::queueRequest(camera, request);

	return 0;
}

bool PipelineHandlerVimc::match(DeviceEnumerator *enumerator)
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

	MediaDevice *media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	std::unique_ptr<VimcCameraData> data = utils::make_unique<VimcCameraData>(this);

	/* Locate and open the capture video node. */
	data->video_ = new V4L2Device(media->getEntityByName("Raw Capture 1"));
	if (data->video_->open())
		return false;

	data->video_->bufferReady.connect(data.get(), &VimcCameraData::bufferReady);

	/* Create and register the camera. */
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, "VIMC Sensor B",
							streams);
	registerCamera(std::move(camera), std::move(data));

	return true;
}

void VimcCameraData::bufferReady(Buffer *buffer)
{
	Request *request = queuedRequests_.front();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerVimc);

} /* namespace libcamera */
