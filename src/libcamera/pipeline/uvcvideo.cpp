/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <algorithm>
#include <iomanip>
#include <sys/sysmacros.h>
#include <tuple>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "utils.h"
#include "v4l2_controls.h"
#include "v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(UVC)

class UVCCameraData : public CameraData
{
public:
	UVCCameraData(PipelineHandler *pipe)
		: CameraData(pipe), video_(nullptr)
	{
	}

	~UVCCameraData()
	{
		delete video_;
	}

	int init(MediaEntity *entity);
	void bufferReady(FrameBuffer *buffer);

	V4L2VideoDevice *video_;
	Stream stream_;
};

class UVCCameraConfiguration : public CameraConfiguration
{
public:
	UVCCameraConfiguration();

	Status validate() override;
};

class PipelineHandlerUVC : public PipelineHandler
{
public:
	PipelineHandlerUVC(CameraManager *manager);

	CameraConfiguration *generateConfiguration(Camera *camera,
		const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	int importFrameBuffers(Camera *camera, Stream *stream) override;
	void freeFrameBuffers(Camera *camera, Stream *stream) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	int processControls(UVCCameraData *data, Request *request);

	UVCCameraData *cameraData(const Camera *camera)
	{
		return static_cast<UVCCameraData *>(
			PipelineHandler::cameraData(camera));
	}
};

UVCCameraConfiguration::UVCCameraConfiguration()
	: CameraConfiguration()
{
}

CameraConfiguration::Status UVCCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];
	const StreamFormats &formats = cfg.formats();
	const PixelFormat pixelFormat = cfg.pixelFormat;
	const Size size = cfg.size;

	const std::vector<PixelFormat> pixelFormats = formats.pixelformats();
	auto iter = std::find(pixelFormats.begin(), pixelFormats.end(), pixelFormat);
	if (iter == pixelFormats.end()) {
		cfg.pixelFormat = pixelFormats.front();
		LOG(UVC, Debug)
			<< "Adjusting pixel format from " << pixelFormat
			<< " to " << cfg.pixelFormat;
		status = Adjusted;
	}

	const std::vector<Size> &formatSizes = formats.sizes(cfg.pixelFormat);
	cfg.size = formatSizes.front();
	for (const Size &formatsSize : formatSizes) {
		if (formatsSize > size)
			break;

		cfg.size = formatsSize;
	}

	if (cfg.size != size) {
		LOG(UVC, Debug)
			<< "Adjusting size from " << size.toString()
			<< " to " << cfg.size.toString();
		status = Adjusted;
	}

	cfg.bufferCount = 4;

	return status;
}

PipelineHandlerUVC::PipelineHandlerUVC(CameraManager *manager)
	: PipelineHandler(manager)
{
}

CameraConfiguration *PipelineHandlerUVC::generateConfiguration(Camera *camera,
	const StreamRoles &roles)
{
	UVCCameraData *data = cameraData(camera);
	CameraConfiguration *config = new UVCCameraConfiguration();

	if (roles.empty())
		return config;

	ImageFormats v4l2formats = data->video_->formats();
	StreamFormats formats(v4l2formats.data());
	StreamConfiguration cfg(formats);

	cfg.pixelFormat = formats.pixelformats().front();
	cfg.size = formats.sizes(cfg.pixelFormat).back();
	cfg.bufferCount = 4;

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int PipelineHandlerUVC::configure(Camera *camera, CameraConfiguration *config)
{
	UVCCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	int ret;

	V4L2DeviceFormat format = {};
	format.fourcc = data->video_->toV4L2Fourcc(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != data->video_->toV4L2Fourcc(cfg.pixelFormat))
		return -EINVAL;

	cfg.setStream(&data->stream_);

	return 0;
}

int PipelineHandlerUVC::exportFrameBuffers(Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	UVCCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->exportBuffers(count, buffers);
}

int PipelineHandlerUVC::importFrameBuffers(Camera *camera, Stream *stream)
{
	UVCCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->importBuffers(count);
}

void PipelineHandlerUVC::freeFrameBuffers(Camera *camera, Stream *stream)
{
	UVCCameraData *data = cameraData(camera);

	data->video_->releaseBuffers();
}

int PipelineHandlerUVC::start(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	return data->video_->streamOn();
}

void PipelineHandlerUVC::stop(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	data->video_->streamOff();
}

int PipelineHandlerUVC::processControls(UVCCameraData *data, Request *request)
{
	ControlList controls(data->video_->controls());

	for (auto it : request->controls()) {
		unsigned int id = it.first;
		ControlValue &value = it.second;

		if (id == controls::Brightness) {
			controls.set(V4L2_CID_BRIGHTNESS, value);
		} else if (id == controls::Contrast) {
			controls.set(V4L2_CID_CONTRAST, value);
		} else if (id == controls::Saturation) {
			controls.set(V4L2_CID_SATURATION, value);
		} else if (id == controls::ManualExposure) {
			controls.set(V4L2_CID_EXPOSURE_AUTO, static_cast<int32_t>(1));
			controls.set(V4L2_CID_EXPOSURE_ABSOLUTE, value);
		} else if (id == controls::ManualGain) {
			controls.set(V4L2_CID_GAIN, value);
		}
	}

	for (const auto &ctrl : controls)
		LOG(UVC, Debug)
			<< "Setting control " << utils::hex(ctrl.first)
			<< " to " << ctrl.second.toString();

	int ret = data->video_->setControls(&controls);
	if (ret) {
		LOG(UVC, Error) << "Failed to set controls: " << ret;
		return ret < 0 ? ret : -EINVAL;
	}

	return ret;
}

int PipelineHandlerUVC::queueRequestDevice(Camera *camera, Request *request)
{
	UVCCameraData *data = cameraData(camera);
	FrameBuffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(UVC, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	int ret = processControls(data, request);
	if (ret < 0)
		return ret;

	ret = data->video_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	return 0;
}

bool PipelineHandlerUVC::match(DeviceEnumerator *enumerator)
{
	MediaDevice *media;
	DeviceMatch dm("uvcvideo");

	media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	std::unique_ptr<UVCCameraData> data = std::make_unique<UVCCameraData>(this);

	/* Locate and initialise the camera data with the default video node. */
	const std::vector<MediaEntity *> &entities = media->entities();
	auto entity = std::find_if(entities.begin(), entities.end(),
				   [](MediaEntity *entity) {
					   return entity->flags() & MEDIA_ENT_FL_DEFAULT;
				   });
	if (entity == entities.end()) {
		LOG(UVC, Error) << "Could not find a default video device";
		return false;
	}

	if (data->init(*entity))
		return false;

	dev_t devnum = makedev((*entity)->deviceMajor(), (*entity)->deviceMinor());

	/* Create and register the camera. */
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, media->model(), streams);
	registerCamera(std::move(camera), std::move(data), devnum);

	/* Enable hot-unplug notifications. */
	hotplugMediaDevice(media);

	return true;
}

int UVCCameraData::init(MediaEntity *entity)
{
	int ret;

	/* Create and open the video device. */
	video_ = new V4L2VideoDevice(entity);
	ret = video_->open();
	if (ret)
		return ret;

	video_->bufferReady.connect(this, &UVCCameraData::bufferReady);

	/* Initialise the supported controls. */
	const ControlInfoMap &controls = video_->controls();
	ControlInfoMap::Map ctrls;

	for (const auto &ctrl : controls) {
		const ControlRange &range = ctrl.second;
		const ControlId *id;

		switch (ctrl.first->id()) {
		case V4L2_CID_BRIGHTNESS:
			id = &controls::Brightness;
			break;
		case V4L2_CID_CONTRAST:
			id = &controls::Contrast;
			break;
		case V4L2_CID_SATURATION:
			id = &controls::Saturation;
			break;
		case V4L2_CID_EXPOSURE_ABSOLUTE:
			id = &controls::ManualExposure;
			break;
		case V4L2_CID_GAIN:
			id = &controls::ManualGain;
			break;
		default:
			continue;
		}

		ctrls.emplace(id, range);
	}

	controlInfo_ = std::move(ctrls);

	return 0;
}

void UVCCameraData::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
