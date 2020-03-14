/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vimc.cpp - Pipeline handler for the vimc device
 */

#include <algorithm>
#include <array>
#include <iomanip>
#include <tuple>

#include <linux/media-bus-format.h>

#include <ipa/ipa_interface.h>
#include <ipa/ipa_module_info.h>
#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "camera_sensor.h"
#include "device_enumerator.h"
#include "ipa_manager.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "utils.h"
#include "v4l2_controls.h"
#include "v4l2_subdevice.h"
#include "v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(VIMC)

class VimcCameraData : public CameraData
{
public:
	VimcCameraData(PipelineHandler *pipe)
		: CameraData(pipe), sensor_(nullptr), debayer_(nullptr),
		  scaler_(nullptr), video_(nullptr), raw_(nullptr)
	{
	}

	~VimcCameraData()
	{
		delete sensor_;
		delete debayer_;
		delete scaler_;
		delete video_;
		delete raw_;
	}

	int init(MediaDevice *media);
	void bufferReady(FrameBuffer *buffer);

	CameraSensor *sensor_;
	V4L2Subdevice *debayer_;
	V4L2Subdevice *scaler_;
	V4L2VideoDevice *video_;
	V4L2VideoDevice *raw_;
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

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	int importFrameBuffers(Camera *camera, Stream *stream) override;
	void freeFrameBuffers(Camera *camera, Stream *stream) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	int processControls(VimcCameraData *data, Request *request);

	VimcCameraData *cameraData(const Camera *camera)
	{
		return static_cast<VimcCameraData *>(
			PipelineHandler::cameraData(camera));
	}
};

namespace {

static const std::array<PixelFormat, 3> pixelformats{
	PixelFormat(DRM_FORMAT_RGB888),
	PixelFormat(DRM_FORMAT_BGR888),
	PixelFormat(DRM_FORMAT_BGRA8888),
};

} /* namespace */

VimcCameraConfiguration::VimcCameraConfiguration()
	: CameraConfiguration()
{
}

CameraConfiguration::Status VimcCameraConfiguration::validate()
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

	/* Adjust the pixel format. */
	if (std::find(pixelformats.begin(), pixelformats.end(), cfg.pixelFormat) ==
	    pixelformats.end()) {
		LOG(VIMC, Debug) << "Adjusting format to RGB24";
		cfg.pixelFormat = PixelFormat(DRM_FORMAT_BGR888);
		status = Adjusted;
	}

	/* Clamp the size based on the device limits. */
	const Size size = cfg.size;

	/* The scaler hardcodes a x3 scale-up ratio. */
	cfg.size.width = std::max(48U, std::min(4096U, cfg.size.width));
	cfg.size.height = std::max(48U, std::min(2160U, cfg.size.height));
	cfg.size.width -= cfg.size.width % 3;
	cfg.size.height -= cfg.size.height % 3;

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

	std::map<PixelFormat, std::vector<SizeRange>> formats;

	for (PixelFormat pixelformat : pixelformats) {
		/* The scaler hardcodes a x3 scale-up ratio. */
		std::vector<SizeRange> sizes{
			SizeRange{ 48, 48, 4096, 2160 }
		};
		formats[pixelformat] = sizes;
	}

	StreamConfiguration cfg(formats);

	cfg.pixelFormat = PixelFormat(DRM_FORMAT_BGR888);
	cfg.size = { 1920, 1080 };
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

	/* The scaler hardcodes a x3 scale-up ratio. */
	V4L2SubdeviceFormat subformat = {};
	subformat.mbus_code = MEDIA_BUS_FMT_SGRBG8_1X8;
	subformat.size = { cfg.size.width / 3, cfg.size.height / 3 };

	ret = data->sensor_->setFormat(&subformat);
	if (ret)
		return ret;

	ret = data->debayer_->setFormat(0, &subformat);
	if (ret)
		return ret;

	subformat.mbus_code = MEDIA_BUS_FMT_RGB888_1X24;
	ret = data->debayer_->setFormat(1, &subformat);
	if (ret)
		return ret;

	ret = data->scaler_->setFormat(0, &subformat);
	if (ret)
		return ret;

	subformat.size = cfg.size;
	ret = data->scaler_->setFormat(1, &subformat);
	if (ret)
		return ret;

	V4L2DeviceFormat format = {};
	format.fourcc = data->video_->toV4L2Fourcc(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != data->video_->toV4L2Fourcc(cfg.pixelFormat))
		return -EINVAL;

	/*
	 * Format has to be set on the raw capture video node, otherwise the
	 * vimc driver will fail pipeline validation.
	 */
	format.fourcc = V4L2_PIX_FMT_SGRBG8;
	format.size = { cfg.size.width / 3, cfg.size.height / 3 };

	ret = data->raw_->setFormat(&format);
	if (ret)
		return ret;

	cfg.setStream(&data->stream_);

	return 0;
}

int PipelineHandlerVimc::exportFrameBuffers(Camera *camera, Stream *stream,
					    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	VimcCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->allocateBuffers(count, buffers);
}

int PipelineHandlerVimc::importFrameBuffers(Camera *camera, Stream *stream)
{
	VimcCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->importBuffers(count);
}

void PipelineHandlerVimc::freeFrameBuffers(Camera *camera, Stream *stream)
{
	VimcCameraData *data = cameraData(camera);

	data->video_->releaseBuffers();
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
}

int PipelineHandlerVimc::processControls(VimcCameraData *data, Request *request)
{
	ControlList controls(data->sensor_->controls());

	for (auto it : request->controls()) {
		unsigned int id = it.first;
		ControlValue &value = it.second;

		if (id == controls::Brightness)
			controls.set(V4L2_CID_BRIGHTNESS, value);
		else if (id == controls::Contrast)
			controls.set(V4L2_CID_CONTRAST, value);
		else if (id == controls::Saturation)
			controls.set(V4L2_CID_SATURATION, value);
	}

	for (const auto &ctrl : controls)
		LOG(VIMC, Debug)
			<< "Setting control " << utils::hex(ctrl.first)
			<< " to " << ctrl.second.toString();

	int ret = data->sensor_->setControls(&controls);
	if (ret) {
		LOG(VIMC, Error) << "Failed to set controls: " << ret;
		return ret < 0 ? ret : -EINVAL;
	}

	return ret;
}

int PipelineHandlerVimc::queueRequestDevice(Camera *camera, Request *request)
{
	VimcCameraData *data = cameraData(camera);
	FrameBuffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(VIMC, Error)
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

	std::unique_ptr<VimcCameraData> data = std::make_unique<VimcCameraData>(this);

	data->ipa_ = IPAManager::instance()->createIPA(this, 0, 0);
	if (data->ipa_ == nullptr)
		LOG(VIMC, Warning) << "no matching IPA found";
	else
		data->ipa_->init();

	/* Locate and open the capture video node. */
	if (data->init(media))
		return false;

	/* Create and register the camera. */
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, "VIMC Sensor B",
							streams);
	registerCamera(std::move(camera), std::move(data));

	return true;
}

int VimcCameraData::init(MediaDevice *media)
{
	int ret;

	ret = media->disableLinks();
	if (ret < 0)
		return ret;

	MediaLink *link = media->link("Debayer B", 1, "Scaler", 0);
	if (!link)
		return -ENODEV;

	ret = link->setEnabled(true);
	if (ret < 0)
		return ret;

	/* Create and open the camera sensor, debayer, scaler and video device. */
	sensor_ = new CameraSensor(media->getEntityByName("Sensor B"));
	ret = sensor_->init();
	if (ret)
		return ret;

	debayer_ = new V4L2Subdevice(media->getEntityByName("Debayer B"));
	if (debayer_->open())
		return -ENODEV;

	scaler_ = new V4L2Subdevice(media->getEntityByName("Scaler"));
	if (scaler_->open())
		return -ENODEV;

	video_ = new V4L2VideoDevice(media->getEntityByName("RGB/YUV Capture"));
	if (video_->open())
		return -ENODEV;

	video_->bufferReady.connect(this, &VimcCameraData::bufferReady);

	raw_ = new V4L2VideoDevice(media->getEntityByName("Raw Capture 1"));
	if (raw_->open())
		return -ENODEV;

	/* Initialise the supported controls. */
	const ControlInfoMap &controls = sensor_->controls();
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
		default:
			continue;
		}

		ctrls.emplace(id, range);
	}

	controlInfo_ = std::move(ctrls);

	/* Initialize the camera properties. */
	properties_ = sensor_->properties();

	return 0;
}

void VimcCameraData::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerVimc);

} /* namespace libcamera */
