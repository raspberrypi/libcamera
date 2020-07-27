/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * uvcvideo.cpp - Pipeline handler for uvcvideo devices
 */

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <tuple>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/sysfs.h"
#include "libcamera/internal/utils.h"
#include "libcamera/internal/v4l2_controls.h"
#include "libcamera/internal/v4l2_videodevice.h"

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
	void addControl(uint32_t cid, const ControlInfo &v4l2info,
			ControlInfoMap::Map *ctrls);
	void bufferReady(FrameBuffer *buffer);

	V4L2VideoDevice *video_;
	Stream stream_;
};

class UVCCameraConfiguration : public CameraConfiguration
{
public:
	UVCCameraConfiguration(UVCCameraData *data);

	Status validate() override;

private:
	UVCCameraData *data_;
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

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	std::string generateId(const UVCCameraData *data);

	int processControl(ControlList *controls, unsigned int id,
			   const ControlValue &value);
	int processControls(UVCCameraData *data, Request *request);

	UVCCameraData *cameraData(const Camera *camera)
	{
		return static_cast<UVCCameraData *>(
			PipelineHandler::cameraData(camera));
	}
};

UVCCameraConfiguration::UVCCameraConfiguration(UVCCameraData *data)
	: CameraConfiguration(), data_(data)
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
			<< "Adjusting pixel format from "
			<< pixelFormat.toString() << " to "
			<< cfg.pixelFormat.toString();
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

	V4L2DeviceFormat format = {};
	format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	int ret = data_->video_->tryFormat(&format);
	if (ret)
		return Invalid;

	cfg.stride = format.planes[0].bpl;
	cfg.frameSize = format.planes[0].size;

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
	CameraConfiguration *config = new UVCCameraConfiguration(data);

	if (roles.empty())
		return config;

	V4L2VideoDevice::Formats v4l2Formats = data->video_->formats();
	std::map<PixelFormat, std::vector<SizeRange>> deviceFormats;
	for (const auto &format : v4l2Formats) {
		PixelFormat pixelFormat = format.first.toPixelFormat();
		if (pixelFormat.isValid())
			deviceFormats[pixelFormat] = format.second;
	}

	StreamFormats formats(deviceFormats);
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
	format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != data->video_->toV4L2PixelFormat(cfg.pixelFormat))
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

int PipelineHandlerUVC::start(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	unsigned int count = data->stream_.configuration().bufferCount;

	int ret = data->video_->importBuffers(count);
	if (ret < 0)
		return ret;

	ret = data->video_->streamOn();
	if (ret < 0) {
		data->video_->releaseBuffers();
		return ret;
	}

	return 0;
}

void PipelineHandlerUVC::stop(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	data->video_->streamOff();
	data->video_->releaseBuffers();
}

int PipelineHandlerUVC::processControl(ControlList *controls, unsigned int id,
				       const ControlValue &value)
{
	uint32_t cid;

	if (id == controls::Brightness)
		cid = V4L2_CID_BRIGHTNESS;
	else if (id == controls::Contrast)
		cid = V4L2_CID_CONTRAST;
	else if (id == controls::Saturation)
		cid = V4L2_CID_SATURATION;
	else if (id == controls::AeEnable)
		cid = V4L2_CID_EXPOSURE_AUTO;
	else if (id == controls::ExposureTime)
		cid = V4L2_CID_EXPOSURE_ABSOLUTE;
	else if (id == controls::AnalogueGain)
		cid = V4L2_CID_GAIN;
	else
		return -EINVAL;

	const ControlInfo &v4l2Info = controls->infoMap()->at(cid);
	int32_t min = v4l2Info.min().get<int32_t>();
	int32_t def = v4l2Info.def().get<int32_t>();
	int32_t max = v4l2Info.max().get<int32_t>();

	/*
	 * See UVCCameraData::addControl() for explanations of the different
	 * value mappings.
	 */
	switch (cid) {
	case V4L2_CID_BRIGHTNESS: {
		float scale = std::max(max - def, def - min);
		float fvalue = value.get<float>() * scale + def;
		controls->set(cid, static_cast<int32_t>(lroundf(fvalue)));
		break;
	}

	case V4L2_CID_SATURATION: {
		float scale = def - min;
		float fvalue = value.get<float>() * scale + min;
		controls->set(cid, static_cast<int32_t>(lroundf(fvalue)));
		break;
	}

	case V4L2_CID_EXPOSURE_AUTO: {
		int32_t ivalue = value.get<bool>()
			       ? V4L2_EXPOSURE_APERTURE_PRIORITY
			       : V4L2_EXPOSURE_MANUAL;
		controls->set(V4L2_CID_EXPOSURE_AUTO, ivalue);
		break;
	}

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		controls->set(cid, value.get<int32_t>() / 100);
		break;

	case V4L2_CID_CONTRAST:
	case V4L2_CID_GAIN: {
		float m = (4.0f - 1.0f) / (max - def);
		float p = 1.0f - m * def;

		if (m * min + p < 0.5f) {
			m = (1.0f - 0.5f) / (def - min);
			p = 1.0f - m * def;
		}

		float fvalue = (value.get<float>() - p) / m;
		controls->set(cid, static_cast<int32_t>(lroundf(fvalue)));
		break;
	}

	default: {
		int32_t ivalue = value.get<int32_t>();
		controls->set(cid, ivalue);
		break;
	}
	}

	return 0;
}

int PipelineHandlerUVC::processControls(UVCCameraData *data, Request *request)
{
	ControlList controls(data->video_->controls());

	for (auto it : request->controls()) {
		unsigned int id = it.first;
		ControlValue &value = it.second;

		processControl(&controls, id, value);
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

std::string PipelineHandlerUVC::generateId(const UVCCameraData *data)
{
	const std::string path = data->video_->devicePath();

	/* Create a controller ID from first device described in firmware. */
	std::string controllerId;
	std::string searchPath = path;
	while (true) {
		std::string::size_type pos = searchPath.rfind('/');
		if (pos <= 1) {
			LOG(UVC, Error) << "Can not find controller ID";
			return {};
		}

		searchPath = searchPath.substr(0, pos);

		controllerId = sysfs::firmwareNodePath(searchPath);
		if (!controllerId.empty())
			break;
	}

	/*
	 * Create a USB ID from the device path which has the known format:
	 *
	 *	path = bus, "-", ports, ":", config, ".", interface ;
	 *	bus = number ;
	 *	ports = port, [ ".", ports ] ;
	 *	port = number ;
	 *	config = number ;
	 *	interface = number ;
	 *
	 * Example: 3-2.4:1.0
	 *
	 * The bus is not guaranteed to be stable and needs to be stripped from
	 * the USB ID. The final USB ID is built up of the ports, config and
	 * interface properties.
	 *
	 * Example 2.4:1.0.
	 */
	std::string usbId = utils::basename(path.c_str());
	usbId = usbId.substr(usbId.find('-') + 1);

	/* Creata a device ID from the USB devices vendor and product ID. */
	std::string deviceId;
	for (const std::string &name : { "idVendor", "idProduct" }) {
		std::ifstream file(path + "/../" + name);

		if (!file.is_open())
			return {};

		std::string value;
		std::getline(file, value);
		file.close();

		if (!deviceId.empty())
			deviceId += ":";

		deviceId += value;
	}

	return controllerId + "-" + usbId + "-" + deviceId;
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

	/* Create and register the camera. */
	std::string id = generateId(data.get());
	if (id.empty()) {
		LOG(UVC, Error) << "Failed to generate camera ID";
		return false;
	}

	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera = Camera::create(this, id, streams);
	registerCamera(std::move(camera), std::move(data));

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
	ControlInfoMap::Map ctrls;

	for (const auto &ctrl : video_->controls()) {
		uint32_t cid = ctrl.first->id();
		const ControlInfo &info = ctrl.second;

		addControl(cid, info, &ctrls);
	}

	controlInfo_ = std::move(ctrls);

	return 0;
}

void UVCCameraData::addControl(uint32_t cid, const ControlInfo &v4l2Info,
			       ControlInfoMap::Map *ctrls)
{
	const ControlId *id;
	ControlInfo info;

	/* Map the control ID. */
	switch (cid) {
	case V4L2_CID_BRIGHTNESS:
		id = &controls::Brightness;
		break;
	case V4L2_CID_CONTRAST:
		id = &controls::Contrast;
		break;
	case V4L2_CID_SATURATION:
		id = &controls::Saturation;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		id = &controls::AeEnable;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		id = &controls::ExposureTime;
		break;
	case V4L2_CID_GAIN:
		id = &controls::AnalogueGain;
		break;
	default:
		return;
	}

	/* Map the control info. */
	int32_t min = v4l2Info.min().get<int32_t>();
	int32_t max = v4l2Info.max().get<int32_t>();
	int32_t def = v4l2Info.def().get<int32_t>();

	switch (cid) {
	case V4L2_CID_BRIGHTNESS: {
		/*
		 * The Brightness control is a float, with 0.0 mapped to the
		 * default value. The control range is [-1.0, 1.0], but the V4L2
		 * default may not be in the middle of the V4L2 range.
		 * Accommodate this by restricting the range of the libcamera
		 * control, but always within the maximum limits.
		 */
		float scale = std::max(max - def, def - min);

		info = ControlInfo{
			{ static_cast<float>(min - def) / scale },
			{ static_cast<float>(max - def) / scale },
			{ 0.0f }
		};
		break;
	}

	case V4L2_CID_SATURATION:
		/*
		 * The Saturation control is a float, with 0.0 mapped to the
		 * minimum value (corresponding to a fully desaturated image)
		 * and 1.0 mapped to the default value. Calculate the maximum
		 * value accordingly.
		 */
		info = ControlInfo{
			{ 0.0f },
			{ static_cast<float>(max - min) / (def - min) },
			{ 1.0f }
		};
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		info = ControlInfo{ false, true, true };
		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/*
		 * ExposureTime is in units of 1 µs, and UVC expects
		 * V4L2_CID_EXPOSURE_ABSOLUTE in units of 100 µs.
		 */
		info = ControlInfo{
			{ min * 100 },
			{ max * 100 },
			{ def * 100 }
		};
		break;

	case V4L2_CID_CONTRAST:
	case V4L2_CID_GAIN: {
		/*
		 * The Contrast and AnalogueGain controls are floats, with 1.0
		 * mapped to the default value. UVC doesn't specify units, and
		 * cameras have been seen to expose very different ranges for
		 * the controls. Arbitrarily assume that the minimum and
		 * maximum values are respectively no lower than 0.5 and no
		 * higher than 4.0.
		 */
		float m = (4.0f - 1.0f) / (max - def);
		float p = 1.0f - m * def;

		if (m * min + p < 0.5f) {
			m = (1.0f - 0.5f) / (def - min);
			p = 1.0f - m * def;
		}

		info = ControlInfo{
			{ m * min + p },
			{ m * max + p },
			{ 1.0f }
		};
		break;
	}

	default:
		info = v4l2Info;
		break;
	}

	ctrls->emplace(id, info);
}

void UVCCameraData::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC);

} /* namespace libcamera */
