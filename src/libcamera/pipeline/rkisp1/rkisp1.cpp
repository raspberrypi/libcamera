/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * rkisp1.cpp - Pipeline handler for Rockchip ISP1
 */

#include <algorithm>
#include <array>
#include <iomanip>
#include <memory>
#include <vector>

#include <linux/media-bus-format.h>

#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "camera_sensor.h"
#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "utils.h"
#include "v4l2_device.h"
#include "v4l2_subdevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(RkISP1)

class RkISP1CameraData : public CameraData
{
public:
	RkISP1CameraData(PipelineHandler *pipe)
		: CameraData(pipe), sensor_(nullptr)
	{
	}

	~RkISP1CameraData()
	{
		delete sensor_;
	}

	Stream stream_;
	CameraSensor *sensor_;
};

class RkISP1CameraConfiguration : public CameraConfiguration
{
public:
	RkISP1CameraConfiguration(Camera *camera, RkISP1CameraData *data);

	Status validate() override;

	const V4L2SubdeviceFormat &sensorFormat() { return sensorFormat_; }

private:
	static constexpr unsigned int RKISP1_BUFFER_COUNT = 4;

	/*
	 * The RkISP1CameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	const RkISP1CameraData *data_;

	V4L2SubdeviceFormat sensorFormat_;
};

class PipelineHandlerRkISP1 : public PipelineHandler
{
public:
	PipelineHandlerRkISP1(CameraManager *manager);
	~PipelineHandlerRkISP1();

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
	RkISP1CameraData *cameraData(const Camera *camera)
	{
		return static_cast<RkISP1CameraData *>(
			PipelineHandler::cameraData(camera));
	}

	int initLinks();
	int createCamera(MediaEntity *sensor);
	void bufferReady(Buffer *buffer);

	MediaDevice *media_;
	V4L2Subdevice *dphy_;
	V4L2Subdevice *isp_;
	V4L2Device *video_;

	Camera *activeCamera_;
};

RkISP1CameraConfiguration::RkISP1CameraConfiguration(Camera *camera,
						     RkISP1CameraData *data)
	: CameraConfiguration()
{
	camera_ = camera->shared_from_this();
	data_ = data;
}

CameraConfiguration::Status RkISP1CameraConfiguration::validate()
{
	static const std::array<unsigned int, 8> formats{
		V4L2_PIX_FMT_YUYV,
		V4L2_PIX_FMT_YVYU,
		V4L2_PIX_FMT_VYUY,
		V4L2_PIX_FMT_NV16,
		V4L2_PIX_FMT_NV61,
		V4L2_PIX_FMT_NV21,
		V4L2_PIX_FMT_NV12,
		V4L2_PIX_FMT_GREY,
	};

	const CameraSensor *sensor = data_->sensor_;
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
		LOG(RkISP1, Debug) << "Adjusting format to NV12";
		cfg.pixelFormat = V4L2_PIX_FMT_NV12;
		status = Adjusted;
	}

	/* Select the sensor format. */
	sensorFormat_ = sensor->getFormat({ MEDIA_BUS_FMT_SBGGR12_1X12,
					    MEDIA_BUS_FMT_SGBRG12_1X12,
					    MEDIA_BUS_FMT_SGRBG12_1X12,
					    MEDIA_BUS_FMT_SRGGB12_1X12,
					    MEDIA_BUS_FMT_SBGGR10_1X10,
					    MEDIA_BUS_FMT_SGBRG10_1X10,
					    MEDIA_BUS_FMT_SGRBG10_1X10,
					    MEDIA_BUS_FMT_SRGGB10_1X10,
					    MEDIA_BUS_FMT_SBGGR8_1X8,
					    MEDIA_BUS_FMT_SGBRG8_1X8,
					    MEDIA_BUS_FMT_SGRBG8_1X8,
					    MEDIA_BUS_FMT_SRGGB8_1X8 },
					  cfg.size);
	if (!sensorFormat_.size.width || !sensorFormat_.size.height)
		sensorFormat_.size = sensor->resolution();

	/*
	 * Provide a suitable default that matches the sensor aspect
	 * ratio and clamp the size to the hardware bounds.
	 *
	 * \todo: Check the hardware alignment constraints.
	 */
	const Size size = cfg.size;

	if (!cfg.size.width || !cfg.size.height) {
		cfg.size.width = 1280;
		cfg.size.height = 1280 * sensorFormat_.size.height
				/ sensorFormat_.size.width;
	}

	cfg.size.width = std::max(32U, std::min(4416U, cfg.size.width));
	cfg.size.height = std::max(16U, std::min(3312U, cfg.size.height));

	if (cfg.size != size) {
		LOG(RkISP1, Debug)
			<< "Adjusting size from " << size.toString()
			<< " to " << cfg.size.toString();
		status = Adjusted;
	}

	cfg.bufferCount = RKISP1_BUFFER_COUNT;

	return status;
}

PipelineHandlerRkISP1::PipelineHandlerRkISP1(CameraManager *manager)
	: PipelineHandler(manager), dphy_(nullptr), isp_(nullptr),
	  video_(nullptr)
{
}

PipelineHandlerRkISP1::~PipelineHandlerRkISP1()
{
	delete video_;
	delete isp_;
	delete dphy_;
}

/* -----------------------------------------------------------------------------
 * Pipeline Operations
 */

CameraConfiguration *PipelineHandlerRkISP1::generateConfiguration(Camera *camera,
	const StreamRoles &roles)
{
	RkISP1CameraData *data = cameraData(camera);
	CameraConfiguration *config = new RkISP1CameraConfiguration(camera, data);

	if (roles.empty())
		return config;

	StreamConfiguration cfg{};
	cfg.pixelFormat = V4L2_PIX_FMT_NV12;
	cfg.size = data->sensor_->resolution();

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int PipelineHandlerRkISP1::configure(Camera *camera, CameraConfiguration *c)
{
	RkISP1CameraConfiguration *config =
		static_cast<RkISP1CameraConfiguration *>(c);
	RkISP1CameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	CameraSensor *sensor = data->sensor_;
	int ret;

	/*
	 * Configure the sensor links: enable the link corresponding to this
	 * camera and disable all the other sensor links.
	 */
	const MediaPad *pad = dphy_->entity()->getPadByIndex(0);

	for (MediaLink *link : pad->links()) {
		bool enable = link->source()->entity() == sensor->entity();

		if (!!(link->flags() & MEDIA_LNK_FL_ENABLED) == enable)
			continue;

		LOG(RkISP1, Debug)
			<< (enable ? "Enabling" : "Disabling")
			<< " link from sensor '"
			<< link->source()->entity()->name()
			<< "' to CSI-2 receiver";

		ret = link->setEnabled(enable);
		if (ret < 0)
			return ret;
	}

	/*
	 * Configure the format on the sensor output and propagate it through
	 * the pipeline.
	 */
	V4L2SubdeviceFormat format = config->sensorFormat();
	LOG(RkISP1, Debug) << "Configuring sensor with " << format.toString();

	ret = sensor->setFormat(&format);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug) << "Sensor configured with " << format.toString();

	ret = dphy_->setFormat(0, &format);
	if (ret < 0)
		return ret;

	ret = dphy_->getFormat(1, &format);
	if (ret < 0)
		return ret;

	ret = isp_->setFormat(0, &format);
	if (ret < 0)
		return ret;

	V4L2DeviceFormat outputFormat = {};
	outputFormat.fourcc = cfg.pixelFormat;
	outputFormat.size = cfg.size;
	outputFormat.planesCount = 2;

	ret = video_->setFormat(&outputFormat);
	if (ret)
		return ret;

	if (outputFormat.size != cfg.size ||
	    outputFormat.fourcc != cfg.pixelFormat) {
		LOG(RkISP1, Error)
			<< "Unable to configure capture in " << cfg.toString();
		return -EINVAL;
	}

	cfg.setStream(&data->stream_);

	return 0;
}

int PipelineHandlerRkISP1::allocateBuffers(Camera *camera,
					   const std::set<Stream *> &streams)
{
	Stream *stream = *streams.begin();
	return video_->exportBuffers(&stream->bufferPool());
}

int PipelineHandlerRkISP1::freeBuffers(Camera *camera,
				       const std::set<Stream *> &streams)
{
	if (video_->releaseBuffers())
		LOG(RkISP1, Error) << "Failed to release buffers";

	return 0;
}

int PipelineHandlerRkISP1::start(Camera *camera)
{
	int ret;

	ret = video_->streamOn();
	if (ret)
		LOG(RkISP1, Error)
			<< "Failed to start camera " << camera->name();

	activeCamera_ = camera;

	return ret;
}

void PipelineHandlerRkISP1::stop(Camera *camera)
{
	int ret;

	ret = video_->streamOff();
	if (ret)
		LOG(RkISP1, Warning)
			<< "Failed to stop camera " << camera->name();

	PipelineHandler::stop(camera);

	activeCamera_ = nullptr;
}

int PipelineHandlerRkISP1::queueRequest(Camera *camera, Request *request)
{
	RkISP1CameraData *data = cameraData(camera);
	Stream *stream = &data->stream_;

	Buffer *buffer = request->findBuffer(stream);
	if (!buffer) {
		LOG(RkISP1, Error)
			<< "Attempt to queue request with invalid stream";
		return -ENOENT;
	}

	int ret = video_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	PipelineHandler::queueRequest(camera, request);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

int PipelineHandlerRkISP1::initLinks()
{
	MediaLink *link;
	int ret;

	ret = media_->disableLinks();
	if (ret < 0)
		return ret;

	link = media_->link("rockchip-sy-mipi-dphy", 1, "rkisp1-isp-subdev", 0);
	if (!link)
		return -ENODEV;

	ret = link->setEnabled(true);
	if (ret < 0)
		return ret;

	link = media_->link("rkisp1-isp-subdev", 2, "rkisp1_mainpath", 0);
	if (!link)
		return -ENODEV;

	ret = link->setEnabled(true);
	if (ret < 0)
		return ret;

	return 0;
}

int PipelineHandlerRkISP1::createCamera(MediaEntity *sensor)
{
	int ret;

	std::unique_ptr<RkISP1CameraData> data =
		utils::make_unique<RkISP1CameraData>(this);

	data->sensor_ = new CameraSensor(sensor);
	ret = data->sensor_->init();
	if (ret)
		return ret;

	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera =
		Camera::create(this, sensor->name(), streams);
	registerCamera(std::move(camera), std::move(data));

	return 0;
}

bool PipelineHandlerRkISP1::match(DeviceEnumerator *enumerator)
{
	const MediaPad *pad;

	DeviceMatch dm("rkisp1");
	dm.add("rkisp1-isp-subdev");
	dm.add("rkisp1_selfpath");
	dm.add("rkisp1_mainpath");
	dm.add("rkisp1-statistics");
	dm.add("rkisp1-input-params");
	dm.add("rockchip-sy-mipi-dphy");

	media_ = acquireMediaDevice(enumerator, dm);
	if (!media_)
		return false;

	/* Create the V4L2 subdevices we will need. */
	dphy_ = V4L2Subdevice::fromEntityName(media_, "rockchip-sy-mipi-dphy");
	if (dphy_->open() < 0)
		return false;

	isp_ = V4L2Subdevice::fromEntityName(media_, "rkisp1-isp-subdev");
	if (isp_->open() < 0)
		return false;

	/* Locate and open the capture video node. */
	video_ = V4L2Device::fromEntityName(media_, "rkisp1_mainpath");
	if (video_->open() < 0)
		return false;

	video_->bufferReady.connect(this, &PipelineHandlerRkISP1::bufferReady);

	/* Configure default links. */
	if (initLinks() < 0) {
		LOG(RkISP1, Error) << "Failed to setup links";
		return false;
	}

	/*
	 * Enumerate all sensors connected to the CSI-2 receiver and create one
	 * camera instance for each of them.
	 */
	pad = dphy_->entity()->getPadByIndex(0);
	if (!pad)
		return false;

	for (MediaLink *link : pad->links())
		createCamera(link->source()->entity());

	return true;
}

/* -----------------------------------------------------------------------------
 * Buffer Handling
 */

void PipelineHandlerRkISP1::bufferReady(Buffer *buffer)
{
	ASSERT(activeCamera_);

	RkISP1CameraData *data = cameraData(activeCamera_);
	Request *request = data->queuedRequests_.front();

	completeBuffer(activeCamera_, request, buffer);
	completeRequest(activeCamera_, request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerRkISP1);

} /* namespace libcamera */
