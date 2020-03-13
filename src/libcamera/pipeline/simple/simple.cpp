/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright (C) 2019, Martijn Braam
 *
 * simple.cpp - Pipeline handler for simple pipelines
 */

#include <algorithm>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string.h>
#include <utility>
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
#include "v4l2_subdevice.h"
#include "v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(SimplePipeline)

class SimplePipelineHandler;

namespace {

static const char * const drivers[] = {
	"imx7-csi",
	"sun6i-csi",
};

} /* namespace */

class SimpleCameraData : public CameraData
{
public:
	SimpleCameraData(PipelineHandler *pipe, MediaEntity *sensor,
			 MediaEntity *video);

	bool isValid() const { return sensor_ != nullptr; }
	std::set<Stream *> streams() { return { &stream_ }; }

	int init();
	int setupLinks();
	int setupFormats(V4L2SubdeviceFormat *format,
			 V4L2Subdevice::Whence whence);

	struct Entity {
		MediaEntity *entity;
		MediaLink *link;
	};

	struct Configuration {
		uint32_t code;
		PixelFormat pixelFormat;
		Size size;
	};

	Stream stream_;
	std::unique_ptr<CameraSensor> sensor_;
	std::list<Entity> entities_;

	std::vector<Configuration> configs_;
	std::map<PixelFormat, Configuration> formats_;
};

class SimpleCameraConfiguration : public CameraConfiguration
{
public:
	SimpleCameraConfiguration(Camera *camera, SimpleCameraData *data);

	Status validate() override;

	const V4L2SubdeviceFormat &sensorFormat() { return sensorFormat_; }

private:
	/*
	 * The SimpleCameraData instance is guaranteed to be valid as long as
	 * the corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	const SimpleCameraData *data_;

	V4L2SubdeviceFormat sensorFormat_;
};

class SimplePipelineHandler : public PipelineHandler
{
public:
	SimplePipelineHandler(CameraManager *manager);
	~SimplePipelineHandler();

	CameraConfiguration *generateConfiguration(Camera *camera,
						   const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	bool match(DeviceEnumerator *enumerator) override;

	V4L2VideoDevice *video() { return video_; }
	V4L2Subdevice *subdev(const MediaEntity *entity);

protected:
	int queueRequestDevice(Camera *camera, Request *request) override;

private:
	SimpleCameraData *cameraData(const Camera *camera)
	{
		return static_cast<SimpleCameraData *>(
			PipelineHandler::cameraData(camera));
	}

	int initLinks();

	int createCamera(MediaEntity *sensor);

	void bufferReady(FrameBuffer *buffer);

	MediaDevice *media_;
	V4L2VideoDevice *video_;
	std::map<const MediaEntity *, V4L2Subdevice> subdevs_;

	Camera *activeCamera_;
};

/* -----------------------------------------------------------------------------
 * Camera Data
 */

SimpleCameraData::SimpleCameraData(PipelineHandler *pipe, MediaEntity *sensor,
				   MediaEntity *video)
	: CameraData(pipe)
{
	int ret;

	/*
	 * Walk the pipeline towards the video node and store all entities
	 * along the way.
	 */
	MediaEntity *source = sensor;

	while (source) {
		/* If we have reached the video node, we're done. */
		if (source == video)
			break;

		/* Use the first output pad that has links. */
		MediaPad *sourcePad = nullptr;
		for (MediaPad *pad : source->pads()) {
			if ((pad->flags() & MEDIA_PAD_FL_SOURCE) &&
			    !pad->links().empty()) {
				sourcePad = pad;
				break;
			}
		}

		if (!sourcePad)
			return;

		/*
		 * Use the first link that is enabled or can be enabled (not
		 * immutable).
		 */
		MediaLink *sourceLink = nullptr;
		for (MediaLink *link : sourcePad->links()) {
			if ((link->flags() & MEDIA_LNK_FL_ENABLED) ||
			    !(link->flags() & MEDIA_LNK_FL_IMMUTABLE)) {
				sourceLink = link;
				break;
			}
		}

		if (!sourceLink)
			return;

		entities_.push_back({ source, sourceLink });

		source = sourceLink->sink()->entity();

		/* Avoid infinite loops. */
		auto iter = std::find_if(entities_.begin(), entities_.end(),
					 [&](const Entity &entity) {
						 return entity.entity == source;
					 });
		if (iter != entities_.end()) {
			LOG(SimplePipeline, Info) << "Loop detected in pipeline";
			return;
		}
	}

	/* We have a valid pipeline, create the camera sensor. */
	sensor_ = std::make_unique<CameraSensor>(sensor);
	ret = sensor_->init();
	if (ret) {
		sensor_.reset();
		return;
	}
}

int SimpleCameraData::init()
{
	SimplePipelineHandler *pipe = static_cast<SimplePipelineHandler *>(pipe_);
	V4L2VideoDevice *video = pipe->video();
	int ret;

	/*
	 * Enumerate the possible pipeline configurations. For each media bus
	 * format supported by the sensor, propagate the formats through the
	 * pipeline, and enumerate the corresponding possible V4L2 pixel
	 * formats on the video node.
	 */
	for (unsigned int code : sensor_->mbusCodes()) {
		V4L2SubdeviceFormat format{ code, sensor_->resolution() };

		/*
		 * Setup links first as some subdev drivers take active links
		 * into account to propagate TRY formats. Such is life :-(
		 */
		ret = setupLinks();
		if (ret < 0)
			return ret;

		ret = setupFormats(&format, V4L2Subdevice::TryFormat);
		if (ret < 0) {
			LOG(SimplePipeline, Error)
				<< "Failed to setup pipeline for media bus code "
				<< utils::hex(code, 4);
			return ret;
		}

		std::map<V4L2PixelFormat, std::vector<SizeRange>> videoFormats =
			video->formats(format.mbus_code);

		LOG(SimplePipeline, Debug)
			<< "Adding configuration for " << format.size.toString()
			<< " in pixel formats [ "
			<< utils::join(videoFormats, ", ",
				       [](const auto &f) {
					       return f.first.toString();
				       })
			<< " ]";

		/*
		 * Store the configuration in the formats_ map, mapping the
		 * PixelFormat to the corresponding configuration. Any
		 * previously stored value is overwritten, as the pipeline
		 * handler currently doesn't care about how a particular
		 * PixelFormat is achieved.
		 */
		for (const auto &videoFormat : videoFormats) {
			PixelFormat pixelFormat = videoFormat.first.toPixelFormat();
			if (!pixelFormat)
				continue;

			Configuration config;
			config.code = code;
			config.pixelFormat = pixelFormat;
			config.size = format.size;

			formats_[pixelFormat] = config;
		}
	}

	if (formats_.empty()) {
		LOG(SimplePipeline, Error) << "No valid configuration found";
		return -EINVAL;
	}

	return 0;
}

int SimpleCameraData::setupLinks()
{
	int ret;

	/*
	 * Configure all links along the pipeline. Some entities may not allow
	 * multiple sink links to be enabled together, even on different sink
	 * pads. We must thus start by disabling all sink links (but the one we
	 * want to enable) before enabling the pipeline link.
	 */
	for (SimpleCameraData::Entity &e : entities_) {
		MediaEntity *remote = e.link->sink()->entity();
		for (MediaPad *pad : remote->pads()) {
			for (MediaLink *link : pad->links()) {
				if (link == e.link)
					continue;

				if ((link->flags() & MEDIA_LNK_FL_ENABLED) &&
				    !(link->flags() & MEDIA_LNK_FL_IMMUTABLE)) {
					ret = link->setEnabled(false);
					if (ret < 0)
						return ret;
				}
			}
		}

		if (!(e.link->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = e.link->setEnabled(true);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

int SimpleCameraData::setupFormats(V4L2SubdeviceFormat *format,
				   V4L2Subdevice::Whence whence)
{
	SimplePipelineHandler *pipe = static_cast<SimplePipelineHandler *>(pipe_);
	int ret;

	/*
	 * Configure the format on the sensor output and propagate it through
	 * the pipeline.
	 */
	ret = sensor_->setFormat(format);
	if (ret < 0)
		return ret;

	for (const Entity &e : entities_) {
		MediaLink *link = e.link;
		MediaPad *source = link->source();
		MediaPad *sink = link->sink();

		if (source->entity() != sensor_->entity()) {
			V4L2Subdevice *subdev = pipe->subdev(source->entity());
			ret = subdev->getFormat(source->index(), format, whence);
			if (ret < 0)
				return ret;
		}

		if (sink->entity()->function() != MEDIA_ENT_F_IO_V4L) {
			V4L2Subdevice *subdev = pipe->subdev(sink->entity());
			ret = subdev->setFormat(sink->index(), format, whence);
			if (ret < 0)
				return ret;
		}

		LOG(SimplePipeline, Debug)
			<< "Link '" << source->entity()->name()
			<< "':" << source->index()
			<< " -> '" << sink->entity()->name()
			<< "':" << sink->index()
			<< " configured with format " << format->toString();
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Camera Configuration
 */

SimpleCameraConfiguration::SimpleCameraConfiguration(Camera *camera,
						     SimpleCameraData *data)
	: CameraConfiguration(), camera_(camera->shared_from_this()),
	  data_(data)
{
}

CameraConfiguration::Status SimpleCameraConfiguration::validate()
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
	auto it = data_->formats_.find(cfg.pixelFormat);
	if (it == data_->formats_.end())
		it = data_->formats_.begin();

	PixelFormat pixelFormat = it->first;
	if (cfg.pixelFormat != pixelFormat) {
		LOG(SimplePipeline, Debug) << "Adjusting pixel format";
		cfg.pixelFormat = pixelFormat;
		status = Adjusted;
	}

	const SimpleCameraData::Configuration &pipeConfig = it->second;
	if (cfg.size != pipeConfig.size) {
		LOG(SimplePipeline, Debug)
			<< "Adjusting size from " << cfg.size.toString()
			<< " to " << pipeConfig.size.toString();
		cfg.size = pipeConfig.size;
		status = Adjusted;
	}

	cfg.bufferCount = 3;

	return status;
}

/* -----------------------------------------------------------------------------
 * Pipeline Handler
 */

SimplePipelineHandler::SimplePipelineHandler(CameraManager *manager)
	: PipelineHandler(manager), video_(nullptr)
{
}

SimplePipelineHandler::~SimplePipelineHandler()
{
	delete video_;
}

CameraConfiguration *SimplePipelineHandler::generateConfiguration(Camera *camera,
								  const StreamRoles &roles)
{
	SimpleCameraData *data = cameraData(camera);
	CameraConfiguration *config =
		new SimpleCameraConfiguration(camera, data);

	if (roles.empty())
		return config;

	/* Create the formats map. */
	std::map<PixelFormat, std::vector<SizeRange>> formats;
	std::transform(data->formats_.begin(), data->formats_.end(),
		       std::inserter(formats, formats.end()),
		       [](const auto &format) -> decltype(formats)::value_type {
			       const PixelFormat &pixelFormat = format.first;
			       const Size &size = format.second.size;
			       return { pixelFormat, { size } };
		       });

	/*
	 * Create the stream configuration. Take the first entry in the formats
	 * map as the default, for lack of a better option.
	 *
	 * \todo Implement a better way to pick the default format
	 */
	StreamConfiguration cfg{ StreamFormats{ formats } };
	cfg.pixelFormat = formats.begin()->first;
	cfg.size = formats.begin()->second[0].max;

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int SimplePipelineHandler::configure(Camera *camera, CameraConfiguration *c)
{
	SimpleCameraConfiguration *config =
		static_cast<SimpleCameraConfiguration *>(c);
	SimpleCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	int ret;

	/*
	 * Configure links on the pipeline and propagate formats from the
	 * sensor to the video node.
	 */
	ret = data->setupLinks();
	if (ret < 0)
		return ret;

	const SimpleCameraData::Configuration &pipeConfig =
		data->formats_[cfg.pixelFormat];

	V4L2SubdeviceFormat format{ pipeConfig.code, data->sensor_->resolution() };

	ret = data->setupFormats(&format, V4L2Subdevice::ActiveFormat);
	if (ret < 0)
		return ret;

	/* Configure the video node. */
	V4L2PixelFormat videoFormat = video_->toV4L2PixelFormat(cfg.pixelFormat);

	V4L2DeviceFormat outputFormat = {};
	outputFormat.fourcc = videoFormat;
	outputFormat.size = cfg.size;

	ret = video_->setFormat(&outputFormat);
	if (ret)
		return ret;

	if (outputFormat.size != cfg.size || outputFormat.fourcc != videoFormat) {
		LOG(SimplePipeline, Error)
			<< "Unable to configure capture in " << cfg.toString();
		return -EINVAL;
	}

	cfg.setStream(&data->stream_);

	return 0;
}

int SimplePipelineHandler::exportFrameBuffers(Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	unsigned int count = stream->configuration().bufferCount;

	return video_->exportBuffers(count, buffers);
}

int SimplePipelineHandler::start(Camera *camera)
{
	SimpleCameraData *data = cameraData(camera);
	unsigned int count = data->stream_.configuration().bufferCount;

	int ret = video_->importBuffers(count);
	if (ret < 0)
		return ret;

	ret = video_->streamOn();
	if (ret < 0) {
		video_->releaseBuffers();
		return ret;
	}

	activeCamera_ = camera;

	return 0;
}

void SimplePipelineHandler::stop(Camera *camera)
{
	video_->streamOff();
	video_->releaseBuffers();
	activeCamera_ = nullptr;
}

int SimplePipelineHandler::queueRequestDevice(Camera *camera, Request *request)
{
	SimpleCameraData *data = cameraData(camera);
	Stream *stream = &data->stream_;

	FrameBuffer *buffer = request->findBuffer(stream);
	if (!buffer) {
		LOG(SimplePipeline, Error)
			<< "Attempt to queue request with invalid stream";
		return -ENOENT;
	}

	return video_->queueBuffer(buffer);
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

bool SimplePipelineHandler::match(DeviceEnumerator *enumerator)
{
	for (const char *driver : drivers) {
		DeviceMatch dm(driver);
		media_ = acquireMediaDevice(enumerator, dm);
		if (media_)
			break;
	}

	if (!media_)
		return false;

	/*
	 * Locate sensors and video nodes. We only support pipelines with at
	 * least one sensor and exactly one video capture node.
	 */
	std::vector<MediaEntity *> sensors;
	std::vector<MediaEntity *> videos;

	for (MediaEntity *entity : media_->entities()) {
		switch (entity->function()) {
		case MEDIA_ENT_F_CAM_SENSOR:
			sensors.push_back(entity);
			break;

		case MEDIA_ENT_F_IO_V4L:
			if (entity->pads().size() == 1 &&
			    (entity->pads()[0]->flags() & MEDIA_PAD_FL_SINK))
				videos.push_back(entity);
			break;

		default:
			break;
		}
	}

	if (sensors.empty()) {
		LOG(SimplePipeline, Error) << "No sensor found";
		return false;
	}

	if (videos.size() != 1) {
		LOG(SimplePipeline, Error)
			<< "Pipeline with " << videos.size()
			<< " video capture nodes is not supported";
		return false;
	}

	/* Locate and open the capture video node. */
	video_ = new V4L2VideoDevice(videos[0]);
	if (video_->open() < 0)
		return false;

	if (video_->caps().isMultiplanar()) {
		LOG(SimplePipeline, Error)
			<< "V4L2 multiplanar devices are not supported";
		return false;
	}

	video_->bufferReady.connect(this, &SimplePipelineHandler::bufferReady);

	/*
	 * Create one camera data instance for each sensor and gather all
	 * entities in all pipelines.
	 */
	std::vector<std::unique_ptr<SimpleCameraData>> pipelines;
	std::set<MediaEntity *> entities;

	pipelines.reserve(sensors.size());

	for (MediaEntity *sensor : sensors) {
		std::unique_ptr<SimpleCameraData> data =
			std::make_unique<SimpleCameraData>(this, sensor,
							   videos[0]);
		if (!data->isValid()) {
			LOG(SimplePipeline, Error)
				<< "No valid pipeline for sensor '"
				<< sensor->name() << "', skipping";
			continue;
		}

		for (SimpleCameraData::Entity &entity : data->entities_)
			entities.insert(entity.entity);

		pipelines.push_back(std::move(data));
	}

	if (entities.empty())
		return false;

	/* Create and open V4L2Subdev instances for all the entities. */
	for (MediaEntity *entity : entities) {
		auto elem = subdevs_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(entity),
					     std::forward_as_tuple(entity));
		V4L2Subdevice *subdev = &elem.first->second;
		int ret = subdev->open();
		if (ret < 0) {
			LOG(SimplePipeline, Error)
				<< "Failed to open " << subdev->deviceNode()
				<< ": " << strerror(-ret);
			return false;
		}
	}

	/* Initialize each pipeline and register a corresponding camera. */
	for (std::unique_ptr<SimpleCameraData> &data : pipelines) {
		int ret = data->init();
		if (ret < 0)
			continue;

		std::shared_ptr<Camera> camera =
			Camera::create(this, data->sensor_->entity()->name(),
				       data->streams());
		registerCamera(std::move(camera), std::move(data));
	}

	return true;
}

V4L2Subdevice *SimplePipelineHandler::subdev(const MediaEntity *entity)
{
	auto iter = subdevs_.find(entity);
	if (iter == subdevs_.end())
		return nullptr;

	return &iter->second;
}

/* -----------------------------------------------------------------------------
 * Buffer Handling
 */

void SimplePipelineHandler::bufferReady(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	Request *request = buffer->request();
	completeBuffer(activeCamera_, request, buffer);
	completeRequest(activeCamera_, request);
}

REGISTER_PIPELINE_HANDLER(SimplePipelineHandler);

} /* namespace libcamera */
