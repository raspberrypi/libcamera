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
#include <queue>
#include <set>
#include <string>
#include <string.h>
#include <utility>
#include <vector>

#include <linux/media-bus-format.h>

#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "converter.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(SimplePipeline)

class SimplePipelineHandler;

struct SimplePipelineInfo {
	const char *driver;
	const char *converter;
};

namespace {

static const SimplePipelineInfo supportedDevices[] = {
	{ "imx7-csi", "pxp" },
	{ "qcom-camss", nullptr },
	{ "sun6i-csi", nullptr },
};

} /* namespace */

class SimpleCameraData : public CameraData
{
public:
	SimpleCameraData(SimplePipelineHandler *pipe, MediaEntity *sensor);

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
		Size captureSize;
		SizeRange outputSizes;
	};

	Stream stream_;
	std::unique_ptr<CameraSensor> sensor_;
	std::list<Entity> entities_;
	V4L2VideoDevice *video_;

	std::map<PixelFormat, Configuration> formats_;
};

class SimpleCameraConfiguration : public CameraConfiguration
{
public:
	SimpleCameraConfiguration(Camera *camera, SimpleCameraData *data);

	Status validate() override;

	const V4L2SubdeviceFormat &sensorFormat() { return sensorFormat_; }

	bool needConversion() const { return needConversion_; }

private:
	/*
	 * The SimpleCameraData instance is guaranteed to be valid as long as
	 * the corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	const SimpleCameraData *data_;

	V4L2SubdeviceFormat sensorFormat_;
	bool needConversion_;
};

class SimplePipelineHandler : public PipelineHandler
{
public:
	SimplePipelineHandler(CameraManager *manager);

	CameraConfiguration *generateConfiguration(Camera *camera,
						   const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stop(Camera *camera) override;

	bool match(DeviceEnumerator *enumerator) override;

	V4L2VideoDevice *video(const MediaEntity *entity);
	V4L2Subdevice *subdev(const MediaEntity *entity);
	SimpleConverter *converter() { return converter_.get(); }

protected:
	int queueRequestDevice(Camera *camera, Request *request) override;

private:
	SimpleCameraData *cameraData(const Camera *camera)
	{
		return static_cast<SimpleCameraData *>(
			PipelineHandler::cameraData(camera));
	}

	void bufferReady(FrameBuffer *buffer);
	void converterDone(FrameBuffer *input, FrameBuffer *output);

	MediaDevice *media_;
	std::map<const MediaEntity *, std::unique_ptr<V4L2VideoDevice>> videos_;
	std::map<const MediaEntity *, V4L2Subdevice> subdevs_;

	std::unique_ptr<SimpleConverter> converter_;
	bool useConverter_;
	std::vector<std::unique_ptr<FrameBuffer>> converterBuffers_;
	std::queue<FrameBuffer *> converterQueue_;

	Camera *activeCamera_;
};

/* -----------------------------------------------------------------------------
 * Camera Data
 */

SimpleCameraData::SimpleCameraData(SimplePipelineHandler *pipe,
				   MediaEntity *sensor)
	: CameraData(pipe)
{
	int ret;

	/*
	 * Walk the pipeline towards the video node and store all entities
	 * along the way.
	 */
	MediaEntity *source = sensor;

	while (source) {
		/* If we have reached a video node, we're done. */
		if (source->function() == MEDIA_ENT_F_IO_V4L)
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

	/*
	 * We have a valid pipeline, get the video device and create the camera
	 * sensor.
	 */
	video_ = pipe->video(source);
	if (!video_)
		return;

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
	SimpleConverter *converter = pipe->converter();
	int ret;

	/*
	 * Setup links first as some subdev drivers take active links into
	 * account to propagate TRY formats. Such is life :-(
	 */
	ret = setupLinks();
	if (ret < 0)
		return ret;

	/*
	 * Enumerate the possible pipeline configurations. For each media bus
	 * format supported by the sensor, propagate the formats through the
	 * pipeline, and enumerate the corresponding possible V4L2 pixel
	 * formats on the video node.
	 */
	for (unsigned int code : sensor_->mbusCodes()) {
		V4L2SubdeviceFormat format{ code, sensor_->resolution() };

		ret = setupFormats(&format, V4L2Subdevice::TryFormat);
		if (ret < 0) {
			LOG(SimplePipeline, Debug)
				<< "Media bus code " << utils::hex(code, 4)
				<< " not supported for this pipeline";
			/* Try next mbus_code supported by the sensor */
			continue;
		}

		V4L2VideoDevice::Formats videoFormats =
			video_->formats(format.mbus_code);

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
			config.captureSize = format.size;

			if (!converter) {
				config.outputSizes = config.captureSize;
				formats_[pixelFormat] = config;
				continue;
			}

			config.outputSizes = converter->sizes(format.size);

			for (PixelFormat fmt : converter->formats(pixelFormat))
				formats_[fmt] = config;
		}
	}

	if (formats_.empty()) {
		LOG(SimplePipeline, Error) << "No valid configuration found";
		return -EINVAL;
	}

	properties_ = sensor_->properties();

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
			V4L2SubdeviceFormat sourceFormat = *format;

			V4L2Subdevice *subdev = pipe->subdev(sink->entity());
			ret = subdev->setFormat(sink->index(), format, whence);
			if (ret < 0)
				return ret;

			if (format->mbus_code != sourceFormat.mbus_code ||
			    format->size != sourceFormat.size) {
				LOG(SimplePipeline, Debug)
					<< "Source '" << source->entity()->name()
					<< "':" << source->index()
					<< " produces " << sourceFormat.toString()
					<< ", sink '" << sink->entity()->name()
					<< "':" << sink->index()
					<< " requires " << format->toString();
				return -EINVAL;
			}
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

	if (transform != Transform::Identity) {
		transform = Transform::Identity;
		status = Adjusted;
	}

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
	if (!pipeConfig.outputSizes.contains(cfg.size)) {
		LOG(SimplePipeline, Debug)
			<< "Adjusting size from " << cfg.size.toString()
			<< " to " << pipeConfig.captureSize.toString();
		cfg.size = pipeConfig.captureSize;
		status = Adjusted;
	}

	needConversion_ = cfg.pixelFormat != pipeConfig.pixelFormat
			|| cfg.size != pipeConfig.captureSize;

	cfg.bufferCount = 3;

	/* Set the stride and frameSize. */
	if (!needConversion_) {
		V4L2DeviceFormat format;
		format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
		format.size = cfg.size;

		int ret = data_->video_->tryFormat(&format);
		if (ret < 0)
			return Invalid;

		cfg.stride = format.planes[0].bpl;
		cfg.frameSize = format.planes[0].size;

		return status;
	}

	SimplePipelineHandler *pipe = static_cast<SimplePipelineHandler *>(data_->pipe_);
	SimpleConverter *converter = pipe->converter();

	std::tie(cfg.stride, cfg.frameSize) =
		converter->strideAndFrameSize(cfg.pixelFormat, cfg.size);
	if (cfg.stride == 0)
		return Invalid;

	return status;
}

/* -----------------------------------------------------------------------------
 * Pipeline Handler
 */

SimplePipelineHandler::SimplePipelineHandler(CameraManager *manager)
	: PipelineHandler(manager)
{
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
			       const Size &size = format.second.captureSize;
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
	V4L2VideoDevice *video = data->video_;
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
	V4L2PixelFormat videoFormat = video->toV4L2PixelFormat(pipeConfig.pixelFormat);

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = videoFormat;
	captureFormat.size = pipeConfig.captureSize;

	ret = video->setFormat(&captureFormat);
	if (ret)
		return ret;

	if (captureFormat.planesCount != 1) {
		LOG(SimplePipeline, Error)
			<< "Planar formats using non-contiguous memory not supported";
		return -EINVAL;
	}

	if (captureFormat.fourcc != videoFormat ||
	    captureFormat.size != pipeConfig.captureSize) {
		LOG(SimplePipeline, Error)
			<< "Unable to configure capture in "
			<< pipeConfig.captureSize.toString() << "-"
			<< videoFormat.toString();
		return -EINVAL;
	}

	/* Configure the converter if required. */
	useConverter_ = config->needConversion();

	if (useConverter_) {
		StreamConfiguration inputCfg;
		inputCfg.pixelFormat = pipeConfig.pixelFormat;
		inputCfg.size = pipeConfig.captureSize;
		inputCfg.stride = captureFormat.planes[0].bpl;
		inputCfg.bufferCount = cfg.bufferCount;

		ret = converter_->configure(inputCfg, cfg);
		if (ret < 0) {
			LOG(SimplePipeline, Error)
				<< "Unable to configure converter";
			return ret;
		}

		LOG(SimplePipeline, Debug) << "Using format converter";
	}

	cfg.setStream(&data->stream_);

	return 0;
}

int SimplePipelineHandler::exportFrameBuffers(Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	SimpleCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	/*
	 * Export buffers on the converter or capture video node, depending on
	 * whether the converter is used or not.
	 */
	if (useConverter_)
		return converter_->exportBuffers(count, buffers);
	else
		return data->video_->exportBuffers(count, buffers);
}

int SimplePipelineHandler::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	SimpleCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	unsigned int count = data->stream_.configuration().bufferCount;
	int ret;

	if (useConverter_)
		ret = video->allocateBuffers(count, &converterBuffers_);
	else
		ret = video->importBuffers(count);
	if (ret < 0)
		return ret;

	ret = video->streamOn();
	if (ret < 0) {
		stop(camera);
		return ret;
	}

	if (useConverter_) {
		ret = converter_->start();
		if (ret < 0) {
			stop(camera);
			return ret;
		}

		/* Queue all internal buffers for capture. */
		for (std::unique_ptr<FrameBuffer> &buffer : converterBuffers_)
			video->queueBuffer(buffer.get());
	}

	activeCamera_ = camera;

	return 0;
}

void SimplePipelineHandler::stop(Camera *camera)
{
	SimpleCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;

	if (useConverter_)
		converter_->stop();

	video->streamOff();
	video->releaseBuffers();

	converterBuffers_.clear();
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

	/*
	 * If conversion is needed, push the buffer to the converter queue, it
	 * will be handed to the converter in the capture completion handler.
	 */
	if (useConverter_) {
		converterQueue_.push(buffer);
		return 0;
	}

	return data->video_->queueBuffer(buffer);
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

bool SimplePipelineHandler::match(DeviceEnumerator *enumerator)
{
	MediaDevice *converter = nullptr;

	for (const SimplePipelineInfo &info : supportedDevices) {
		DeviceMatch dm(info.driver);
		media_ = acquireMediaDevice(enumerator, dm);
		if (!media_)
			continue;

		if (!info.converter)
			break;

		DeviceMatch converterMatch(info.converter);
		converter = acquireMediaDevice(enumerator, converterMatch);
		break;
	}

	if (!media_)
		return false;

	/* Locate the sensors. */
	std::vector<MediaEntity *> sensors;

	for (MediaEntity *entity : media_->entities()) {
		switch (entity->function()) {
		case MEDIA_ENT_F_CAM_SENSOR:
			sensors.push_back(entity);
			break;

		default:
			break;
		}
	}

	if (sensors.empty()) {
		LOG(SimplePipeline, Error) << "No sensor found";
		return false;
	}

	/* Open the converter, if any. */
	if (converter) {
		converter_ = std::make_unique<SimpleConverter>(converter);
		if (converter_->open() < 0) {
			LOG(SimplePipeline, Warning)
				<< "Failed to open converter, disabling format conversion";
			converter_.reset();
		} else {
			converter_->bufferReady.connect(this, &SimplePipelineHandler::converterDone);
		}
	}

	/*
	 * Create one camera data instance for each sensor and gather all
	 * entities in all pipelines.
	 */
	std::vector<std::unique_ptr<SimpleCameraData>> pipelines;
	std::set<MediaEntity *> entities;

	pipelines.reserve(sensors.size());

	for (MediaEntity *sensor : sensors) {
		std::unique_ptr<SimpleCameraData> data =
			std::make_unique<SimpleCameraData>(this, sensor);
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
	bool registered = false;

	for (std::unique_ptr<SimpleCameraData> &data : pipelines) {
		int ret = data->init();
		if (ret < 0)
			continue;

		std::shared_ptr<Camera> camera =
			Camera::create(this, data->sensor_->id(),
				       data->streams());
		registerCamera(std::move(camera), std::move(data));
		registered = true;
	}

	return registered;
}

V4L2VideoDevice *SimplePipelineHandler::video(const MediaEntity *entity)
{
	/*
	 * Return the V4L2VideoDevice corresponding to the media entity, either
	 * as a previously constructed device if available from the cache, or
	 * by constructing a new one.
	 */

	auto iter = videos_.find(entity);
	if (iter != videos_.end())
		return iter->second.get();

	std::unique_ptr<V4L2VideoDevice> video =
		std::make_unique<V4L2VideoDevice>(entity);
	if (video->open() < 0)
		return nullptr;

	video->bufferReady.connect(this, &SimplePipelineHandler::bufferReady);

	auto element = videos_.emplace(entity, std::move(video));
	return element.first->second.get();
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
	SimpleCameraData *data = cameraData(activeCamera_);

	/*
	 * If an error occurred during capture, or if the buffer was cancelled,
	 * complete the request, even if the converter is in use as there's no
	 * point converting an erroneous buffer.
	 */
	if (buffer->metadata().status != FrameMetadata::FrameSuccess) {
		if (useConverter_) {
			/* Requeue the buffer for capture. */
			data->video_->queueBuffer(buffer);

			/*
			 * Get the next user-facing buffer to complete the
			 * request.
			 */
			if (converterQueue_.empty())
				return;

			buffer = converterQueue_.front();
			converterQueue_.pop();
		}

		Request *request = buffer->request();
		completeBuffer(request, buffer);
		completeRequest(request);
		return;
	}

	/*
	 * Queue the captured and the request buffer to the converter if format
	 * conversion is needed. If there's no queued request, just requeue the
	 * captured buffer for capture.
	 */
	if (useConverter_) {
		if (converterQueue_.empty()) {
			data->video_->queueBuffer(buffer);
			return;
		}

		FrameBuffer *output = converterQueue_.front();
		converterQueue_.pop();

		converter_->queueBuffers(buffer, output);
		return;
	}

	/* Otherwise simply complete the request. */
	Request *request = buffer->request();
	completeBuffer(request, buffer);
	completeRequest(request);
}

void SimplePipelineHandler::converterDone(FrameBuffer *input,
					  FrameBuffer *output)
{
	ASSERT(activeCamera_);
	SimpleCameraData *data = cameraData(activeCamera_);

	/* Complete the request. */
	Request *request = output->request();
	completeBuffer(request, output);
	completeRequest(request);

	/* Queue the input buffer back for capture. */
	data->video_->queueBuffer(input);
}

REGISTER_PIPELINE_HANDLER(SimplePipelineHandler)

} /* namespace libcamera */
