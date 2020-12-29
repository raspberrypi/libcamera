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

/* -----------------------------------------------------------------------------
 *
 * Overview
 * --------
 *
 * The SimplePipelineHandler relies on generic kernel APIs to control a camera
 * device, without any device-specific code and with limited device-specific
 * static data.
 *
 * To qualify for support by the simple pipeline handler, a device shall
 *
 * - be supported by V4L2 drivers, exposing the Media Controller API, the V4L2
 *   subdev APIs and the media bus format-based enumeration extension for the
 *   VIDIOC_ENUM_FMT ioctl ;
 * - not expose any device-specific API from drivers to userspace ;
 * - include one or more camera sensor media entities and one or more video
 *   capture devices ;
 * - have a capture pipeline with linear paths from the camera sensors to the
 *   video capture devices ; and
 * - have an optional memory-to-memory device to perform format conversion
 *   and/or scaling, exposed as a V4L2 M2M device.
 *
 * As devices that require a specific pipeline handler may still match the
 * above characteristics, the simple pipeline handler doesn't attempt to
 * automatically determine which devices it can support. It instead relies on
 * an explicit list of supported devices, provided in the supportedDevices
 * array.
 *
 * When matching a device, the pipeline handler enumerates all camera sensors
 * and attempts, for each of them, to find a path to a video capture video node.
 * It does so by traversing the media graph, following the first non permanently
 * disabled downstream link. If such a path is found, the pipeline handler
 * creates a corresponding SimpleCameraData instance, and stores the media graph
 * path in its entities_ list.
 *
 * A more complex graph search algorithm could be implemented if a device that
 * would otherwise be compatible with the pipeline handler isn't correctly
 * handled by this heuristic.
 *
 * Once the camera data instances have been created, the match() function
 * creates a V4L2Subdevice instance for each entity used by any of the cameras
 * and stores the instances in SimplePipelineHandler::subdevs_, accessible by
 * the SimpleCameraData class through the SimplePipelineHandler::subdev()
 * function. This avoids duplication of subdev instances between different
 * cameras when the same entity is used in multiple paths. A similar mechanism
 * is used for V4L2VideoDevice instances, but instances are in this case created
 * on demand when accessed through SimplePipelineHandler::video() instead of all
 * in one go at initialization time.
 *
 * Finally, all camera data instances are initialized to gather information
 * about the possible pipeline configurations for the corresponding camera. If
 * valid pipeline configurations are found, a Camera is registered for the
 * SimpleCameraData instance.
 *
 * Pipeline Configuration
 * ----------------------
 *
 * The simple pipeline handler configures the pipeline by propagating V4L2
 * subdev formats from the camera sensor to the video node. The format is first
 * set on the camera sensor's output, using the native camera sensor
 * resolution. Then, on every link in the pipeline, the format is retrieved on
 * the link source and set unmodified on the link sink.
 *
 * When initializating the camera data, this above procedure is repeated for
 * every media bus format supported by the camera sensor. Upon reaching the
 * video node, the pixel formats compatible with the media bus format are
 * enumerated. Each of those pixel formats corresponds to one possible pipeline
 * configuration, stored as an instance of SimpleCameraData::Configuration in
 * the SimpleCameraData::formats_ map.
 *
 * Format Conversion and Scaling
 * -----------------------------
 *
 * The capture pipeline isn't expected to include a scaler, and if a scaler is
 * available, it is ignored when configuring the pipeline. However, the simple
 * pipeline handler supports optional memory-to-memory converters to scale the
 * image and convert it to a different pixel format. If such a converter is
 * present, the pipeline handler enumerates, for each pipeline configuration,
 * the pixel formats and sizes that the converter can produce for the output of
 * the capture video node, and stores the information in the outputFormats and
 * outputSizes of the SimpleCameraData::Configuration structure.
 */

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
		PixelFormat captureFormat;
		Size captureSize;
		std::vector<PixelFormat> outputFormats;
		SizeRange outputSizes;
	};

	std::vector<Stream> streams_;
	std::unique_ptr<CameraSensor> sensor_;
	std::list<Entity> entities_;
	V4L2VideoDevice *video_;

	std::vector<Configuration> configs_;
	std::map<PixelFormat, const Configuration *> formats_;

	std::vector<std::unique_ptr<FrameBuffer>> converterBuffers_;
	bool useConverter_;
	std::queue<FrameBuffer *> converterQueue_;
};

class SimpleCameraConfiguration : public CameraConfiguration
{
public:
	SimpleCameraConfiguration(Camera *camera, SimpleCameraData *data);

	Status validate() override;

	const SimpleCameraData::Configuration *pipeConfig() const
	{
		return pipeConfig_;
	}

	bool needConversion() const { return needConversion_; }

private:
	/*
	 * The SimpleCameraData instance is guaranteed to be valid as long as
	 * the corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	const SimpleCameraData *data_;

	const SimpleCameraData::Configuration *pipeConfig_;
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
	static constexpr unsigned int kNumInternalBuffers = 3;

	SimpleCameraData *cameraData(const Camera *camera)
	{
		return static_cast<SimpleCameraData *>(
			PipelineHandler::cameraData(camera));
	}

	void bufferReady(FrameBuffer *buffer);
	void converterInputDone(FrameBuffer *buffer);
	void converterOutputDone(FrameBuffer *buffer);

	MediaDevice *media_;
	std::map<const MediaEntity *, std::unique_ptr<V4L2VideoDevice>> videos_;
	std::map<const MediaEntity *, V4L2Subdevice> subdevs_;

	std::unique_ptr<SimpleConverter> converter_;

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

	streams_.resize(1);

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

		for (const auto &videoFormat : videoFormats) {
			PixelFormat pixelFormat = videoFormat.first.toPixelFormat();
			if (!pixelFormat)
				continue;

			Configuration config;
			config.code = code;
			config.captureFormat = pixelFormat;
			config.captureSize = format.size;

			if (!converter) {
				config.outputFormats = { pixelFormat };
				config.outputSizes = config.captureSize;
			} else {
				config.outputFormats = converter->formats(pixelFormat);
				config.outputSizes = converter->sizes(format.size);
			}

			configs_.push_back(config);
		}
	}

	if (configs_.empty()) {
		LOG(SimplePipeline, Error) << "No valid configuration found";
		return -EINVAL;
	}

	/*
	 * Map the pixel formats to configurations. Any previously stored value
	 * is overwritten, as the pipeline handler currently doesn't care about
	 * how a particular PixelFormat is achieved.
	 */
	for (const Configuration &config : configs_) {
		formats_[config.captureFormat] = &config;

		for (PixelFormat fmt : config.outputFormats)
			formats_[fmt] = &config;
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
	  data_(data), pipeConfig_(nullptr)
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
	if (config_.size() > data_->streams_.size()) {
		config_.resize(data_->streams_.size());
		status = Adjusted;
	}

	/*
	 * Pick a configuration for the pipeline based on the pixel format for
	 * the streams (ordered from highest to lowest priority). Default to
	 * the first pipeline configuration if no streams requests a supported
	 * pixel format.
	 */
	pipeConfig_ = data_->formats_.begin()->second;

	for (const StreamConfiguration &cfg : config_) {
		auto it = data_->formats_.find(cfg.pixelFormat);
		if (it != data_->formats_.end()) {
			pipeConfig_ = it->second;
			break;
		}
	}

	/* Adjust the requested streams. */
	SimplePipelineHandler *pipe = static_cast<SimplePipelineHandler *>(data_->pipe_);
	SimpleConverter *converter = pipe->converter();

	/*
	 * Enable usage of the converter when producing multiple streams, as
	 * the video capture device can't capture to multiple buffers.
	 *
	 * It is possible to produce up to one stream without conversion
	 * (provided the format and size match), at the expense of more complex
	 * buffer handling (including allocation of internal buffers to be used
	 * when a request doesn't contain a buffer for the stream that doesn't
	 * require any conversion, similar to raw capture use cases). This is
	 * left as a future improvement.
	 */
	needConversion_ = config_.size() > 1;

	for (unsigned int i = 0; i < config_.size(); ++i) {
		StreamConfiguration &cfg = config_[i];

		/* Adjust the pixel format and size. */
		auto it = std::find(pipeConfig_->outputFormats.begin(),
				    pipeConfig_->outputFormats.end(),
				    cfg.pixelFormat);
		if (it == pipeConfig_->outputFormats.end())
			it = pipeConfig_->outputFormats.begin();

		PixelFormat pixelFormat = *it;
		if (cfg.pixelFormat != pixelFormat) {
			LOG(SimplePipeline, Debug) << "Adjusting pixel format";
			cfg.pixelFormat = pixelFormat;
			status = Adjusted;
		}

		if (!pipeConfig_->outputSizes.contains(cfg.size)) {
			LOG(SimplePipeline, Debug)
				<< "Adjusting size from " << cfg.size.toString()
				<< " to " << pipeConfig_->captureSize.toString();
			cfg.size = pipeConfig_->captureSize;
			status = Adjusted;
		}

		/* \todo Create a libcamera core class to group format and size */
		if (cfg.pixelFormat != pipeConfig_->captureFormat ||
		    cfg.size != pipeConfig_->captureSize)
			needConversion_ = true;

		/* Set the stride, frameSize and bufferCount. */
		if (needConversion_) {
			std::tie(cfg.stride, cfg.frameSize) =
				converter->strideAndFrameSize(cfg.pixelFormat, cfg.size);
			if (cfg.stride == 0)
				return Invalid;
		} else {
			V4L2DeviceFormat format;
			format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
			format.size = cfg.size;

			int ret = data_->video_->tryFormat(&format);
			if (ret < 0)
				return Invalid;

			cfg.stride = format.planes[0].bpl;
			cfg.frameSize = format.planes[0].size;
		}

		cfg.bufferCount = 3;
	}

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
			       const Size &size = format.second->captureSize;
			       return { pixelFormat, { size } };
		       });

	/*
	 * Create the stream configurations. Take the first entry in the formats
	 * map as the default, for lack of a better option.
	 *
	 * \todo Implement a better way to pick the default format
	 */
	for ([[maybe_unused]] StreamRole role : roles) {
		StreamConfiguration cfg{ StreamFormats{ formats } };
		cfg.pixelFormat = formats.begin()->first;
		cfg.size = formats.begin()->second[0].max;

		config->addConfiguration(cfg);
	}

	config->validate();

	return config;
}

int SimplePipelineHandler::configure(Camera *camera, CameraConfiguration *c)
{
	SimpleCameraConfiguration *config =
		static_cast<SimpleCameraConfiguration *>(c);
	SimpleCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	int ret;

	/*
	 * Configure links on the pipeline and propagate formats from the
	 * sensor to the video node.
	 */
	ret = data->setupLinks();
	if (ret < 0)
		return ret;

	const SimpleCameraData::Configuration *pipeConfig = config->pipeConfig();
	V4L2SubdeviceFormat format{ pipeConfig->code, data->sensor_->resolution() };

	ret = data->setupFormats(&format, V4L2Subdevice::ActiveFormat);
	if (ret < 0)
		return ret;

	/* Configure the video node. */
	V4L2PixelFormat videoFormat = video->toV4L2PixelFormat(pipeConfig->captureFormat);

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = videoFormat;
	captureFormat.size = pipeConfig->captureSize;

	ret = video->setFormat(&captureFormat);
	if (ret)
		return ret;

	if (captureFormat.planesCount != 1) {
		LOG(SimplePipeline, Error)
			<< "Planar formats using non-contiguous memory not supported";
		return -EINVAL;
	}

	if (captureFormat.fourcc != videoFormat ||
	    captureFormat.size != pipeConfig->captureSize) {
		LOG(SimplePipeline, Error)
			<< "Unable to configure capture in "
			<< pipeConfig->captureSize.toString() << "-"
			<< videoFormat.toString();
		return -EINVAL;
	}

	/* Configure the converter if needed. */
	std::vector<std::reference_wrapper<StreamConfiguration>> outputCfgs;
	data->useConverter_ = config->needConversion();

	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = config->at(i);

		cfg.setStream(&data->streams_[i]);

		if (data->useConverter_)
			outputCfgs.push_back(cfg);
	}

	if (outputCfgs.empty())
		return 0;

	StreamConfiguration inputCfg;
	inputCfg.pixelFormat = pipeConfig->captureFormat;
	inputCfg.size = pipeConfig->captureSize;
	inputCfg.stride = captureFormat.planes[0].bpl;
	inputCfg.bufferCount = kNumInternalBuffers;

	return converter_->configure(inputCfg, outputCfgs);
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
	if (data->useConverter_)
		return converter_->exportBuffers(0, count, buffers);
	else
		return data->video_->exportBuffers(count, buffers);
}

int SimplePipelineHandler::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	SimpleCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	int ret;

	if (data->useConverter_) {
		/*
		 * When using the converter allocate a fixed number of internal
		 * buffers.
		 */
		ret = video->allocateBuffers(kNumInternalBuffers,
					     &data->converterBuffers_);
	} else {
		/* Otherwise, prepare for using buffers from the only stream. */
		Stream *stream = &data->streams_[0];
		ret = video->importBuffers(stream->configuration().bufferCount);
	}
	if (ret < 0)
		return ret;

	ret = video->streamOn();
	if (ret < 0) {
		stop(camera);
		return ret;
	}

	if (data->useConverter_) {
		ret = converter_->start();
		if (ret < 0) {
			stop(camera);
			return ret;
		}

		/* Queue all internal buffers for capture. */
		for (std::unique_ptr<FrameBuffer> &buffer : data->converterBuffers_)
			video->queueBuffer(buffer.get());
	}

	activeCamera_ = camera;

	return 0;
}

void SimplePipelineHandler::stop(Camera *camera)
{
	SimpleCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;

	if (data->useConverter_)
		converter_->stop();

	video->streamOff();
	video->releaseBuffers();

	data->converterBuffers_.clear();
	activeCamera_ = nullptr;
}

int SimplePipelineHandler::queueRequestDevice(Camera *camera, Request *request)
{
	SimpleCameraData *data = cameraData(camera);
	Stream *stream = &data->streams_[0];

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
	if (data->useConverter_) {
		data->converterQueue_.push(buffer);
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
		if (!converter_->isValid()) {
			LOG(SimplePipeline, Warning)
				<< "Failed to create converter, disabling format conversion";
			converter_.reset();
		} else {
			converter_->inputBufferReady.connect(this, &SimplePipelineHandler::converterInputDone);
			converter_->outputBufferReady.connect(this, &SimplePipelineHandler::converterOutputDone);
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

		std::set<Stream *> streams;
		std::transform(data->streams_.begin(), data->streams_.end(),
			       std::inserter(streams, streams.end()),
			       [](Stream &stream) { return &stream; });

		std::shared_ptr<Camera> camera =
			Camera::create(this, data->sensor_->id(), streams);
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
		if (data->useConverter_) {
			/* Requeue the buffer for capture. */
			data->video_->queueBuffer(buffer);

			/*
			 * Get the next user-facing buffer to complete the
			 * request.
			 */
			if (data->converterQueue_.empty())
				return;

			buffer = data->converterQueue_.front();
			data->converterQueue_.pop();
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
	if (data->useConverter_) {
		if (data->converterQueue_.empty()) {
			data->video_->queueBuffer(buffer);
			return;
		}

		FrameBuffer *output = data->converterQueue_.front();
		data->converterQueue_.pop();

		converter_->queueBuffers(buffer, { { 0, output } });
		return;
	}

	/* Otherwise simply complete the request. */
	Request *request = buffer->request();
	completeBuffer(request, buffer);
	completeRequest(request);
}

void SimplePipelineHandler::converterInputDone(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	SimpleCameraData *data = cameraData(activeCamera_);

	/* Queue the input buffer back for capture. */
	data->video_->queueBuffer(buffer);
}

void SimplePipelineHandler::converterOutputDone(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);

	/* Complete the request. */
	Request *request = buffer->request();
	completeBuffer(request, buffer);
	completeRequest(request);
}

REGISTER_PIPELINE_HANDLER(SimplePipelineHandler)

} /* namespace libcamera */
