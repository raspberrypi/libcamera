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
#include <unordered_map>
#include <utility>
#include <vector>

#include <linux/media-bus-format.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
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
 * It does so by using a breadth-first search to find the shortest path from the
 * sensor device to a valid capture device. This is guaranteed to produce a
 * valid path on devices with one only option and is a good heuristic on more
 * complex devices to skip paths that aren't suitable for the simple pipeline
 * handler. For instance, on the IPU-based i.MX6, the shortest path will skip
 * encoders and image converters, and it will end in a CSI capture device.
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
	/*
	 * Each converter in the list contains the name
	 * and the number of streams it supports.
	 */
	std::vector<std::pair<const char *, unsigned int>> converters;
};

namespace {

static const SimplePipelineInfo supportedDevices[] = {
	{ "imx7-csi", { { "pxp", 1 } } },
	{ "qcom-camss", {} },
	{ "sun6i-csi", {} },
};

} /* namespace */

class SimpleCameraData : public CameraData
{
public:
	SimpleCameraData(SimplePipelineHandler *pipe,
			 unsigned int numStreams,
			 MediaEntity *sensor);

	bool isValid() const { return sensor_ != nullptr; }

	int init();
	int setupLinks();
	int setupFormats(V4L2SubdeviceFormat *format,
			 V4L2Subdevice::Whence whence);

	unsigned int streamIndex(const Stream *stream) const
	{
		return stream - &streams_.front();
	}

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
	std::queue<std::map<unsigned int, FrameBuffer *>> converterQueue_;
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

	std::vector<MediaEntity *> locateSensors();

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
				   unsigned int numStreams,
				   MediaEntity *sensor)
	: CameraData(pipe), streams_(numStreams)
{
	int ret;

	/*
	 * Find the shortest path from the camera sensor to a video capture
	 * device using the breadth-first search algorithm. This heuristic will
	 * be most likely to skip paths that aren't suitable for the simple
	 * pipeline handler on more complex devices, and is guaranteed to
	 * produce a valid path on all devices that have a single option.
	 *
	 * For instance, on the IPU-based i.MX6Q, the shortest path will skip
	 * encoders and image converters, and will end in a CSI capture device.
	 */
	std::unordered_set<MediaEntity *> visited;
	std::queue<MediaEntity *> queue;

	/* Remember at each entity where we came from. */
	std::unordered_map<MediaEntity *, Entity> parents;
	MediaEntity *entity = nullptr;

	queue.push(sensor);

	while (!queue.empty()) {
		entity = queue.front();
		queue.pop();

		/* Found the capture device. */
		if (entity->function() == MEDIA_ENT_F_IO_V4L) {
			LOG(SimplePipeline, Debug)
				<< "Found capture device " << entity->name();
			video_ = pipe->video(entity);
			break;
		}

		/* The actual breadth-first search algorithm. */
		visited.insert(entity);
		for (MediaPad *pad : entity->pads()) {
			if (!(pad->flags() & MEDIA_PAD_FL_SOURCE))
				continue;

			for (MediaLink *link : pad->links()) {
				MediaEntity *next = link->sink()->entity();
				if (visited.find(next) == visited.end()) {
					queue.push(next);
					parents.insert({ next, { entity, link } });
				}
			}
		}
	}

	if (!video_)
		return;

	/*
	 * With the parents, we can follow back our way from the capture device
	 * to the sensor.
	 */
	for (auto it = parents.find(entity); it != parents.end();
	     it = parents.find(entity)) {
		const Entity &e = it->second;
		entities_.push_front(e);
		entity = e.entity;
	}

	/* Finally also remember the sensor. */
	sensor_ = std::make_unique<CameraSensor>(sensor);
	ret = sensor_->init();
	if (ret)
		sensor_.reset();
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
		return converter_->exportBuffers(data->streamIndex(stream),
						 count, buffers);
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
	int ret;

	std::map<unsigned int, FrameBuffer *> buffers;

	for (auto &[stream, buffer] : request->buffers()) {
		/*
		 * If conversion is needed, push the buffer to the converter
		 * queue, it will be handed to the converter in the capture
		 * completion handler.
		 */
		if (data->useConverter_) {
			buffers.emplace(data->streamIndex(stream), buffer);
		} else {
			ret = data->video_->queueBuffer(buffer);
			if (ret < 0)
				return ret;
		}
	}

	if (data->useConverter_)
		data->converterQueue_.push(std::move(buffers));

	return 0;
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

std::vector<MediaEntity *> SimplePipelineHandler::locateSensors()
{
	std::vector<MediaEntity *> entities;

	/*
	 * Gather all the camera sensor entities based on the function they
	 * expose.
	 */
	for (MediaEntity *entity : media_->entities()) {
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR)
			entities.push_back(entity);
	}

	if (entities.empty())
		return {};

	/*
	 * Sensors can be made of multiple entities. For instance, a raw sensor
	 * can be connected to an ISP, and the combination of both should be
	 * treated as one sensor. To support this, as a crude heuristic, check
	 * the downstream entity from the camera sensor, and if it is an ISP,
	 * use it instead of the sensor.
	 */
	std::vector<MediaEntity *> sensors;

	for (MediaEntity *entity : entities) {
		/*
		 * Locate the downstream entity by following the first link
		 * from a source pad.
		 */
		const MediaLink *link = nullptr;

		for (const MediaPad *pad : entity->pads()) {
			if ((pad->flags() & MEDIA_PAD_FL_SOURCE) &&
			    !pad->links().empty()) {
				link = pad->links()[0];
				break;
			}
		}

		if (!link)
			continue;

		MediaEntity *remote = link->sink()->entity();
		if (remote->function() == MEDIA_ENT_F_PROC_VIDEO_ISP)
			sensors.push_back(remote);
		else
			sensors.push_back(entity);
	}

	/*
	 * Remove duplicates, in case multiple sensors are connected to the
	 * same ISP.
	 */
	std::sort(sensors.begin(), sensors.end());
	auto last = std::unique(sensors.begin(), sensors.end());
	sensors.erase(last, sensors.end());

	return sensors;
}

bool SimplePipelineHandler::match(DeviceEnumerator *enumerator)
{
	const SimplePipelineInfo *info = nullptr;
	MediaDevice *converter = nullptr;
	unsigned int numStreams = 1;

	for (const SimplePipelineInfo &inf : supportedDevices) {
		DeviceMatch dm(inf.driver);
		media_ = acquireMediaDevice(enumerator, dm);
		if (media_) {
			info = &inf;
			break;
		}
	}

	if (!media_)
		return false;

	for (const auto &[name, streams] : info->converters) {
		DeviceMatch converterMatch(name);
		converter = acquireMediaDevice(enumerator, converterMatch);
		if (converter) {
			numStreams = streams;
			break;
		}
	}

	/* Locate the sensors. */
	std::vector<MediaEntity *> sensors = locateSensors();
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
			std::make_unique<SimpleCameraData>(this, numStreams, sensor);
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
		if (!data->useConverter_) {
			/* No conversion, just complete the request. */
			Request *request = buffer->request();
			completeBuffer(request, buffer);
			completeRequest(request);
			return;
		}

		/*
		 * The converter is in use. Requeue the internal buffer for
		 * capture (unless the stream is being stopped), and complete
		 * the request with all the user-facing buffers.
		 */
		if (buffer->metadata().status != FrameMetadata::FrameCancelled)
			data->video_->queueBuffer(buffer);

		if (data->converterQueue_.empty())
			return;

		Request *request = nullptr;
		for (auto &item : data->converterQueue_.front()) {
			FrameBuffer *outputBuffer = item.second;
			request = outputBuffer->request();
			completeBuffer(request, outputBuffer);
		}
		data->converterQueue_.pop();

		if (request)
			completeRequest(request);
		return;
	}

	/*
	 * Record the sensor's timestamp in the request metadata.
	 *
	 * \todo The sensor timestamp should be better estimated by connecting
	 * to the V4L2Device::frameStart signal if the platform provides it.
	 */
	Request *request = buffer->request();
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

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

		converter_->queueBuffers(buffer, data->converterQueue_.front());
		data->converterQueue_.pop();
		return;
	}

	/* Otherwise simply complete the request. */
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

	/* Complete the buffer and the request. */
	Request *request = buffer->request();
	if (completeBuffer(request, buffer))
		completeRequest(request);
}

REGISTER_PIPELINE_HANDLER(SimplePipelineHandler)

} /* namespace libcamera */
