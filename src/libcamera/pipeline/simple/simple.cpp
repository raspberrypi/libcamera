/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright (C) 2019, Martijn Braam
 *
 * Pipeline handler for simple pipelines
 */

#include <algorithm>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <stdint.h>
#include <string.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <linux/media-bus-format.h>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/converter.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/software_isp/software_isp.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

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
 * creates a V4L2VideoDevice or V4L2Subdevice instance for each entity used by
 * any of the cameras and stores them in SimplePipelineHandler::entities_,
 * accessible by the SimpleCameraData class through the
 * SimplePipelineHandler::subdev() and SimplePipelineHandler::video() functions.
 * This avoids duplication of subdev instances between different cameras when
 * the same entity is used in multiple paths.
 *
 * Finally, all camera data instances are initialized to gather information
 * about the possible pipeline configurations for the corresponding camera. If
 * valid pipeline configurations are found, a Camera is registered for the
 * SimpleCameraData instance.
 *
 * Pipeline Traversal
 * ------------------
 *
 * During the breadth-first search, the pipeline is traversed from entity to
 * entity, by following media graph links from source to sink, starting at the
 * camera sensor.
 *
 * When reaching an entity (on its sink side), if the entity is a V4L2 subdev
 * that supports the streams API, the subdev internal routes are followed to
 * find the connected source pads. Otherwise all of the entity's source pads
 * are considered to continue the graph traversal. The pipeline handler
 * currently considers the default internal routes only and doesn't attempt to
 * setup custom routes. This can be extended if needed.
 *
 * The shortest path between the camera sensor and a video node is stored in
 * SimpleCameraData::entities_ as a list of SimpleCameraData::Entity structures,
 * ordered along the data path from the camera sensor to the video node. The
 * Entity structure stores a pointer to the MediaEntity, as well as information
 * about how it is connected in that particular path for later usage when
 * configuring the pipeline.
 *
 * Pipeline Configuration
 * ----------------------
 *
 * The simple pipeline handler configures the pipeline by propagating V4L2
 * subdev formats from the camera sensor to the video node. The format is first
 * set on the camera sensor's output, picking a resolution supported by the
 * sensor that best matches the needs of the requested streams. Then, on every
 * link in the pipeline, the format is retrieved on the link source and set
 * unmodified on the link sink.
 *
 * The best sensor resolution is selected using a heuristic that tries to
 * minimize the required bus and memory bandwidth, as the simple pipeline
 * handler is typically used on smaller, less powerful systems. To avoid the
 * need to upscale, the pipeline handler picks the smallest sensor resolution
 * large enough to accommodate the need of all streams. Resolutions that
 * significantly restrict the field of view are ignored.
 *
 * When initializating the camera data, the above format propagation procedure
 * is repeated for every media bus format and size supported by the camera
 * sensor. Upon reaching the video node, the pixel formats compatible with the
 * media bus format are enumerated. Each combination of the input media bus
 * format, output pixel format and output size are recorded in an instance of
 * the SimpleCameraData::Configuration structure, stored in the
 * SimpleCameraData::configs_ vector.
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
 *
 * Concurrent Access to Cameras
 * ----------------------------
 *
 * The cameras created by the same pipeline handler instance may share hardware
 * resources. For instances, a platform may have multiple CSI-2 receivers but a
 * single DMA engine, prohibiting usage of multiple cameras concurrently. This
 * depends heavily on the hardware architecture, which the simple pipeline
 * handler has no a priori knowledge of. The pipeline handler thus implements a
 * heuristic to handle sharing of hardware resources in a generic fashion.
 *
 * Two cameras are considered to be mutually exclusive if they share common
 * pads along the pipeline from the camera sensor to the video node. An entity
 * can thus be used concurrently by multiple cameras, as long as pads are
 * distinct.
 *
 * A resource reservation mechanism is implemented by the SimplePipelineHandler
 * acquirePipeline() and releasePipeline() functions to manage exclusive access
 * to pads. A camera reserves all the pads present in its pipeline when it is
 * started, and the start() function returns an error if any of the required
 * pads is already in use. When the camera is stopped, the pads it has reserved
 * are released.
 */

class SimplePipelineHandler;

struct SimpleFrameInfo {
	SimpleFrameInfo(uint32_t f, Request *r, bool m)
		: frame(f), request(r), metadataRequired(m), metadataProcessed(false)
	{
	}

	uint32_t frame;
	Request *request;
	bool metadataRequired;
	bool metadataProcessed;
};

class SimpleFrames
{
public:
	void create(Request *request, bool metadataRequested);
	void destroy(uint32_t frame);
	void clear();

	SimpleFrameInfo *find(uint32_t frame);

private:
	std::map<uint32_t, SimpleFrameInfo> frameInfo_;
};

void SimpleFrames::create(Request *request, bool metadataRequired)
{
	const uint32_t frame = request->sequence();
	auto [it, inserted] = frameInfo_.try_emplace(frame, frame, request, metadataRequired);
	ASSERT(inserted);
}

void SimpleFrames::destroy(uint32_t frame)
{
	frameInfo_.erase(frame);
}

void SimpleFrames::clear()
{
	frameInfo_.clear();
}

SimpleFrameInfo *SimpleFrames::find(uint32_t frame)
{
	auto info = frameInfo_.find(frame);
	if (info == frameInfo_.end())
		return nullptr;
	return &info->second;
}

struct SimplePipelineInfo {
	const char *driver;
	/*
	 * Each converter in the list contains the name
	 * and the number of streams it supports.
	 */
	std::vector<std::pair<const char *, unsigned int>> converters;
	/*
	 * Using Software ISP is to be enabled per driver.
	 *
	 * The Software ISP can't be used together with the converters.
	 */
	bool swIspEnabled;
};

namespace {

static const SimplePipelineInfo supportedDevices[] = {
	{ "dcmipp", {}, false },
	{ "imx7-csi", { { "pxp", 1 } }, false },
	{ "intel-ipu6", {}, true },
	{ "intel-ipu7", {}, true },
	{ "j721e-csi2rx", {}, true },
	{ "mtk-seninf", { { "mtk-mdp", 3 } }, false },
	{ "mxc-isi", {}, false },
	{ "qcom-camss", {}, true },
	{ "sun6i-csi", {}, false },
};

} /* namespace */

class SimpleCameraData : public Camera::Private
{
public:
	SimpleCameraData(SimplePipelineHandler *pipe,
			 unsigned int numStreams,
			 MediaEntity *sensor);

	bool isValid() const { return sensor_ != nullptr; }
	SimplePipelineHandler *pipe();

	int init();
	int setupLinks();
	int setupFormats(V4L2SubdeviceFormat *format,
			 V4L2Subdevice::Whence whence,
			 Transform transform = Transform::Identity);
	void imageBufferReady(FrameBuffer *buffer);
	void clearIncompleteRequests();

	unsigned int streamIndex(const Stream *stream) const
	{
		return stream - &streams_.front();
	}

	struct Entity {
		/* The media entity, always valid. */
		MediaEntity *entity;
		/*
		 * Whether or not the entity is a subdev that supports the
		 * routing API.
		 */
		bool supportsRouting;
		/*
		 * The local sink pad connected to the upstream entity, null for
		 * the camera sensor at the beginning of the pipeline.
		 */
		const MediaPad *sink;
		/*
		 * The local source pad connected to the downstream entity, null
		 * for the video node at the end of the pipeline.
		 */
		const MediaPad *source;
		/*
		 * The link on the source pad, to the downstream entity, null
		 * for the video node at the end of the pipeline.
		 */
		MediaLink *sourceLink;
	};

	struct Configuration {
		uint32_t code;
		Size sensorSize;
		PixelFormat captureFormat;
		Size captureSize;
		std::vector<PixelFormat> outputFormats;
		SizeRange outputSizes;
	};

	std::vector<Stream> streams_;

	/*
	 * All entities in the pipeline, from the camera sensor to the video
	 * node.
	 */
	std::list<Entity> entities_;
	std::unique_ptr<CameraSensor> sensor_;
	V4L2VideoDevice *video_;
	V4L2Subdevice *frameStartEmitter_;

	std::vector<Configuration> configs_;
	std::map<PixelFormat, std::vector<const Configuration *>> formats_;

	std::unique_ptr<DelayedControls> delayedCtrls_;

	std::vector<std::unique_ptr<FrameBuffer>> conversionBuffers_;
	struct RequestOutputs {
		Request *request;
		std::map<const Stream *, FrameBuffer *> outputs;
	};
	std::queue<RequestOutputs> conversionQueue_;
	bool useConversion_;

	std::unique_ptr<Converter> converter_;
	std::unique_ptr<SoftwareIsp> swIsp_;
	SimpleFrames frameInfo_;

private:
	void tryPipeline(unsigned int code, const Size &size);
	static std::vector<const MediaPad *> routedSourcePads(MediaPad *sink);

	void tryCompleteRequest(Request *request);
	void conversionInputDone(FrameBuffer *buffer);
	void conversionOutputDone(FrameBuffer *buffer);

	void ispStatsReady(uint32_t frame, uint32_t bufferId);
	void metadataReady(uint32_t frame, const ControlList &metadata);
	void setSensorControls(const ControlList &sensorControls);
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
	const Transform &combinedTransform() const { return combinedTransform_; }

private:
	/*
	 * The SimpleCameraData instance is guaranteed to be valid as long as
	 * the corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	SimpleCameraData *data_;

	const SimpleCameraData::Configuration *pipeConfig_;
	bool needConversion_;
	Transform combinedTransform_;
};

class SimplePipelineHandler : public PipelineHandler
{
public:
	SimplePipelineHandler(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	bool match(DeviceEnumerator *enumerator) override;

	V4L2VideoDevice *video(const MediaEntity *entity);
	V4L2Subdevice *subdev(const MediaEntity *entity);
	MediaDevice *converter() { return converter_; }
	bool swIspEnabled() const { return swIspEnabled_; }

protected:
	int queueRequestDevice(Camera *camera, Request *request) override;

private:
	static constexpr unsigned int kNumInternalBuffers = 3;

	struct EntityData {
		std::unique_ptr<V4L2VideoDevice> video;
		std::unique_ptr<V4L2Subdevice> subdev;
		std::map<const MediaPad *, SimpleCameraData *> owners;
	};

	SimpleCameraData *cameraData(Camera *camera)
	{
		return static_cast<SimpleCameraData *>(camera->_d());
	}

	bool matchDevice(MediaDevice *media, const SimplePipelineInfo &info,
			 DeviceEnumerator *enumerator);

	std::vector<MediaEntity *> locateSensors(MediaDevice *media);
	static int resetRoutingTable(V4L2Subdevice *subdev);

	const MediaPad *acquirePipeline(SimpleCameraData *data);
	void releasePipeline(SimpleCameraData *data);

	std::map<const MediaEntity *, EntityData> entities_;

	MediaDevice *converter_;
	bool swIspEnabled_;
};

/* -----------------------------------------------------------------------------
 * Camera Data
 */

SimpleCameraData::SimpleCameraData(SimplePipelineHandler *pipe,
				   unsigned int numStreams,
				   MediaEntity *sensor)
	: Camera::Private(pipe), streams_(numStreams)
{
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
	std::queue<std::tuple<MediaEntity *, MediaPad *>> queue;

	/* Remember at each entity where we came from. */
	std::unordered_map<MediaEntity *, Entity> parents;
	MediaEntity *entity = nullptr;
	MediaEntity *video = nullptr;
	MediaPad *sinkPad;

	queue.push({ sensor, nullptr });

	while (!queue.empty()) {
		std::tie(entity, sinkPad) = queue.front();
		queue.pop();

		/* Found the capture device. */
		if (entity->function() == MEDIA_ENT_F_IO_V4L) {
			LOG(SimplePipeline, Debug)
				<< "Found capture device " << entity->name();
			video = entity;
			break;
		}

		visited.insert(entity);

		/*
		 * Add direct downstream entities to the search queue. If the
		 * current entity supports the subdev internal routing API,
		 * restrict the search to downstream entities reachable through
		 * active routes.
		 */

		std::vector<const MediaPad *> pads;
		bool supportsRouting = false;

		if (sinkPad) {
			pads = routedSourcePads(sinkPad);
			if (!pads.empty())
				supportsRouting = true;
		}

		if (pads.empty()) {
			for (const MediaPad *pad : entity->pads()) {
				if (!(pad->flags() & MEDIA_PAD_FL_SOURCE))
					continue;
				pads.push_back(pad);
			}
		}

		for (const MediaPad *pad : pads) {
			for (MediaLink *link : pad->links()) {
				MediaEntity *next = link->sink()->entity();
				if (visited.find(next) == visited.end()) {
					queue.push({ next, link->sink() });

					Entity e{ entity, supportsRouting, sinkPad, pad, link };
					parents.insert({ next, e });
				}
			}
		}
	}

	if (!video)
		return;

	/*
	 * With the parents, we can follow back our way from the capture device
	 * to the sensor. Store all the entities in the pipeline, from the
	 * camera sensor to the video node, in entities_.
	 */
	entities_.push_front({ entity, false, sinkPad, nullptr, nullptr });

	for (auto it = parents.find(entity); it != parents.end();
	     it = parents.find(entity)) {
		const Entity &e = it->second;
		entities_.push_front(e);
		entity = e.entity;
	}

	/* Finally also remember the sensor. */
	sensor_ = CameraSensorFactoryBase::create(sensor);
	if (!sensor_)
		return;

	const CameraSensorProperties::SensorDelays &delays = sensor_->sensorDelays();
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
		{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
	};
	delayedCtrls_ = std::make_unique<DelayedControls>(sensor_->device(), params);

	LOG(SimplePipeline, Debug)
		<< "Found pipeline: "
		<< utils::join(entities_, " -> ",
			       [](const Entity &e) {
				       std::string s = "[";
				       if (e.sink)
					       s += std::to_string(e.sink->index()) + "|";
				       s += e.entity->name();
				       if (e.source)
					       s += "|" + std::to_string(e.source->index());
				       s += "]";
				       return s;
			       });
}

SimplePipelineHandler *SimpleCameraData::pipe()
{
	return static_cast<SimplePipelineHandler *>(Camera::Private::pipe());
}

int SimpleCameraData::init()
{
	SimplePipelineHandler *pipe = SimpleCameraData::pipe();
	int ret;

	/* Open the converter, if any. */
	MediaDevice *converter = pipe->converter();
	if (converter) {
		converter_ = ConverterFactoryBase::create(converter);
		if (!converter_) {
			LOG(SimplePipeline, Warning)
				<< "Failed to create converter, disabling format conversion";
			converter_.reset();
		} else {
			converter_->inputBufferReady.connect(this, &SimpleCameraData::conversionInputDone);
			converter_->outputBufferReady.connect(this, &SimpleCameraData::conversionOutputDone);
		}
	}

	/*
	 * Instantiate Soft ISP if this is enabled for the given driver and no converter is used.
	 */
	if (!converter_ && pipe->swIspEnabled()) {
		swIsp_ = std::make_unique<SoftwareIsp>(pipe, sensor_.get(), &controlInfo_);
		if (!swIsp_->isValid()) {
			LOG(SimplePipeline, Warning)
				<< "Failed to create software ISP, disabling software debayering";
			swIsp_.reset();
		} else {
			swIsp_->inputBufferReady.connect(this, &SimpleCameraData::conversionInputDone);
			swIsp_->outputBufferReady.connect(this, &SimpleCameraData::conversionOutputDone);
			swIsp_->ispStatsReady.connect(this, &SimpleCameraData::ispStatsReady);
			swIsp_->metadataReady.connect(this, &SimpleCameraData::metadataReady);
			swIsp_->setSensorControls.connect(this, &SimpleCameraData::setSensorControls);
		}
	}

	video_ = pipe->video(entities_.back().entity);
	ASSERT(video_);

	/*
	 * Setup links first as some subdev drivers take active links into
	 * account to propagate TRY formats. Such is life :-(
	 */
	ret = setupLinks();
	if (ret < 0)
		return ret;

	/*
	 * Generate the list of possible pipeline configurations by trying each
	 * media bus format and size supported by the sensor.
	 */
	for (unsigned int code : sensor_->mbusCodes()) {
		for (const Size &size : sensor_->sizes(code))
			tryPipeline(code, size);
	}

	if (configs_.empty()) {
		LOG(SimplePipeline, Error) << "No valid configuration found";
		return -EINVAL;
	}

	/* Map the pixel formats to configurations. */
	for (const Configuration &config : configs_) {
		formats_[config.captureFormat].push_back(&config);

		for (PixelFormat fmt : config.outputFormats)
			formats_[fmt].push_back(&config);
	}

	properties_ = sensor_->properties();

	/* Find the first subdev that can generate a frame start signal, if any. */
	frameStartEmitter_ = nullptr;
	for (const Entity &entity : entities_) {
		V4L2Subdevice *sd = pipe->subdev(entity.entity);
		if (!sd || !sd->supportsFrameStartEvent())
			continue;

		LOG(SimplePipeline, Debug)
			<< "Using frameStart signal from '"
			<< entity.entity->name() << "'";
		frameStartEmitter_ = sd;
		break;
	}

	return 0;
}

/*
 * Generate a list of supported pipeline configurations for a sensor media bus
 * code and size.
 *
 * First propagate the media bus code and size through the pipeline from the
 * camera sensor to the video node. Then, query the video node for all supported
 * pixel formats compatible with the media bus code. For each pixel format, store
 * a full pipeline configuration in the configs_ vector.
 */
void SimpleCameraData::tryPipeline(unsigned int code, const Size &size)
{
	/*
	 * Propagate the format through the pipeline, and enumerate the
	 * corresponding possible V4L2 pixel formats on the video node.
	 */
	V4L2SubdeviceFormat format{};
	format.code = code;
	format.size = size;

	int ret = setupFormats(&format, V4L2Subdevice::TryFormat);
	if (ret < 0) {
		/* Pipeline configuration failed, skip this configuration. */
		format.code = code;
		format.size = size;
		LOG(SimplePipeline, Debug)
			<< "Sensor format " << format
			<< " not supported for this pipeline";
		return;
	}

	V4L2VideoDevice::Formats videoFormats = video_->formats(format.code);

	LOG(SimplePipeline, Debug)
		<< "Adding configuration for " << format.size
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
		config.sensorSize = size;
		config.captureFormat = pixelFormat;
		config.captureSize = format.size;

		if (converter_) {
			config.outputFormats = converter_->formats(pixelFormat);
			config.outputSizes = converter_->sizes(format.size);
		} else if (swIsp_) {
			config.outputFormats = swIsp_->formats(pixelFormat);
			config.outputSizes = swIsp_->sizes(pixelFormat, format.size);
			if (config.outputFormats.empty()) {
				/* Do not use swIsp for unsupported pixelFormat's. */
				config.outputFormats = { pixelFormat };
				config.outputSizes = config.captureSize;
			}
		} else {
			config.outputFormats = { pixelFormat };
			config.outputSizes = config.captureSize;
		}

		configs_.push_back(config);
	}
}

int SimpleCameraData::setupLinks()
{
	int ret;

	/*
	 * Configure all links along the pipeline. Some entities may not allow
	 * multiple sink links to be enabled together, even on different sink
	 * pads. We must thus start by disabling all sink links (but the one we
	 * want to enable) before enabling the pipeline link.
	 *
	 * The entities_ list stores entities along with their source link. We
	 * need to process the link in the context of the sink entity, so
	 * record the source link of the current entity as the sink link of the
	 * next entity, and skip the first entity in the loop.
	 */
	MediaLink *sinkLink = nullptr;

	for (SimpleCameraData::Entity &e : entities_) {
		if (!sinkLink) {
			sinkLink = e.sourceLink;
			continue;
		}

		for (MediaPad *pad : e.entity->pads()) {
			/*
			 * If the entity supports the V4L2 internal routing API,
			 * assume that it may carry multiple independent streams
			 * concurrently, and only disable links on the sink and
			 * source pads used by the pipeline.
			 */
			if (e.supportsRouting && pad != e.sink && pad != e.source)
				continue;

			for (MediaLink *link : pad->links()) {
				if (link == sinkLink)
					continue;

				if ((link->flags() & MEDIA_LNK_FL_ENABLED) &&
				    !(link->flags() & MEDIA_LNK_FL_IMMUTABLE)) {
					ret = link->setEnabled(false);
					if (ret < 0)
						return ret;
				}
			}
		}

		if (!(sinkLink->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = sinkLink->setEnabled(true);
			if (ret < 0)
				return ret;
		}

		sinkLink = e.sourceLink;
	}

	return 0;
}

int SimpleCameraData::setupFormats(V4L2SubdeviceFormat *format,
				   V4L2Subdevice::Whence whence,
				   Transform transform)
{
	SimplePipelineHandler *pipe = SimpleCameraData::pipe();
	int ret;

	/*
	 * Configure the format on the sensor output and propagate it through
	 * the pipeline.
	 */
	ret = sensor_->setFormat(format, transform);
	if (ret < 0)
		return ret;

	for (const Entity &e : entities_) {
		if (!e.sourceLink)
			break;

		MediaLink *link = e.sourceLink;
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

			if (format->code != sourceFormat.code ||
			    format->size != sourceFormat.size) {
				LOG(SimplePipeline, Debug)
					<< "Source '" << source->entity()->name()
					<< "':" << source->index()
					<< " produces " << sourceFormat
					<< ", sink '" << sink->entity()->name()
					<< "':" << sink->index()
					<< " requires " << *format;
				return -EINVAL;
			}
		}

		LOG(SimplePipeline, Debug)
			<< "Link " << *link << ": configured with format "
			<< *format;
	}

	return 0;
}

void SimpleCameraData::imageBufferReady(FrameBuffer *buffer)
{
	SimplePipelineHandler *pipe = SimpleCameraData::pipe();

	/*
	 * If an error occurred during capture, or if the buffer was cancelled,
	 * complete the request, even if the converter is in use as there's no
	 * point converting an erroneous buffer.
	 */
	if (buffer->metadata().status != FrameMetadata::FrameSuccess) {
		if (!useConversion_) {
			/* No conversion, just complete the request. */
			Request *request = buffer->request();
			pipe->completeBuffer(request, buffer);
			tryCompleteRequest(request);
			return;
		}

		/*
		 * The converter or Software ISP is in use. Requeue the internal
		 * buffer for capture (unless the stream is being stopped), and
		 * complete the request with all the user-facing buffers.
		 */
		if (buffer->metadata().status != FrameMetadata::FrameCancelled)
			video_->queueBuffer(buffer);

		if (conversionQueue_.empty())
			return;

		const RequestOutputs &outputs = conversionQueue_.front();
		for (auto &[stream, buf] : outputs.outputs)
			pipe->completeBuffer(outputs.request, buf);
		SimpleFrameInfo *info = frameInfo_.find(outputs.request->sequence());
		if (info)
			info->metadataRequired = false;
		tryCompleteRequest(outputs.request);
		conversionQueue_.pop();

		return;
	}

	/*
	 * Record the sensor's timestamp in the request metadata. The request
	 * needs to be obtained from the user-facing buffer, as internal
	 * buffers are free-wheeling and have no request associated with them.
	 *
	 * \todo The sensor timestamp should be better estimated by connecting
	 * to the V4L2Device::frameStart signal if the platform provides it.
	 */
	Request *request = buffer->request();

	if (useConversion_ && !conversionQueue_.empty()) {
		const std::map<const Stream *, FrameBuffer *> &outputs =
			conversionQueue_.front().outputs;
		if (!outputs.empty()) {
			FrameBuffer *outputBuffer = outputs.begin()->second;
			if (outputBuffer)
				request = outputBuffer->request();
		}
	}

	if (request)
		request->metadata().set(controls::SensorTimestamp,
					buffer->metadata().timestamp);

	/*
	 * Queue the captured and the request buffer to the converter or Software
	 * ISP if format conversion is needed. If there's no queued request, just
	 * requeue the captured buffer for capture.
	 */
	if (useConversion_) {
		if (conversionQueue_.empty()) {
			video_->queueBuffer(buffer);
			return;
		}

		if (converter_)
			converter_->queueBuffers(buffer, conversionQueue_.front().outputs);
		else
			/*
			 * request->sequence() cannot be retrieved from `buffer' inside
			 * queueBuffers because unique_ptr's make buffer->request() invalid
			 * already here.
			 */
			swIsp_->queueBuffers(request->sequence(), buffer,
					     conversionQueue_.front().outputs);

		conversionQueue_.pop();
		return;
	}

	/* Otherwise simply complete the request. */
	pipe->completeBuffer(request, buffer);
	tryCompleteRequest(request);
}

void SimpleCameraData::clearIncompleteRequests()
{
	while (!conversionQueue_.empty()) {
		pipe()->cancelRequest(conversionQueue_.front().request);
		conversionQueue_.pop();
	}
}

void SimpleCameraData::tryCompleteRequest(Request *request)
{
	if (request->hasPendingBuffers())
		return;

	SimpleFrameInfo *info = frameInfo_.find(request->sequence());
	if (!info) {
		/* Something is really wrong, let's return. */
		return;
	}

	if (info->metadataRequired && !info->metadataProcessed)
		return;

	frameInfo_.destroy(info->frame);
	pipe()->completeRequest(request);
}

void SimpleCameraData::conversionInputDone(FrameBuffer *buffer)
{
	/* Queue the input buffer back for capture. */
	video_->queueBuffer(buffer);
}

void SimpleCameraData::conversionOutputDone(FrameBuffer *buffer)
{
	SimplePipelineHandler *pipe = SimpleCameraData::pipe();

	/* Complete the buffer and the request. */
	Request *request = buffer->request();
	if (pipe->completeBuffer(request, buffer))
		tryCompleteRequest(request);
}

void SimpleCameraData::ispStatsReady(uint32_t frame, uint32_t bufferId)
{
	swIsp_->processStats(frame, bufferId,
			     delayedCtrls_->get(frame));
}

void SimpleCameraData::metadataReady(uint32_t frame, const ControlList &metadata)
{
	SimpleFrameInfo *info = frameInfo_.find(frame);
	if (!info)
		return;

	info->request->metadata().merge(metadata);
	info->metadataProcessed = true;
	tryCompleteRequest(info->request);
}

void SimpleCameraData::setSensorControls(const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
	/*
	 * Directly apply controls now if there is no frameStart signal.
	 *
	 * \todo Applying controls directly not only increases the risk of
	 * applying them to the wrong frame (or across a frame boundary),
	 * but it also bypasses delayedCtrls_, creating AGC regulation issues.
	 * Both problems should be fixed.
	 */
	if (!frameStartEmitter_) {
		ControlList ctrls(sensorControls);
		sensor_->setControls(&ctrls);
	}
}

/* Retrieve all source pads connected to a sink pad through active routes. */
std::vector<const MediaPad *> SimpleCameraData::routedSourcePads(MediaPad *sink)
{
	MediaEntity *entity = sink->entity();
	std::unique_ptr<V4L2Subdevice> subdev =
		std::make_unique<V4L2Subdevice>(entity);

	int ret = subdev->open();
	if (ret < 0)
		return {};

	V4L2Subdevice::Routing routing = {};
	ret = subdev->getRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret < 0)
		return {};

	std::vector<const MediaPad *> pads;

	for (const V4L2Subdevice::Route &route : routing) {
		if (sink->index() != route.sink.pad ||
		    !(route.flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		const MediaPad *pad = entity->getPadByIndex(route.source.pad);
		if (!pad) {
			LOG(SimplePipeline, Warning)
				<< "Entity " << entity->name()
				<< " has invalid route source pad "
				<< route.source.pad;
		}

		pads.push_back(pad);
	}

	return pads;
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

namespace {

static Size adjustSize(const Size &requestedSize, const SizeRange &supportedSizes)
{
	ASSERT(supportedSizes.min <= supportedSizes.max);

	if (supportedSizes.min == supportedSizes.max)
		return supportedSizes.max;

	unsigned int hStep = supportedSizes.hStep;
	unsigned int vStep = supportedSizes.vStep;

	if (hStep == 0)
		hStep = supportedSizes.max.width - supportedSizes.min.width;
	if (vStep == 0)
		vStep = supportedSizes.max.height - supportedSizes.min.height;

	Size adjusted = requestedSize.boundedTo(supportedSizes.max)
				.expandedTo(supportedSizes.min);

	return adjusted.shrunkBy(supportedSizes.min)
		.alignedDownTo(hStep, vStep)
		.grownBy(supportedSizes.min);
}

} /* namespace */

CameraConfiguration::Status SimpleCameraConfiguration::validate()
{
	const CameraSensor *sensor = data_->sensor_.get();
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	Orientation requestedOrientation = orientation;
	combinedTransform_ = sensor->computeTransform(&orientation);
	if (orientation != requestedOrientation)
		status = Adjusted;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > data_->streams_.size()) {
		config_.resize(data_->streams_.size());
		status = Adjusted;
	}

	/* Find the largest stream size. */
	Size maxStreamSize;
	for (const StreamConfiguration &cfg : config_)
		maxStreamSize.expandTo(cfg.size);

	LOG(SimplePipeline, Debug)
		<< "Largest stream size is " << maxStreamSize;

	/*
	 * Find the best configuration for the pipeline using a heuristic.
	 * First select the pixel format based on the streams (which are
	 * considered ordered from highest to lowest priority). Default to the
	 * first pipeline configuration if no streams request a supported pixel
	 * format.
	 */
	const std::vector<const SimpleCameraData::Configuration *> *configs =
		&data_->formats_.begin()->second;

	for (const StreamConfiguration &cfg : config_) {
		auto it = data_->formats_.find(cfg.pixelFormat);
		if (it != data_->formats_.end()) {
			configs = &it->second;
			break;
		}
	}

	/*
	 * \todo Pick the best sensor output media bus format when the
	 * requested pixel format can be produced from multiple sensor media
	 * bus formats.
	 */

	/*
	 * Then pick, among the possible configuration for the pixel format,
	 * the smallest sensor resolution that can accommodate all streams
	 * without upscaling.
	 */
	const SimpleCameraData::Configuration *maxPipeConfig = nullptr;
	pipeConfig_ = nullptr;

	for (const SimpleCameraData::Configuration *pipeConfig : *configs) {
		const Size &size = pipeConfig->captureSize;

		if (size.width >= maxStreamSize.width &&
		    size.height >= maxStreamSize.height) {
			if (!pipeConfig_ || size < pipeConfig_->captureSize)
				pipeConfig_ = pipeConfig;
		}

		if (!maxPipeConfig || maxPipeConfig->captureSize < size)
			maxPipeConfig = pipeConfig;
	}

	/* If no configuration was large enough, select the largest one. */
	if (!pipeConfig_)
		pipeConfig_ = maxPipeConfig;

	LOG(SimplePipeline, Debug)
		<< "Picked "
		<< V4L2SubdeviceFormat{ pipeConfig_->code, pipeConfig_->sensorSize, {} }
		<< " -> " << pipeConfig_->captureSize
		<< "-" << pipeConfig_->captureFormat
		<< " for max stream size " << maxStreamSize;

	/*
	 * Adjust the requested streams.
	 *
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
			LOG(SimplePipeline, Debug)
				<< "Adjusting pixel format from "
				<< cfg.pixelFormat << " to " << pixelFormat;
			cfg.pixelFormat = pixelFormat;
			status = Adjusted;
		}

		if (!pipeConfig_->outputSizes.contains(cfg.size)) {
			Size adjustedSize = pipeConfig_->captureSize;
			/*
			 * The converter (when present) may not be able to output
			 * a size identical to its input size. The capture size is thus
			 * not guaranteed to be a valid output size. In such cases, use
			 * the smaller valid output size closest to the requested.
			 */
			if (!pipeConfig_->outputSizes.contains(adjustedSize))
				adjustedSize = adjustSize(cfg.size, pipeConfig_->outputSizes);
			LOG(SimplePipeline, Debug)
				<< "Adjusting size from " << cfg.size
				<< " to " << adjustedSize;
			cfg.size = adjustedSize;
			status = Adjusted;
		}

		/* \todo Create a libcamera core class to group format and size */
		if (cfg.pixelFormat != pipeConfig_->captureFormat ||
		    cfg.size != pipeConfig_->captureSize)
			needConversion_ = true;

		/* Set the stride, frameSize and bufferCount. */
		if (needConversion_) {
			std::tie(cfg.stride, cfg.frameSize) =
				data_->converter_
					? data_->converter_->strideAndFrameSize(cfg.pixelFormat,
										cfg.size)
					: data_->swIsp_->strideAndFrameSize(cfg.pixelFormat,
									    cfg.size);
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

		cfg.bufferCount = 4;
	}

	return status;
}

/* -----------------------------------------------------------------------------
 * Pipeline Handler
 */

SimplePipelineHandler::SimplePipelineHandler(CameraManager *manager)
	: PipelineHandler(manager), converter_(nullptr)
{
}

std::unique_ptr<CameraConfiguration>
SimplePipelineHandler::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	SimpleCameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<SimpleCameraConfiguration>(camera, data);

	if (roles.empty())
		return config;

	/* Create the formats map. */
	std::map<PixelFormat, std::vector<SizeRange>> formats;

	for (const SimpleCameraData::Configuration &cfg : data->configs_) {
		for (PixelFormat format : cfg.outputFormats)
			formats[format].push_back(cfg.outputSizes);
	}

	/* Sort the sizes and merge any consecutive overlapping ranges. */
	for (auto &[format, sizes] : formats) {
		std::sort(sizes.begin(), sizes.end(),
			  [](SizeRange &a, SizeRange &b) {
				  return a.min < b.min;
			  });

		auto cur = sizes.begin();
		auto next = cur;

		while (++next != sizes.end()) {
			if (cur->max.width >= next->min.width &&
			    cur->max.height >= next->min.height)
				cur->max = next->max;
			else if (++cur != next)
				*cur = *next;
		}

		sizes.erase(++cur, sizes.end());
	}

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
	V4L2SubdeviceFormat format{};
	format.code = pipeConfig->code;
	format.size = pipeConfig->sensorSize;

	ret = data->setupFormats(&format, V4L2Subdevice::ActiveFormat,
				 config->combinedTransform());
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
			<< pipeConfig->captureSize << "-" << videoFormat
			<< " (got " << captureFormat << ")";
		return -EINVAL;
	}

	/* Configure the converter if needed. */
	std::vector<std::reference_wrapper<StreamConfiguration>> outputCfgs;
	data->useConversion_ = config->needConversion();

	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = config->at(i);

		cfg.setStream(&data->streams_[i]);

		if (data->useConversion_)
			outputCfgs.push_back(cfg);
	}

	if (outputCfgs.empty())
		return 0;

	StreamConfiguration inputCfg;
	inputCfg.pixelFormat = pipeConfig->captureFormat;
	inputCfg.size = pipeConfig->captureSize;
	inputCfg.stride = captureFormat.planes[0].bpl;
	inputCfg.bufferCount = kNumInternalBuffers;

	if (data->converter_) {
		return data->converter_->configure(inputCfg, outputCfgs);
	} else {
		ipa::soft::IPAConfigInfo configInfo;
		configInfo.sensorControls = data->sensor_->controls();
		return data->swIsp_->configure(inputCfg, outputCfgs, configInfo);
	}
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
	if (data->useConversion_)
		return data->converter_
			       ? data->converter_->exportBuffers(stream, count, buffers)
			       : data->swIsp_->exportBuffers(stream, count, buffers);
	else
		return data->video_->exportBuffers(count, buffers);
}

int SimplePipelineHandler::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	SimpleCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	V4L2Subdevice *frameStartEmitter = data->frameStartEmitter_;
	int ret;

	const MediaPad *pad = acquirePipeline(data);
	if (pad) {
		LOG(SimplePipeline, Info)
			<< "Failed to acquire pipeline, entity "
			<< pad->entity()->name() << " in use";
		return -EBUSY;
	}

	if (data->useConversion_) {
		/*
		 * When using the converter allocate a fixed number of internal
		 * buffers.
		 */
		ret = video->allocateBuffers(kNumInternalBuffers,
					     &data->conversionBuffers_);
	} else {
		/* Otherwise, prepare for using buffers from the only stream. */
		Stream *stream = &data->streams_[0];
		ret = video->importBuffers(stream->configuration().bufferCount);
	}
	if (ret < 0) {
		releasePipeline(data);
		return ret;
	}

	video->bufferReady.connect(data, &SimpleCameraData::imageBufferReady);

	data->delayedCtrls_->reset();
	if (frameStartEmitter) {
		ret = frameStartEmitter->setFrameStartEnabled(true);
		if (ret) {
			stop(camera);
			return ret;
		}
		frameStartEmitter->frameStart.connect(data->delayedCtrls_.get(),
						      &DelayedControls::applyControls);
	}

	ret = video->streamOn();
	if (ret < 0) {
		stop(camera);
		return ret;
	}

	if (data->useConversion_) {
		if (data->converter_)
			ret = data->converter_->start();
		else if (data->swIsp_)
			ret = data->swIsp_->start();
		else
			ret = 0;

		if (ret < 0) {
			stop(camera);
			return ret;
		}

		/* Queue all internal buffers for capture. */
		for (std::unique_ptr<FrameBuffer> &buffer : data->conversionBuffers_)
			video->queueBuffer(buffer.get());
	}

	return 0;
}

void SimplePipelineHandler::stopDevice(Camera *camera)
{
	SimpleCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	V4L2Subdevice *frameStartEmitter = data->frameStartEmitter_;

	if (frameStartEmitter) {
		frameStartEmitter->setFrameStartEnabled(false);
		frameStartEmitter->frameStart.disconnect(data->delayedCtrls_.get(),
							 &DelayedControls::applyControls);
	}

	if (data->useConversion_) {
		if (data->converter_)
			data->converter_->stop();
		else if (data->swIsp_)
			data->swIsp_->stop();
	}

	video->streamOff();
	video->releaseBuffers();

	video->bufferReady.disconnect(data, &SimpleCameraData::imageBufferReady);

	data->frameInfo_.clear();
	data->clearIncompleteRequests();
	data->conversionBuffers_.clear();

	releasePipeline(data);
}

int SimplePipelineHandler::queueRequestDevice(Camera *camera, Request *request)
{
	SimpleCameraData *data = cameraData(camera);
	int ret;

	std::map<const Stream *, FrameBuffer *> buffers;

	for (auto &[stream, buffer] : request->buffers()) {
		/*
		 * If conversion is needed, push the buffer to the converter
		 * queue, it will be handed to the converter in the capture
		 * completion handler.
		 */
		if (data->useConversion_) {
			buffers.emplace(stream, buffer);
		} else {
			ret = data->video_->queueBuffer(buffer);
			if (ret < 0)
				return ret;
		}
	}

	data->frameInfo_.create(request, !!data->swIsp_);
	if (data->useConversion_) {
		data->conversionQueue_.push({ request, std::move(buffers) });
		if (data->swIsp_)
			data->swIsp_->queueRequest(request->sequence(), request->controls());
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

std::vector<MediaEntity *>
SimplePipelineHandler::locateSensors(MediaDevice *media)
{
	std::vector<MediaEntity *> entities;

	/*
	 * Gather all the camera sensor entities based on the function they
	 * expose.
	 */
	for (MediaEntity *entity : media->entities()) {
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

int SimplePipelineHandler::resetRoutingTable(V4L2Subdevice *subdev)
{
	/* Reset the media entity routing table to its default state. */
	V4L2Subdevice::Routing routing = {};

	int ret = subdev->getRouting(&routing, V4L2Subdevice::TryFormat);
	if (ret)
		return ret;

	ret = subdev->setRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret)
		return ret;

	/*
	 * If the routing table is empty we won't be able to meaningfully use
	 * the subdev.
	 */
	if (routing.empty()) {
		LOG(SimplePipeline, Error)
			<< "Default routing table of " << subdev->deviceNode()
			<< " is empty";
		return -EINVAL;
	}

	LOG(SimplePipeline, Debug)
		<< "Routing table of " << subdev->deviceNode()
		<< " reset to " << routing;

	return 0;
}

bool SimplePipelineHandler::matchDevice(MediaDevice *media,
					const SimplePipelineInfo &info,
					DeviceEnumerator *enumerator)
{
	unsigned int numStreams = 1;

	for (const auto &[name, streams] : info.converters) {
		DeviceMatch converterMatch(name);
		converter_ = acquireMediaDevice(enumerator, converterMatch);
		if (converter_) {
			numStreams = streams;
			break;
		}
	}

	swIspEnabled_ = info.swIspEnabled;

	/* Locate the sensors. */
	std::vector<MediaEntity *> sensors = locateSensors(media);
	if (sensors.empty()) {
		LOG(SimplePipeline, Info) << "No sensor found for " << media->deviceNode();
		return false;
	}

	LOG(SimplePipeline, Debug) << "Sensor found for " << media->deviceNode();

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

	/*
	 * Insert all entities in the global entities list. Create and open
	 * V4L2VideoDevice and V4L2Subdevice instances for the corresponding
	 * entities.
	 */
	for (MediaEntity *entity : entities) {
		std::unique_ptr<V4L2VideoDevice> video;
		std::unique_ptr<V4L2Subdevice> subdev;
		int ret;

		switch (entity->type()) {
		case MediaEntity::Type::V4L2VideoDevice:
			video = std::make_unique<V4L2VideoDevice>(entity);
			ret = video->open();
			if (ret < 0) {
				LOG(SimplePipeline, Error)
					<< "Failed to open " << video->deviceNode()
					<< ": " << strerror(-ret);
				return false;
			}
			break;

		case MediaEntity::Type::V4L2Subdevice:
			subdev = std::make_unique<V4L2Subdevice>(entity);
			ret = subdev->open();
			if (ret < 0) {
				LOG(SimplePipeline, Error)
					<< "Failed to open " << subdev->deviceNode()
					<< ": " << strerror(-ret);
				return false;
			}

			if (subdev->caps().hasStreams()) {
				/*
				 * Reset the routing table to its default state
				 * to make sure entities are enumerated according
				 * to the default routing configuration.
				 */
				ret = resetRoutingTable(subdev.get());
				if (ret) {
					LOG(SimplePipeline, Error)
						<< "Failed to reset routes for "
						<< subdev->deviceNode() << ": "
						<< strerror(-ret);
					return false;
				}
			}

			break;

		default:
			break;
		}

		entities_[entity] = { std::move(video), std::move(subdev), {} };
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

		const std::string &id = data->sensor_->id();
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), id, streams);
		registerCamera(std::move(camera));
		registered = true;
	}

	return registered;
}

bool SimplePipelineHandler::match(DeviceEnumerator *enumerator)
{
	MediaDevice *media;

	for (const SimplePipelineInfo &inf : supportedDevices) {
		DeviceMatch dm(inf.driver);
		while ((media = acquireMediaDevice(enumerator, dm))) {
			/*
			 * If match succeeds, return true to let match() be
			 * called again on a new instance of the pipeline
			 * handler. Otherwise keep looping until we do
			 * successfully match one (or run out).
			 */
			if (matchDevice(media, inf, enumerator)) {
				LOG(SimplePipeline, Debug)
					<< "Matched on device: "
					<< media->deviceNode();
				return true;
			}

			/*
			 * \todo We need to clear the list of media devices
			 * that we've already acquired in the event that we
			 * fail to create a camera. This requires a rework of
			 * DeviceEnumerator, or even how we create pipelines
			 * handlers. This is because at the moment acquired
			 * media devices are only released on pipeline handler
			 * deconstruction, and if we release them any earlier
			 * then DeviceEnumerator::search() will keep returning
			 * the same media devices.
			 */
		}
	}

	return false;
}

V4L2VideoDevice *SimplePipelineHandler::video(const MediaEntity *entity)
{
	auto iter = entities_.find(entity);
	if (iter == entities_.end())
		return nullptr;

	return iter->second.video.get();
}

V4L2Subdevice *SimplePipelineHandler::subdev(const MediaEntity *entity)
{
	auto iter = entities_.find(entity);
	if (iter == entities_.end())
		return nullptr;

	return iter->second.subdev.get();
}

/**
 * \brief Acquire all resources needed by the camera pipeline
 * \return nullptr on success, a pointer to the contended pad on error
 */
const MediaPad *SimplePipelineHandler::acquirePipeline(SimpleCameraData *data)
{
	for (const SimpleCameraData::Entity &entity : data->entities_) {
		const EntityData &edata = entities_[entity.entity];

		if (entity.sink) {
			auto iter = edata.owners.find(entity.sink);
			if (iter != edata.owners.end() && iter->second != data)
				return entity.sink;
		}

		if (entity.source) {
			auto iter = edata.owners.find(entity.source);
			if (iter != edata.owners.end() && iter->second != data)
				return entity.source;
		}
	}

	for (const SimpleCameraData::Entity &entity : data->entities_) {
		EntityData &edata = entities_[entity.entity];

		if (entity.sink)
			edata.owners[entity.sink] = data;
		if (entity.source)
			edata.owners[entity.source] = data;
	}

	return nullptr;
}

void SimplePipelineHandler::releasePipeline(SimpleCameraData *data)
{
	for (const SimpleCameraData::Entity &entity : data->entities_) {
		EntityData &edata = entities_[entity.entity];

		if (entity.sink) {
			auto iter = edata.owners.find(entity.sink);
			ASSERT(iter->second == data);
			edata.owners.erase(iter);
		}

		if (entity.source) {
			auto iter = edata.owners.find(entity.source);
			ASSERT(iter->second == data);
			edata.owners.erase(iter);
		}
	}
}

REGISTER_PIPELINE_HANDLER(SimplePipelineHandler, "simple")

} /* namespace libcamera */
