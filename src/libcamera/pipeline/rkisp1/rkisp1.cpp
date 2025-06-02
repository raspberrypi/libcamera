/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Pipeline handler for Rockchip ISP1
 */

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include <vector>

#include <linux/media-bus-format.h>
#include <linux/rkisp1-config.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/color_space.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <libcamera/transform.h>

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/rkisp1_ipa_interface.h>
#include <libcamera/ipa/rkisp1_ipa_proxy.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/converter/converter_v4l2_m2m.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_pipeline.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "rkisp1_path.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(RkISP1)

class PipelineHandlerRkISP1;
class RkISP1CameraData;

struct RkISP1FrameInfo {
	unsigned int frame;
	Request *request;

	FrameBuffer *paramBuffer;
	FrameBuffer *statBuffer;
	FrameBuffer *mainPathBuffer;
	FrameBuffer *selfPathBuffer;

	bool paramDequeued;
	bool metadataProcessed;
};

class RkISP1Frames
{
public:
	RkISP1Frames(PipelineHandler *pipe);

	RkISP1FrameInfo *create(const RkISP1CameraData *data, Request *request,
				bool isRaw);
	int destroy(unsigned int frame);
	void clear();

	RkISP1FrameInfo *find(unsigned int frame);
	RkISP1FrameInfo *find(FrameBuffer *buffer);
	RkISP1FrameInfo *find(Request *request);

private:
	PipelineHandlerRkISP1 *pipe_;
	std::map<unsigned int, RkISP1FrameInfo *> frameInfo_;
};

class RkISP1CameraData : public Camera::Private
{
public:
	RkISP1CameraData(PipelineHandler *pipe, RkISP1MainPath *mainPath,
			 RkISP1SelfPath *selfPath)
		: Camera::Private(pipe), frame_(0), frameInfo_(pipe),
		  mainPath_(mainPath), selfPath_(selfPath)
	{
	}

	PipelineHandlerRkISP1 *pipe();
	const PipelineHandlerRkISP1 *pipe() const;
	int loadIPA(unsigned int hwRevision);

	Stream mainPathStream_;
	Stream selfPathStream_;
	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<DelayedControls> delayedCtrls_;
	unsigned int frame_;
	std::vector<IPABuffer> ipaBuffers_;
	RkISP1Frames frameInfo_;

	RkISP1MainPath *mainPath_;
	RkISP1SelfPath *selfPath_;

	std::unique_ptr<ipa::rkisp1::IPAProxyRkISP1> ipa_;

	ControlInfoMap ipaControls_;

	/*
	 * All entities in the pipeline, from the camera sensor to the RKISP1.
	 */
	MediaPipeline pipe_;

private:
	void paramsComputed(unsigned int frame, unsigned int bytesused);
	void setSensorControls(unsigned int frame,
			       const ControlList &sensorControls);

	void metadataReady(unsigned int frame, const ControlList &metadata);
};

class RkISP1CameraConfiguration : public CameraConfiguration
{
public:
	RkISP1CameraConfiguration(Camera *camera, RkISP1CameraData *data);

	Status validate() override;

	const V4L2SubdeviceFormat &sensorFormat() { return sensorFormat_; }
	const Transform &combinedTransform() { return combinedTransform_; }

private:
	bool fitsAllPaths(const StreamConfiguration &cfg);

	/*
	 * The RkISP1CameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	const RkISP1CameraData *data_;

	V4L2SubdeviceFormat sensorFormat_;
	Transform combinedTransform_;
};

class PipelineHandlerRkISP1 : public PipelineHandler
{
public:
	PipelineHandlerRkISP1(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	static constexpr Size kRkISP1PreviewSize = { 1920, 1080 };

	RkISP1CameraData *cameraData(Camera *camera)
	{
		return static_cast<RkISP1CameraData *>(camera->_d());
	}

	friend RkISP1CameraData;
	friend RkISP1CameraConfiguration;
	friend RkISP1Frames;

	int initLinks(Camera *camera, const RkISP1CameraConfiguration &config);
	int createCamera(MediaEntity *sensor);
	void tryCompleteRequest(RkISP1FrameInfo *info);
	void imageBufferReady(FrameBuffer *buffer);
	void paramBufferReady(FrameBuffer *buffer);
	void statBufferReady(FrameBuffer *buffer);
	void dewarpBufferReady(FrameBuffer *buffer);
	void frameStart(uint32_t sequence);

	int allocateBuffers(Camera *camera);
	int freeBuffers(Camera *camera);

	int updateControls(RkISP1CameraData *data);

	MediaDevice *media_;
	std::unique_ptr<V4L2Subdevice> isp_;
	std::unique_ptr<V4L2VideoDevice> param_;
	std::unique_ptr<V4L2VideoDevice> stat_;

	bool hasSelfPath_;
	bool isRaw_;

	RkISP1MainPath mainPath_;
	RkISP1SelfPath selfPath_;

	std::unique_ptr<V4L2M2MConverter> dewarper_;
	Rectangle scalerMaxCrop_;
	bool useDewarper_;

	std::optional<Rectangle> activeCrop_;

	/* Internal buffers used when dewarper is being used */
	std::vector<std::unique_ptr<FrameBuffer>> mainPathBuffers_;
	std::queue<FrameBuffer *> availableMainPathBuffers_;

	std::vector<std::unique_ptr<FrameBuffer>> paramBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statBuffers_;
	std::queue<FrameBuffer *> availableParamBuffers_;
	std::queue<FrameBuffer *> availableStatBuffers_;

	Camera *activeCamera_;
};

RkISP1Frames::RkISP1Frames(PipelineHandler *pipe)
	: pipe_(static_cast<PipelineHandlerRkISP1 *>(pipe))
{
}

RkISP1FrameInfo *RkISP1Frames::create(const RkISP1CameraData *data, Request *request,
				      bool isRaw)
{
	unsigned int frame = data->frame_;

	FrameBuffer *paramBuffer = nullptr;
	FrameBuffer *statBuffer = nullptr;
	FrameBuffer *mainPathBuffer = nullptr;
	FrameBuffer *selfPathBuffer = nullptr;

	if (!isRaw) {
		if (pipe_->availableParamBuffers_.empty()) {
			LOG(RkISP1, Error) << "Parameters buffer underrun";
			return nullptr;
		}

		if (pipe_->availableStatBuffers_.empty()) {
			LOG(RkISP1, Error) << "Statistic buffer underrun";
			return nullptr;
		}

		paramBuffer = pipe_->availableParamBuffers_.front();
		pipe_->availableParamBuffers_.pop();

		statBuffer = pipe_->availableStatBuffers_.front();
		pipe_->availableStatBuffers_.pop();

		if (pipe_->useDewarper_) {
			mainPathBuffer = pipe_->availableMainPathBuffers_.front();
			pipe_->availableMainPathBuffers_.pop();
		}
	}

	if (!mainPathBuffer)
		mainPathBuffer = request->findBuffer(&data->mainPathStream_);
	selfPathBuffer = request->findBuffer(&data->selfPathStream_);

	RkISP1FrameInfo *info = new RkISP1FrameInfo;

	info->frame = frame;
	info->request = request;
	info->paramBuffer = paramBuffer;
	info->mainPathBuffer = mainPathBuffer;
	info->selfPathBuffer = selfPathBuffer;
	info->statBuffer = statBuffer;
	info->paramDequeued = false;
	info->metadataProcessed = false;

	frameInfo_[frame] = info;

	return info;
}

int RkISP1Frames::destroy(unsigned int frame)
{
	RkISP1FrameInfo *info = find(frame);
	if (!info)
		return -ENOENT;

	pipe_->availableParamBuffers_.push(info->paramBuffer);
	pipe_->availableStatBuffers_.push(info->statBuffer);
	pipe_->availableMainPathBuffers_.push(info->mainPathBuffer);

	frameInfo_.erase(info->frame);

	delete info;

	return 0;
}

void RkISP1Frames::clear()
{
	for (const auto &entry : frameInfo_) {
		RkISP1FrameInfo *info = entry.second;

		pipe_->availableParamBuffers_.push(info->paramBuffer);
		pipe_->availableStatBuffers_.push(info->statBuffer);
		pipe_->availableMainPathBuffers_.push(info->mainPathBuffer);

		delete info;
	}

	frameInfo_.clear();
}

RkISP1FrameInfo *RkISP1Frames::find(unsigned int frame)
{
	auto itInfo = frameInfo_.find(frame);

	if (itInfo != frameInfo_.end())
		return itInfo->second;

	LOG(RkISP1, Fatal) << "Can't locate info from frame";

	return nullptr;
}

RkISP1FrameInfo *RkISP1Frames::find(FrameBuffer *buffer)
{
	for (auto &itInfo : frameInfo_) {
		RkISP1FrameInfo *info = itInfo.second;

		if (info->paramBuffer == buffer ||
		    info->statBuffer == buffer ||
		    info->mainPathBuffer == buffer ||
		    info->selfPathBuffer == buffer)
			return info;
	}

	LOG(RkISP1, Fatal) << "Can't locate info from buffer";

	return nullptr;
}

RkISP1FrameInfo *RkISP1Frames::find(Request *request)
{
	for (auto &itInfo : frameInfo_) {
		RkISP1FrameInfo *info = itInfo.second;

		if (info->request == request)
			return info;
	}

	LOG(RkISP1, Fatal) << "Can't locate info from request";

	return nullptr;
}

PipelineHandlerRkISP1 *RkISP1CameraData::pipe()
{
	return static_cast<PipelineHandlerRkISP1 *>(Camera::Private::pipe());
}

const PipelineHandlerRkISP1 *RkISP1CameraData::pipe() const
{
	return static_cast<const PipelineHandlerRkISP1 *>(Camera::Private::pipe());
}

int RkISP1CameraData::loadIPA(unsigned int hwRevision)
{
	ipa_ = IPAManager::createIPA<ipa::rkisp1::IPAProxyRkISP1>(pipe(), 1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->setSensorControls.connect(this, &RkISP1CameraData::setSensorControls);
	ipa_->paramsComputed.connect(this, &RkISP1CameraData::paramsComputed);
	ipa_->metadataReady.connect(this, &RkISP1CameraData::metadataReady);

	/* The IPA tuning file is made from the sensor name. */
	std::string ipaTuningFile =
		ipa_->configurationFile(sensor_->model() + ".yaml", "uncalibrated.yaml");

	IPACameraSensorInfo sensorInfo{};
	int ret = sensor_->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(RkISP1, Error) << "Camera sensor information not available";
		return ret;
	}

	ret = ipa_->init({ ipaTuningFile, sensor_->model() }, hwRevision,
			 sensorInfo, sensor_->controls(), &ipaControls_);
	if (ret < 0) {
		LOG(RkISP1, Error) << "IPA initialization failure";
		return ret;
	}

	return 0;
}

void RkISP1CameraData::paramsComputed(unsigned int frame, unsigned int bytesused)
{
	PipelineHandlerRkISP1 *pipe = RkISP1CameraData::pipe();
	RkISP1FrameInfo *info = frameInfo_.find(frame);
	if (!info)
		return;

	info->paramBuffer->_d()->metadata().planes()[0].bytesused = bytesused;
	pipe->param_->queueBuffer(info->paramBuffer);
	pipe->stat_->queueBuffer(info->statBuffer);

	if (info->mainPathBuffer)
		mainPath_->queueBuffer(info->mainPathBuffer);

	if (selfPath_ && info->selfPathBuffer)
		selfPath_->queueBuffer(info->selfPathBuffer);
}

void RkISP1CameraData::setSensorControls([[maybe_unused]] unsigned int frame,
					 const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
}

void RkISP1CameraData::metadataReady(unsigned int frame, const ControlList &metadata)
{
	RkISP1FrameInfo *info = frameInfo_.find(frame);
	if (!info)
		return;

	info->request->metadata().merge(metadata);
	info->metadataProcessed = true;

	pipe()->tryCompleteRequest(info);
}

/* -----------------------------------------------------------------------------
 * Camera Configuration
 */

namespace {

/* Keep in sync with the supported raw formats in rkisp1_path.cpp. */
const std::map<PixelFormat, uint32_t> rawFormats = {
	{ formats::SBGGR8, MEDIA_BUS_FMT_SBGGR8_1X8 },
	{ formats::SGBRG8, MEDIA_BUS_FMT_SGBRG8_1X8 },
	{ formats::SGRBG8, MEDIA_BUS_FMT_SGRBG8_1X8 },
	{ formats::SRGGB8, MEDIA_BUS_FMT_SRGGB8_1X8 },
	{ formats::SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10 },
	{ formats::SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10 },
	{ formats::SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10 },
	{ formats::SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10 },
	{ formats::SBGGR12, MEDIA_BUS_FMT_SBGGR12_1X12 },
	{ formats::SGBRG12, MEDIA_BUS_FMT_SGBRG12_1X12 },
	{ formats::SGRBG12, MEDIA_BUS_FMT_SGRBG12_1X12 },
	{ formats::SRGGB12, MEDIA_BUS_FMT_SRGGB12_1X12 },
};

} /* namespace */

RkISP1CameraConfiguration::RkISP1CameraConfiguration(Camera *camera,
						     RkISP1CameraData *data)
	: CameraConfiguration()
{
	camera_ = camera->shared_from_this();
	data_ = data;
}

bool RkISP1CameraConfiguration::fitsAllPaths(const StreamConfiguration &cfg)
{
	const CameraSensor *sensor = data_->sensor_.get();
	StreamConfiguration config;

	config = cfg;
	if (data_->mainPath_->validate(sensor, sensorConfig, &config) != Valid)
		return false;

	config = cfg;
	if (data_->selfPath_ &&
	    data_->selfPath_->validate(sensor, sensorConfig, &config) != Valid)
		return false;

	return true;
}

CameraConfiguration::Status RkISP1CameraConfiguration::validate()
{
	const PipelineHandlerRkISP1 *pipe = data_->pipe();
	const CameraSensor *sensor = data_->sensor_.get();
	unsigned int pathCount = data_->selfPath_ ? 2 : 1;
	Status status;

	if (config_.empty())
		return Invalid;

	status = validateColorSpaces(ColorSpaceFlag::StreamsShareColorSpace);

	/*
	 * Make sure that if a sensor configuration has been requested it
	 * is valid.
	 */
	if (sensorConfig) {
		if (!sensorConfig->isValid()) {
			LOG(RkISP1, Error)
				<< "Invalid sensor configuration request";

			return Invalid;
		}

		unsigned int bitDepth = sensorConfig->bitDepth;
		if (bitDepth != 8 && bitDepth != 10 && bitDepth != 12) {
			LOG(RkISP1, Error)
				<< "Invalid sensor configuration bit depth";

			return Invalid;
		}
	}

	/* Cap the number of entries to the available streams. */
	if (config_.size() > pathCount) {
		config_.resize(pathCount);
		status = Adjusted;
	}

	Orientation requestedOrientation = orientation;
	combinedTransform_ = data_->sensor_->computeTransform(&orientation);
	if (orientation != requestedOrientation)
		status = Adjusted;

	/*
	 * Simultaneous capture of raw and processed streams isn't possible. If
	 * there is any raw stream, cap the number of streams to one.
	 */
	if (config_.size() > 1) {
		for (const auto &cfg : config_) {
			if (PixelFormatInfo::info(cfg.pixelFormat).colourEncoding ==
			    PixelFormatInfo::ColourEncodingRAW) {
				config_.resize(1);
				status = Adjusted;
				break;
			}
		}
	}

	bool useDewarper = false;
	if (pipe->dewarper_) {
		/*
		 * Platforms with dewarper support, such as i.MX8MP, support
		 * only a single stream. We can inspect config_[0] only here.
		 */
		bool isRaw = PixelFormatInfo::info(config_[0].pixelFormat).colourEncoding ==
			     PixelFormatInfo::ColourEncodingRAW;
		if (!isRaw)
			useDewarper = true;
	}

	/*
	 * If there are more than one stream in the configuration figure out the
	 * order to evaluate the streams. The first stream has the highest
	 * priority but if both main path and self path can satisfy it evaluate
	 * the second stream first as the first stream is guaranteed to work
	 * with whichever path is not used by the second one.
	 */
	std::vector<unsigned int> order(config_.size());
	std::iota(order.begin(), order.end(), 0);
	if (config_.size() == 2 && fitsAllPaths(config_[0]))
		std::reverse(order.begin(), order.end());

	/*
	 * Validate the configuration against the desired path and, if the
	 * platform supports it, the dewarper.
	 */
	auto validateConfig = [&](StreamConfiguration &cfg, RkISP1Path *path,
				  Stream *stream, Status expectedStatus) {
		StreamConfiguration tryCfg = cfg;

		Status ret = path->validate(sensor, sensorConfig, &tryCfg);
		if (ret == Invalid)
			return false;

		if (!useDewarper &&
		    (expectedStatus == Valid && ret == Adjusted))
			return false;

		if (useDewarper) {
			bool adjusted;

			pipe->dewarper_->validateOutput(&tryCfg, &adjusted,
							Converter::Alignment::Down);
			if (expectedStatus == Valid && adjusted)
				return false;
		}

		cfg = tryCfg;
		cfg.setStream(stream);
		return true;
	};

	bool mainPathAvailable = true;
	bool selfPathAvailable = data_->selfPath_;
	RkISP1Path *mainPath = data_->mainPath_;
	RkISP1Path *selfPath = data_->selfPath_;
	Stream *mainPathStream = const_cast<Stream *>(&data_->mainPathStream_);
	Stream *selfPathStream = const_cast<Stream *>(&data_->selfPathStream_);
	for (unsigned int index : order) {
		StreamConfiguration &cfg = config_[index];

		/* Try to match stream without adjusting configuration. */
		if (mainPathAvailable) {
			if (validateConfig(cfg, mainPath, mainPathStream, Valid)) {
				mainPathAvailable = false;
				continue;
			}
		}

		if (selfPathAvailable) {
			if (validateConfig(cfg, selfPath, selfPathStream, Valid)) {
				selfPathAvailable = false;
				continue;
			}
		}

		/* Try to match stream allowing adjusting configuration. */
		if (mainPathAvailable) {
			if (validateConfig(cfg, mainPath, mainPathStream, Adjusted)) {
				mainPathAvailable = false;
				status = Adjusted;
				continue;
			}
		}

		if (selfPathAvailable) {
			if (validateConfig(cfg, selfPath, selfPathStream, Adjusted)) {
				selfPathAvailable = false;
				status = Adjusted;
				continue;
			}
		}

		/* All paths rejected configuration. */
		LOG(RkISP1, Debug) << "Camera configuration not supported "
				   << cfg.toString();
		return Invalid;
	}

	/* Select the sensor format. */
	PixelFormat rawFormat;
	Size maxSize;

	for (const StreamConfiguration &cfg : config_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW)
			rawFormat = cfg.pixelFormat;

		maxSize = std::max(maxSize, cfg.size);
	}

	std::vector<unsigned int> mbusCodes;

	if (rawFormat.isValid()) {
		mbusCodes = { rawFormats.at(rawFormat) };
	} else {
		std::transform(rawFormats.begin(), rawFormats.end(),
			       std::back_inserter(mbusCodes),
			       [](const auto &value) { return value.second; });
	}

	sensorFormat_ = sensor->getFormat(mbusCodes, maxSize,
					  mainPath->maxResolution());

	if (sensorFormat_.size.isNull())
		sensorFormat_.size = sensor->resolution();

	return status;
}

/* -----------------------------------------------------------------------------
 * Pipeline Operations
 */

PipelineHandlerRkISP1::PipelineHandlerRkISP1(CameraManager *manager)
	: PipelineHandler(manager), hasSelfPath_(true), useDewarper_(false)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerRkISP1::generateConfiguration(Camera *camera,
					     Span<const StreamRole> roles)
{
	RkISP1CameraData *data = cameraData(camera);

	unsigned int pathCount = data->selfPath_ ? 2 : 1;
	if (roles.size() > pathCount) {
		LOG(RkISP1, Error) << "Too many stream roles requested";
		return nullptr;
	}

	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<RkISP1CameraConfiguration>(camera, data);
	if (roles.empty())
		return config;

	/*
	 * As the ISP can't output different color spaces for the main and self
	 * path, pick a sensible default color space based on the role of the
	 * first stream and use it for all streams.
	 */
	std::optional<ColorSpace> colorSpace;
	bool mainPathAvailable = true;

	for (const StreamRole role : roles) {
		Size size;

		switch (role) {
		case StreamRole::StillCapture:
			/* JPEG encoders typically expect sYCC. */
			if (!colorSpace)
				colorSpace = ColorSpace::Sycc;

			size = data->sensor_->resolution();
			break;

		case StreamRole::Viewfinder:
			/*
			 * sYCC is the YCbCr encoding of sRGB, which is commonly
			 * used by displays.
			 */
			if (!colorSpace)
				colorSpace = ColorSpace::Sycc;

			size = kRkISP1PreviewSize;
			break;

		case StreamRole::VideoRecording:
			/* Rec. 709 is a good default for HD video recording. */
			if (!colorSpace)
				colorSpace = ColorSpace::Rec709;

			size = kRkISP1PreviewSize;
			break;

		case StreamRole::Raw:
			if (roles.size() > 1) {
				LOG(RkISP1, Error)
					<< "Can't capture both raw and processed streams";
				return nullptr;
			}

			colorSpace = ColorSpace::Raw;
			size = data->sensor_->resolution();
			break;

		default:
			LOG(RkISP1, Warning)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		/*
		 * Prefer the main path if available, as it supports higher
		 * resolutions.
		 *
		 * \todo Using the main path unconditionally hides support for
		 * RGB (only available on the self path) in the streams formats
		 * exposed to applications. This likely calls for a better API
		 * to expose streams capabilities.
		 */
		RkISP1Path *path;
		if (mainPathAvailable) {
			path = data->mainPath_;
			mainPathAvailable = false;
		} else {
			path = data->selfPath_;
		}

		StreamConfiguration cfg =
			path->generateConfiguration(data->sensor_.get(), size, role);
		if (!cfg.pixelFormat.isValid())
			return nullptr;

		cfg.colorSpace = colorSpace;
		config->addConfiguration(cfg);
	}

	config->validate();

	return config;
}

int PipelineHandlerRkISP1::configure(Camera *camera, CameraConfiguration *c)
{
	RkISP1CameraConfiguration *config =
		static_cast<RkISP1CameraConfiguration *>(c);
	RkISP1CameraData *data = cameraData(camera);
	CameraSensor *sensor = data->sensor_.get();
	int ret;

	ret = initLinks(camera, *config);
	if (ret)
		return ret;

	/*
	 * Configure the format on the sensor output and propagate it through
	 * the pipeline.
	 */
	V4L2SubdeviceFormat format = config->sensorFormat();
	LOG(RkISP1, Debug) << "Configuring sensor with " << format;

	if (config->sensorConfig)
		ret = sensor->applyConfiguration(*config->sensorConfig,
						 config->combinedTransform(),
						 &format);
	else
		ret = sensor->setFormat(&format, config->combinedTransform());

	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug) << "Sensor configured with " << format;

	/* Propagate format through the internal media pipeline up to the ISP */
	ret = data->pipe_.configure(sensor, &format);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug) << "Configuring ISP with : " << format;
	ret = isp_->setFormat(0, &format);
	if (ret < 0)
		return ret;

	Rectangle inputCrop(0, 0, format.size);
	ret = isp_->setSelection(0, V4L2_SEL_TGT_CROP, &inputCrop);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "ISP input pad configured with " << format
		<< " crop " << inputCrop;

	Rectangle outputCrop = inputCrop;
	const PixelFormat &streamFormat = config->at(0).pixelFormat;
	const PixelFormatInfo &info = PixelFormatInfo::info(streamFormat);
	isRaw_ = info.colourEncoding == PixelFormatInfo::ColourEncodingRAW;
	useDewarper_ = dewarper_ && !isRaw_;

	/* YUYV8_2X8 is required on the ISP source path pad for YUV output. */
	if (!isRaw_)
		format.code = MEDIA_BUS_FMT_YUYV8_2X8;

	/*
	 * On devices without DUAL_CROP (like the imx8mp) cropping needs to be
	 * done on the ISP/IS output.
	 */
	if (media_->hwRevision() == RKISP1_V_IMX8MP) {
		/* imx8mp has only a single path. */
		const auto &cfg = config->at(0);
		Size ispCrop = format.size.boundedToAspectRatio(cfg.size);
		if (useDewarper_)
			ispCrop = dewarper_->adjustInputSize(cfg.pixelFormat,
							     ispCrop);
		else
			ispCrop.alignUpTo(2, 2);

		outputCrop = ispCrop.centeredTo(Rectangle(format.size).center());
		format.size = ispCrop;
	}

	LOG(RkISP1, Debug)
		<< "Configuring ISP output pad with " << format
		<< " crop " << outputCrop;

	ret = isp_->setSelection(2, V4L2_SEL_TGT_CROP, &outputCrop);
	if (ret < 0)
		return ret;

	format.colorSpace = config->at(0).colorSpace;
	ret = isp_->setFormat(2, &format);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "ISP output pad configured with " << format
		<< " crop " << outputCrop;

	IPACameraSensorInfo sensorInfo;
	ret = data->sensor_->sensorInfo(&sensorInfo);
	if (ret)
		return ret;

	std::map<unsigned int, IPAStream> streamConfig;
	std::vector<std::reference_wrapper<StreamConfiguration>> outputCfgs;

	for (const StreamConfiguration &cfg : *config) {
		if (cfg.stream() == &data->mainPathStream_) {
			ret = mainPath_.configure(cfg, format);
			streamConfig[0] = IPAStream(cfg.pixelFormat,
						    cfg.size);
			/* Configure dewarp */
			if (dewarper_ && !isRaw_) {
				outputCfgs.push_back(const_cast<StreamConfiguration &>(cfg));
				ret = dewarper_->configure(cfg, outputCfgs);
				if (ret)
					return ret;

				/*
				 * Calculate the crop rectangle of the data
				 * flowing into the dewarper in sensor
				 * coordinates.
				 */
				scalerMaxCrop_ =
					outputCrop.transformedBetween(inputCrop,
								      sensorInfo.analogCrop);
			}
		} else if (hasSelfPath_) {
			ret = selfPath_.configure(cfg, format);
			streamConfig[1] = IPAStream(cfg.pixelFormat,
						    cfg.size);
		} else {
			return -ENODEV;
		}

		if (ret)
			return ret;
	}

	V4L2DeviceFormat paramFormat;
	paramFormat.fourcc = V4L2PixelFormat(V4L2_META_FMT_RK_ISP1_EXT_PARAMS);
	ret = param_->setFormat(&paramFormat);
	if (ret)
		return ret;

	V4L2DeviceFormat statFormat;
	statFormat.fourcc = V4L2PixelFormat(V4L2_META_FMT_RK_ISP1_STAT_3A);
	ret = stat_->setFormat(&statFormat);
	if (ret)
		return ret;

	/* Inform IPA of stream configuration and sensor controls. */
	ipa::rkisp1::IPAConfigInfo ipaConfig{ sensorInfo,
					      data->sensor_->controls(),
					      paramFormat.fourcc };

	ret = data->ipa_->configure(ipaConfig, streamConfig, &data->ipaControls_);
	if (ret) {
		LOG(RkISP1, Error) << "failed configuring IPA (" << ret << ")";
		return ret;
	}

	return updateControls(data);
}

int PipelineHandlerRkISP1::exportFrameBuffers([[maybe_unused]] Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	RkISP1CameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->mainPathStream_) {
		/*
		 * Currently, i.MX8MP is the only platform with DW100 dewarper.
		 * It has mainpath and no self path. Hence, export buffers from
		 * dewarper just for the main path stream, for now.
		 */
		if (useDewarper_)
			return dewarper_->exportBuffers(&data->mainPathStream_, count, buffers);
		else
			return mainPath_.exportBuffers(count, buffers);
	} else if (hasSelfPath_ && stream == &data->selfPathStream_) {
		return selfPath_.exportBuffers(count, buffers);
	}

	return -EINVAL;
}

int PipelineHandlerRkISP1::allocateBuffers(Camera *camera)
{
	RkISP1CameraData *data = cameraData(camera);
	unsigned int ipaBufferId = 1;
	int ret;

	unsigned int maxCount = std::max({
		data->mainPathStream_.configuration().bufferCount,
		data->selfPathStream_.configuration().bufferCount,
	});

	if (!isRaw_) {
		ret = param_->allocateBuffers(maxCount, &paramBuffers_);
		if (ret < 0)
			goto error;

		ret = stat_->allocateBuffers(maxCount, &statBuffers_);
		if (ret < 0)
			goto error;
	}

	/* If the dewarper is being used, allocate internal buffers for ISP. */
	if (useDewarper_) {
		ret = mainPath_.exportBuffers(maxCount, &mainPathBuffers_);
		if (ret < 0)
			goto error;

		for (std::unique_ptr<FrameBuffer> &buffer : mainPathBuffers_)
			availableMainPathBuffers_.push(buffer.get());
	}

	for (std::unique_ptr<FrameBuffer> &buffer : paramBuffers_) {
		buffer->setCookie(ipaBufferId++);
		data->ipaBuffers_.emplace_back(buffer->cookie(),
					       buffer->planes());
		availableParamBuffers_.push(buffer.get());
	}

	for (std::unique_ptr<FrameBuffer> &buffer : statBuffers_) {
		buffer->setCookie(ipaBufferId++);
		data->ipaBuffers_.emplace_back(buffer->cookie(),
					       buffer->planes());
		availableStatBuffers_.push(buffer.get());
	}

	data->ipa_->mapBuffers(data->ipaBuffers_);

	return 0;

error:
	paramBuffers_.clear();
	statBuffers_.clear();
	mainPathBuffers_.clear();

	return ret;
}

int PipelineHandlerRkISP1::freeBuffers(Camera *camera)
{
	RkISP1CameraData *data = cameraData(camera);

	while (!availableStatBuffers_.empty())
		availableStatBuffers_.pop();

	while (!availableParamBuffers_.empty())
		availableParamBuffers_.pop();

	while (!availableMainPathBuffers_.empty())
		availableMainPathBuffers_.pop();

	paramBuffers_.clear();
	statBuffers_.clear();
	mainPathBuffers_.clear();

	std::vector<unsigned int> ids;
	for (IPABuffer &ipabuf : data->ipaBuffers_)
		ids.push_back(ipabuf.id);

	data->ipa_->unmapBuffers(ids);
	data->ipaBuffers_.clear();

	if (param_->releaseBuffers())
		LOG(RkISP1, Error) << "Failed to release parameters buffers";

	if (stat_->releaseBuffers())
		LOG(RkISP1, Error) << "Failed to release stat buffers";

	return 0;
}

int PipelineHandlerRkISP1::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	RkISP1CameraData *data = cameraData(camera);
	utils::ScopeExitActions actions;
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers(camera);
	if (ret)
		return ret;
	actions += [&]() { freeBuffers(camera); };

	ret = data->ipa_->start();
	if (ret) {
		LOG(RkISP1, Error)
			<< "Failed to start IPA " << camera->id();
		return ret;
	}
	actions += [&]() { data->ipa_->stop(); };

	data->frame_ = 0;

	if (!isRaw_) {
		ret = param_->streamOn();
		if (ret) {
			LOG(RkISP1, Error)
				<< "Failed to start parameters " << camera->id();
			return ret;
		}
		actions += [&]() { param_->streamOff(); };

		ret = stat_->streamOn();
		if (ret) {
			LOG(RkISP1, Error)
				<< "Failed to start statistics " << camera->id();
			return ret;
		}
		actions += [&]() { stat_->streamOff(); };

		if (useDewarper_) {
			ret = dewarper_->start();
			if (ret) {
				LOG(RkISP1, Error) << "Failed to start dewarper";
				return ret;
			}
			actions += [&]() { dewarper_->stop(); };
		}
	}

	if (data->mainPath_->isEnabled()) {
		ret = mainPath_.start();
		if (ret)
			return ret;
		actions += [&]() { mainPath_.stop(); };
	}

	if (hasSelfPath_ && data->selfPath_->isEnabled()) {
		ret = selfPath_.start();
		if (ret)
			return ret;
	}

	isp_->setFrameStartEnabled(true);

	activeCamera_ = camera;

	actions.release();
	return 0;
}

void PipelineHandlerRkISP1::stopDevice(Camera *camera)
{
	RkISP1CameraData *data = cameraData(camera);
	int ret;

	isp_->setFrameStartEnabled(false);

	data->ipa_->stop();

	if (hasSelfPath_)
		selfPath_.stop();
	mainPath_.stop();

	if (!isRaw_) {
		ret = stat_->streamOff();
		if (ret)
			LOG(RkISP1, Warning)
				<< "Failed to stop statistics for " << camera->id();

		ret = param_->streamOff();
		if (ret)
			LOG(RkISP1, Warning)
				<< "Failed to stop parameters for " << camera->id();

		if (useDewarper_)
			dewarper_->stop();
	}

	ASSERT(data->queuedRequests_.empty());
	data->frameInfo_.clear();

	freeBuffers(camera);

	activeCamera_ = nullptr;
}

int PipelineHandlerRkISP1::queueRequestDevice(Camera *camera, Request *request)
{
	RkISP1CameraData *data = cameraData(camera);

	RkISP1FrameInfo *info = data->frameInfo_.create(data, request, isRaw_);
	if (!info)
		return -ENOENT;

	data->ipa_->queueRequest(data->frame_, request->controls());
	if (isRaw_) {
		if (info->mainPathBuffer)
			data->mainPath_->queueBuffer(info->mainPathBuffer);

		if (data->selfPath_ && info->selfPathBuffer)
			data->selfPath_->queueBuffer(info->selfPathBuffer);
	} else {
		data->ipa_->computeParams(data->frame_,
					  info->paramBuffer->cookie());
	}

	data->frame_++;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

int PipelineHandlerRkISP1::initLinks(Camera *camera,
				     const RkISP1CameraConfiguration &config)
{
	RkISP1CameraData *data = cameraData(camera);
	int ret;

	ret = media_->disableLinks();
	if (ret < 0)
		return ret;

	/*
	 * Configure the sensor links: enable the links corresponding to this
	 * pipeline all the way up to the ISP, through any connected CSI receiver.
	 */
	ret = data->pipe_.initLinks();
	if (ret) {
		LOG(RkISP1, Error) << "Failed to set up pipe links";
		return ret;
	}

	/* Configure the paths after the ISP */
	for (const StreamConfiguration &cfg : config) {
		if (cfg.stream() == &data->mainPathStream_)
			ret = data->mainPath_->setEnabled(true);
		else if (hasSelfPath_ && cfg.stream() == &data->selfPathStream_)
			ret = data->selfPath_->setEnabled(true);
		else
			return -EINVAL;

		if (ret < 0)
			return ret;
	}

	return 0;
}

/**
 * \brief Update the camera controls
 * \param[in] data The camera data
 *
 * Compute the camera controls by calculating controls which the pipeline
 * is reponsible for and merge them with the controls computed by the IPA.
 *
 * This function needs data->ipaControls_ to be refreshed when a new
 * configuration is applied to the camera by the IPA configure() function.
 *
 * Always call this function after IPA configure() to make sure to have a
 * properly refreshed IPA controls list.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineHandlerRkISP1::updateControls(RkISP1CameraData *data)
{
	ControlInfoMap::Map controls;

	if (dewarper_) {
		std::pair<Rectangle, Rectangle> cropLimits;
		if (dewarper_->isConfigured(&data->mainPathStream_))
			cropLimits = dewarper_->inputCropBounds(&data->mainPathStream_);
		else
			cropLimits = dewarper_->inputCropBounds();

		/*
		 * ScalerCrop is specified to be in Sensor coordinates.
		 * So we need to transform the limits to sensor coordinates.
		 * We can safely assume that the maximum crop limit contains the
		 * full fov of the dewarper.
		 */
		Rectangle min = cropLimits.first.transformedBetween(cropLimits.second,
								    scalerMaxCrop_);

		controls[&controls::ScalerCrop] = ControlInfo(min,
							      scalerMaxCrop_,
							      scalerMaxCrop_);
		data->properties_.set(properties::ScalerCropMaximum, scalerMaxCrop_);
		activeCrop_ = scalerMaxCrop_;
	}

	/* Add the IPA registered controls to list of camera controls. */
	for (const auto &ipaControl : data->ipaControls_)
		controls[ipaControl.first] = ipaControl.second;

	data->controlInfo_ = ControlInfoMap(std::move(controls),
					    controls::controls);

	return 0;
}

int PipelineHandlerRkISP1::createCamera(MediaEntity *sensor)
{
	int ret;

	std::unique_ptr<RkISP1CameraData> data =
		std::make_unique<RkISP1CameraData>(this, &mainPath_,
						   hasSelfPath_ ? &selfPath_ : nullptr);

	/* Identify the pipeline path between the sensor and the rkisp1_isp */
	ret = data->pipe_.init(sensor, "rkisp1_isp");
	if (ret) {
		LOG(RkISP1, Error) << "Failed to identify path from sensor to sink";
		return ret;
	}

	data->sensor_ = CameraSensorFactoryBase::create(sensor);
	if (!data->sensor_)
		return -ENODEV;

	/* Initialize the camera properties. */
	data->properties_ = data->sensor_->properties();

	scalerMaxCrop_ = Rectangle(data->sensor_->resolution());

	const CameraSensorProperties::SensorDelays &delays = data->sensor_->sensorDelays();
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
		{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
		{ V4L2_CID_VBLANK, { delays.vblankDelay, false } },
	};

	data->delayedCtrls_ =
		std::make_unique<DelayedControls>(data->sensor_->device(),
						  params);
	isp_->frameStart.connect(data->delayedCtrls_.get(),
				 &DelayedControls::applyControls);

	ret = data->loadIPA(media_->hwRevision());
	if (ret)
		return ret;

	updateControls(data.get());

	std::set<Stream *> streams{
		&data->mainPathStream_,
		&data->selfPathStream_,
	};
	const std::string &id = data->sensor_->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), id, streams);

	registerCamera(std::move(camera));

	return 0;
}

bool PipelineHandlerRkISP1::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("rkisp1");
	dm.add("rkisp1_isp");
	dm.add("rkisp1_resizer_mainpath");
	dm.add("rkisp1_mainpath");
	dm.add("rkisp1_stats");
	dm.add("rkisp1_params");

	media_ = acquireMediaDevice(enumerator, dm);
	if (!media_)
		return false;

	if (!media_->hwRevision()) {
		LOG(RkISP1, Error)
			<< "The rkisp1 driver is too old, v5.11 or newer is required";
		return false;
	}

	hasSelfPath_ = !!media_->getEntityByName("rkisp1_selfpath");

	/* Create the V4L2 subdevices we will need. */
	isp_ = V4L2Subdevice::fromEntityName(media_, "rkisp1_isp");
	if (isp_->open() < 0)
		return false;

	/* Locate and open the stats and params video nodes. */
	stat_ = V4L2VideoDevice::fromEntityName(media_, "rkisp1_stats");
	if (stat_->open() < 0)
		return false;

	param_ = V4L2VideoDevice::fromEntityName(media_, "rkisp1_params");
	if (param_->open() < 0)
		return false;

	/* Locate and open the ISP main and self paths. */
	if (!mainPath_.init(media_))
		return false;

	if (hasSelfPath_ && !selfPath_.init(media_))
		return false;

	mainPath_.bufferReady().connect(this, &PipelineHandlerRkISP1::imageBufferReady);
	if (hasSelfPath_)
		selfPath_.bufferReady().connect(this, &PipelineHandlerRkISP1::imageBufferReady);
	stat_->bufferReady.connect(this, &PipelineHandlerRkISP1::statBufferReady);
	param_->bufferReady.connect(this, &PipelineHandlerRkISP1::paramBufferReady);

	/* If dewarper is present, create its instance. */
	DeviceMatch dwp("dw100");
	dwp.add("dw100-source");
	dwp.add("dw100-sink");

	std::shared_ptr<MediaDevice> dwpMediaDevice = enumerator->search(dwp);
	if (dwpMediaDevice) {
		dewarper_ = std::make_unique<V4L2M2MConverter>(dwpMediaDevice.get());
		if (dewarper_->isValid()) {
			dewarper_->outputBufferReady.connect(
				this, &PipelineHandlerRkISP1::dewarpBufferReady);

			LOG(RkISP1, Info)
				<< "Using DW100 dewarper " << dewarper_->deviceNode();
		} else {
			LOG(RkISP1, Warning)
				<< "Found DW100 dewarper " << dewarper_->deviceNode()
				<< " but invalid";

			dewarper_.reset();
		}
	}

	/*
	 * Enumerate all sensors connected to the ISP and create one
	 * camera instance for each of them.
	 */
	bool registered = false;

	for (MediaEntity *entity : media_->locateEntities(MEDIA_ENT_F_CAM_SENSOR)) {
		LOG(RkISP1, Debug) << "Identified " << entity->name();
		if (!createCamera(entity))
			registered = true;
	}

	return registered;
}

/* -----------------------------------------------------------------------------
 * Buffer Handling
 */

void PipelineHandlerRkISP1::tryCompleteRequest(RkISP1FrameInfo *info)
{
	RkISP1CameraData *data = cameraData(activeCamera_);
	Request *request = info->request;

	if (request->hasPendingBuffers())
		return;

	if (!info->metadataProcessed)
		return;

	if (!isRaw_ && !info->paramDequeued)
		return;

	data->frameInfo_.destroy(info->frame);

	completeRequest(request);
}

void PipelineHandlerRkISP1::imageBufferReady(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	RkISP1CameraData *data = cameraData(activeCamera_);

	RkISP1FrameInfo *info = data->frameInfo_.find(buffer);
	if (!info)
		return;

	const FrameMetadata &metadata = buffer->metadata();
	Request *request = info->request;

	if (metadata.status != FrameMetadata::FrameCancelled) {
		/*
		 * Record the sensor's timestamp in the request metadata.
		 *
		 * \todo The sensor timestamp should be better estimated by connecting
		 * to the V4L2Device::frameStart signal.
		 */
		request->metadata().set(controls::SensorTimestamp,
					metadata.timestamp);

		if (isRaw_) {
			const ControlList &ctrls =
				data->delayedCtrls_->get(metadata.sequence);
			data->ipa_->processStats(info->frame, 0, ctrls);
		}
	} else {
		if (isRaw_)
			info->metadataProcessed = true;
	}

	if (!useDewarper_) {
		completeBuffer(request, buffer);
		tryCompleteRequest(info);

		return;
	}

	/* Do not queue cancelled frames to dewarper. */
	if (metadata.status == FrameMetadata::FrameCancelled) {
		/*
		 * i.MX8MP is the only known platform with dewarper. It has
		 * no self path. Hence, only main path buffer completion is
		 * required.
		 *
		 * Also, we cannot completeBuffer(request, buffer) as buffer
		 * here, is an internal buffer (between ISP and dewarper) and
		 * is not associated to the any specific request. The request
		 * buffer associated with main path stream is the one that
		 * is required to be completed (not the internal buffer).
		 */
		for (auto it : request->buffers()) {
			if (it.first == &data->mainPathStream_)
				completeBuffer(request, it.second);
		}

		tryCompleteRequest(info);
		return;
	}

	/* Handle scaler crop control. */
	const auto &crop = request->controls().get(controls::ScalerCrop);
	if (crop) {
		Rectangle rect = crop.value();

		/*
		 * ScalerCrop is specified to be in Sensor coordinates.
		 * So we need to transform it into dewarper coordinates.
		 * We can safely assume that the maximum crop limit contains the
		 * full fov of the dewarper.
		 */
		std::pair<Rectangle, Rectangle> cropLimits =
			dewarper_->inputCropBounds(&data->mainPathStream_);

		rect = rect.transformedBetween(scalerMaxCrop_, cropLimits.second);
		int ret = dewarper_->setInputCrop(&data->mainPathStream_,
						  &rect);
		rect = rect.transformedBetween(cropLimits.second, scalerMaxCrop_);
		if (!ret && rect != crop.value()) {
			/*
			 * If the rectangle is changed by setInputCrop on the
			 * dewarper, log a debug message and cache the actual
			 * applied rectangle for metadata reporting.
			 */
			LOG(RkISP1, Debug)
				<< "Applied rectangle " << rect.toString()
				<< " differs from requested " << crop.value().toString();
		}

		activeCrop_ = rect;
	}

	/*
	 * Queue input and output buffers to the dewarper. The output
	 * buffers for the dewarper are the buffers of the request, supplied
	 * by the application.
	 */
	int ret = dewarper_->queueBuffers(buffer, request->buffers());
	if (ret < 0)
		LOG(RkISP1, Error) << "Cannot queue buffers to dewarper: "
				   << strerror(-ret);

	request->metadata().set(controls::ScalerCrop, activeCrop_.value());
}

void PipelineHandlerRkISP1::dewarpBufferReady(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	RkISP1CameraData *data = cameraData(activeCamera_);
	Request *request = buffer->request();

	RkISP1FrameInfo *info = data->frameInfo_.find(buffer->request());
	if (!info)
		return;

	completeBuffer(request, buffer);
	tryCompleteRequest(info);
}

void PipelineHandlerRkISP1::paramBufferReady(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	RkISP1CameraData *data = cameraData(activeCamera_);

	RkISP1FrameInfo *info = data->frameInfo_.find(buffer);
	if (!info)
		return;

	info->paramDequeued = true;
	tryCompleteRequest(info);
}

void PipelineHandlerRkISP1::statBufferReady(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	RkISP1CameraData *data = cameraData(activeCamera_);

	RkISP1FrameInfo *info = data->frameInfo_.find(buffer);
	if (!info)
		return;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		info->metadataProcessed = true;
		tryCompleteRequest(info);
		return;
	}

	if (data->frame_ <= buffer->metadata().sequence)
		data->frame_ = buffer->metadata().sequence + 1;

	data->ipa_->processStats(info->frame, info->statBuffer->cookie(),
				 data->delayedCtrls_->get(buffer->metadata().sequence));
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerRkISP1, "rkisp1")

} /* namespace libcamera */
