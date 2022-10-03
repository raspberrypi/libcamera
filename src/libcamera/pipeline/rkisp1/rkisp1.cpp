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
#include <numeric>
#include <queue>

#include <linux/media-bus-format.h>
#include <linux/rkisp1-config.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/color_space.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/rkisp1_ipa_interface.h>
#include <libcamera/ipa/rkisp1_ipa_proxy.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
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

private:
	void paramFilled(unsigned int frame);
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
};

class PipelineHandlerRkISP1 : public PipelineHandler
{
public:
	PipelineHandlerRkISP1(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
		const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	RkISP1CameraData *cameraData(Camera *camera)
	{
		return static_cast<RkISP1CameraData *>(camera->_d());
	}

	friend RkISP1CameraData;
	friend RkISP1Frames;

	int initLinks(Camera *camera, const CameraSensor *sensor,
		      const RkISP1CameraConfiguration &config);
	int createCamera(MediaEntity *sensor);
	void tryCompleteRequest(RkISP1FrameInfo *info);
	void bufferReady(FrameBuffer *buffer);
	void paramReady(FrameBuffer *buffer);
	void statReady(FrameBuffer *buffer);
	void frameStart(uint32_t sequence);

	int allocateBuffers(Camera *camera);
	int freeBuffers(Camera *camera);

	MediaDevice *media_;
	std::unique_ptr<V4L2Subdevice> isp_;
	std::unique_ptr<V4L2VideoDevice> param_;
	std::unique_ptr<V4L2VideoDevice> stat_;
	std::unique_ptr<V4L2Subdevice> csi_;

	bool hasSelfPath_;
	bool isRaw_;

	RkISP1MainPath mainPath_;
	RkISP1SelfPath selfPath_;

	std::vector<std::unique_ptr<FrameBuffer>> paramBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statBuffers_;
	std::queue<FrameBuffer *> availableParamBuffers_;
	std::queue<FrameBuffer *> availableStatBuffers_;

	Camera *activeCamera_;

	const MediaPad *ispSink_;
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

	if (!isRaw) {
		if (pipe_->availableParamBuffers_.empty()) {
			LOG(RkISP1, Error) << "Parameters buffer underrun";
			return nullptr;
		}

		if (pipe_->availableStatBuffers_.empty()) {
			LOG(RkISP1, Error) << "Statisitc buffer underrun";
			return nullptr;
		}

		paramBuffer = pipe_->availableParamBuffers_.front();
		pipe_->availableParamBuffers_.pop();

		statBuffer = pipe_->availableStatBuffers_.front();
		pipe_->availableStatBuffers_.pop();
	}

	FrameBuffer *mainPathBuffer = request->findBuffer(&data->mainPathStream_);
	FrameBuffer *selfPathBuffer = request->findBuffer(&data->selfPathStream_);

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

int RkISP1CameraData::loadIPA(unsigned int hwRevision)
{
	ipa_ = IPAManager::createIPA<ipa::rkisp1::IPAProxyRkISP1>(pipe(), 1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->setSensorControls.connect(this, &RkISP1CameraData::setSensorControls);
	ipa_->paramsBufferReady.connect(this, &RkISP1CameraData::paramFilled);
	ipa_->metadataReady.connect(this, &RkISP1CameraData::metadataReady);

	/*
	 * The API tuning file is made from the sensor name unless the
	 * environment variable overrides it. If
	 */
	std::string ipaTuningFile;
	char const *configFromEnv = utils::secure_getenv("LIBCAMERA_RKISP1_TUNING_FILE");
	if (!configFromEnv || *configFromEnv == '\0') {
		ipaTuningFile = ipa_->configurationFile(sensor_->model() + ".yaml");
		/*
		 * If the tuning file isn't found, fall back to the
		 * 'uncalibrated' configuration file.
		 */
		if (ipaTuningFile.empty())
			ipaTuningFile = ipa_->configurationFile("uncalibrated.yaml");
	} else {
		ipaTuningFile = std::string(configFromEnv);
	}

	IPACameraSensorInfo sensorInfo{};
	int ret = sensor_->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(RkISP1, Error) << "Camera sensor information not available";
		return ret;
	}

	ret = ipa_->init({ ipaTuningFile, sensor_->model() }, hwRevision,
			 sensorInfo, sensor_->controls(), &controlInfo_);
	if (ret < 0) {
		LOG(RkISP1, Error) << "IPA initialization failure";
		return ret;
	}

	return 0;
}

void RkISP1CameraData::paramFilled(unsigned int frame)
{
	PipelineHandlerRkISP1 *pipe = RkISP1CameraData::pipe();
	RkISP1FrameInfo *info = frameInfo_.find(frame);
	if (!info)
		return;

	info->paramBuffer->_d()->metadata().planes()[0].bytesused =
		sizeof(struct rkisp1_params_cfg);
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
	if (data_->mainPath_->validate(sensor, &config) != Valid)
		return false;

	config = cfg;
	if (data_->selfPath_ && data_->selfPath_->validate(sensor, &config) != Valid)
		return false;

	return true;
}

CameraConfiguration::Status RkISP1CameraConfiguration::validate()
{
	const CameraSensor *sensor = data_->sensor_.get();
	unsigned int pathCount = data_->selfPath_ ? 2 : 1;
	Status status;

	if (config_.empty())
		return Invalid;

	status = validateColorSpaces(ColorSpaceFlag::StreamsShareColorSpace);

	if (transform != Transform::Identity) {
		transform = Transform::Identity;
		status = Adjusted;
	}

	/* Cap the number of entries to the available streams. */
	if (config_.size() > pathCount) {
		config_.resize(pathCount);
		status = Adjusted;
	}

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

	bool mainPathAvailable = true;
	bool selfPathAvailable = data_->selfPath_;
	for (unsigned int index : order) {
		StreamConfiguration &cfg = config_[index];

		/* Try to match stream without adjusting configuration. */
		if (mainPathAvailable) {
			StreamConfiguration tryCfg = cfg;
			if (data_->mainPath_->validate(sensor, &tryCfg) == Valid) {
				mainPathAvailable = false;
				cfg = tryCfg;
				cfg.setStream(const_cast<Stream *>(&data_->mainPathStream_));
				continue;
			}
		}

		if (selfPathAvailable) {
			StreamConfiguration tryCfg = cfg;
			if (data_->selfPath_->validate(sensor, &tryCfg) == Valid) {
				selfPathAvailable = false;
				cfg = tryCfg;
				cfg.setStream(const_cast<Stream *>(&data_->selfPathStream_));
				continue;
			}
		}

		/* Try to match stream allowing adjusting configuration. */
		if (mainPathAvailable) {
			StreamConfiguration tryCfg = cfg;
			if (data_->mainPath_->validate(sensor, &tryCfg) == Adjusted) {
				mainPathAvailable = false;
				cfg = tryCfg;
				cfg.setStream(const_cast<Stream *>(&data_->mainPathStream_));
				status = Adjusted;
				continue;
			}
		}

		if (selfPathAvailable) {
			StreamConfiguration tryCfg = cfg;
			if (data_->selfPath_->validate(sensor, &tryCfg) == Adjusted) {
				selfPathAvailable = false;
				cfg = tryCfg;
				cfg.setStream(const_cast<Stream *>(&data_->selfPathStream_));
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

	sensorFormat_ = sensor->getFormat(mbusCodes, maxSize);

	if (sensorFormat_.size.isNull())
		sensorFormat_.size = sensor->resolution();

	return status;
}

/* -----------------------------------------------------------------------------
 * Pipeline Operations
 */

PipelineHandlerRkISP1::PipelineHandlerRkISP1(CameraManager *manager)
	: PipelineHandler(manager), hasSelfPath_(true)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerRkISP1::generateConfiguration(Camera *camera,
	const StreamRoles &roles)
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
	bool selfPathAvailable = data->selfPath_;

	for (const StreamRole role : roles) {
		bool useMainPath;

		switch (role) {
		case StreamRole::StillCapture:
			useMainPath = mainPathAvailable;
			/* JPEG encoders typically expect sYCC. */
			if (!colorSpace)
				colorSpace = ColorSpace::Sycc;
			break;

		case StreamRole::Viewfinder:
			useMainPath = !selfPathAvailable;
			/*
			 * sYCC is the YCbCr encoding of sRGB, which is commonly
			 * used by displays.
			 */
			if (!colorSpace)
				colorSpace = ColorSpace::Sycc;
			break;

		case StreamRole::VideoRecording:
			useMainPath = !selfPathAvailable;
			/* Rec. 709 is a good default for HD video recording. */
			if (!colorSpace)
				colorSpace = ColorSpace::Rec709;
			break;

		case StreamRole::Raw:
			if (roles.size() > 1) {
				LOG(RkISP1, Error)
					<< "Can't capture both raw and processed streams";
				return nullptr;
			}

			useMainPath = true;
			colorSpace = ColorSpace::Raw;
			break;

		default:
			LOG(RkISP1, Warning)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		RkISP1Path *path;

		if (useMainPath) {
			path = data->mainPath_;
			mainPathAvailable = false;
		} else {
			path = data->selfPath_;
			selfPathAvailable = false;
		}

		StreamConfiguration cfg =
			path->generateConfiguration(data->sensor_.get(), role);
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

	ret = initLinks(camera, sensor, *config);
	if (ret)
		return ret;

	/*
	 * Configure the format on the sensor output and propagate it through
	 * the pipeline.
	 */
	V4L2SubdeviceFormat format = config->sensorFormat();
	LOG(RkISP1, Debug) << "Configuring sensor with " << format;

	ret = sensor->setFormat(&format);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug) << "Sensor configured with " << format;

	if (csi_) {
		ret = csi_->setFormat(0, &format);
		if (ret < 0)
			return ret;
	}

	ret = isp_->setFormat(0, &format);
	if (ret < 0)
		return ret;

	Rectangle rect(0, 0, format.size);
	ret = isp_->setSelection(0, V4L2_SEL_TGT_CROP, &rect);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "ISP input pad configured with " << format
		<< " crop " << rect;

	const PixelFormat &streamFormat = config->at(0).pixelFormat;
	const PixelFormatInfo &info = PixelFormatInfo::info(streamFormat);
	isRaw_ = info.colourEncoding == PixelFormatInfo::ColourEncodingRAW;

	/* YUYV8_2X8 is required on the ISP source path pad for YUV output. */
	if (!isRaw_)
		format.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8;

	LOG(RkISP1, Debug)
		<< "Configuring ISP output pad with " << format
		<< " crop " << rect;

	ret = isp_->setSelection(2, V4L2_SEL_TGT_CROP, &rect);
	if (ret < 0)
		return ret;

	format.colorSpace = config->at(0).colorSpace;
	ret = isp_->setFormat(2, &format);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "ISP output pad configured with " << format
		<< " crop " << rect;

	std::map<unsigned int, IPAStream> streamConfig;

	for (const StreamConfiguration &cfg : *config) {
		if (cfg.stream() == &data->mainPathStream_) {
			ret = mainPath_.configure(cfg, format);
			streamConfig[0] = IPAStream(cfg.pixelFormat,
						    cfg.size);
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
	paramFormat.fourcc = V4L2PixelFormat(V4L2_META_FMT_RK_ISP1_PARAMS);
	ret = param_->setFormat(&paramFormat);
	if (ret)
		return ret;

	V4L2DeviceFormat statFormat;
	statFormat.fourcc = V4L2PixelFormat(V4L2_META_FMT_RK_ISP1_STAT_3A);
	ret = stat_->setFormat(&statFormat);
	if (ret)
		return ret;

	/* Inform IPA of stream configuration and sensor controls. */
	ipa::rkisp1::IPAConfigInfo ipaConfig{};

	ret = data->sensor_->sensorInfo(&ipaConfig.sensorInfo);
	if (ret)
		return ret;

	ipaConfig.sensorControls = data->sensor_->controls();

	ret = data->ipa_->configure(ipaConfig, streamConfig, &data->controlInfo_);
	if (ret) {
		LOG(RkISP1, Error) << "failed configuring IPA (" << ret << ")";
		return ret;
	}
	return 0;
}

int PipelineHandlerRkISP1::exportFrameBuffers([[maybe_unused]] Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	RkISP1CameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->mainPathStream_)
		return mainPath_.exportBuffers(count, buffers);
	else if (hasSelfPath_ && stream == &data->selfPathStream_)
		return selfPath_.exportBuffers(count, buffers);

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

	return ret;
}

int PipelineHandlerRkISP1::freeBuffers(Camera *camera)
{
	RkISP1CameraData *data = cameraData(camera);

	while (!availableStatBuffers_.empty())
		availableStatBuffers_.pop();

	while (!availableParamBuffers_.empty())
		availableParamBuffers_.pop();

	paramBuffers_.clear();
	statBuffers_.clear();

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
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers(camera);
	if (ret)
		return ret;

	ret = data->ipa_->start();
	if (ret) {
		freeBuffers(camera);
		LOG(RkISP1, Error)
			<< "Failed to start IPA " << camera->id();
		return ret;
	}

	data->frame_ = 0;

	if (!isRaw_) {
		ret = param_->streamOn();
		if (ret) {
			data->ipa_->stop();
			freeBuffers(camera);
			LOG(RkISP1, Error)
				<< "Failed to start parameters " << camera->id();
			return ret;
		}

		ret = stat_->streamOn();
		if (ret) {
			param_->streamOff();
			data->ipa_->stop();
			freeBuffers(camera);
			LOG(RkISP1, Error)
				<< "Failed to start statistics " << camera->id();
			return ret;
		}
	}

	if (data->mainPath_->isEnabled()) {
		ret = mainPath_.start();
		if (ret) {
			param_->streamOff();
			stat_->streamOff();
			data->ipa_->stop();
			freeBuffers(camera);
			return ret;
		}
	}

	if (hasSelfPath_ && data->selfPath_->isEnabled()) {
		ret = selfPath_.start();
		if (ret) {
			mainPath_.stop();
			param_->streamOff();
			stat_->streamOff();
			data->ipa_->stop();
			freeBuffers(camera);
			return ret;
		}
	}

	isp_->setFrameStartEnabled(true);

	activeCamera_ = camera;
	return ret;
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
		data->ipa_->fillParamsBuffer(data->frame_,
					     info->paramBuffer->cookie());
	}

	data->frame_++;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

int PipelineHandlerRkISP1::initLinks(Camera *camera,
				     const CameraSensor *sensor,
				     const RkISP1CameraConfiguration &config)
{
	RkISP1CameraData *data = cameraData(camera);
	int ret;

	ret = media_->disableLinks();
	if (ret < 0)
		return ret;

	/*
	 * Configure the sensor links: enable the link corresponding to this
	 * camera.
	 */
	for (MediaLink *link : ispSink_->links()) {
		if (link->source()->entity() != sensor->entity())
			continue;

		LOG(RkISP1, Debug)
			<< "Enabling link from sensor '"
			<< link->source()->entity()->name()
			<< "' to ISP";

		ret = link->setEnabled(true);
		if (ret < 0)
			return ret;
	}

	if (csi_) {
		MediaLink *link = isp_->entity()->getPadByIndex(0)->links().at(0);

		ret = link->setEnabled(true);
		if (ret < 0)
			return ret;
	}

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

int PipelineHandlerRkISP1::createCamera(MediaEntity *sensor)
{
	int ret;

	std::unique_ptr<RkISP1CameraData> data =
		std::make_unique<RkISP1CameraData>(this, &mainPath_,
						   hasSelfPath_ ? &selfPath_ : nullptr);

	data->sensor_ = std::make_unique<CameraSensor>(sensor);
	ret = data->sensor_->init();
	if (ret)
		return ret;

	/* Initialize the camera properties. */
	data->properties_ = data->sensor_->properties();

	/*
	 * \todo Read dealy values from the sensor itself or from a
	 * a sensor database. For now use generic values taken from
	 * the Raspberry Pi and listed as generic values.
	 */
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { 1, false } },
		{ V4L2_CID_EXPOSURE, { 2, false } },
	};

	data->delayedCtrls_ =
		std::make_unique<DelayedControls>(data->sensor_->device(),
						  params);
	isp_->frameStart.connect(data->delayedCtrls_.get(),
				 &DelayedControls::applyControls);

	ret = data->loadIPA(media_->hwRevision());
	if (ret)
		return ret;

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
	const MediaPad *pad;

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

	/* Locate and open the optional CSI-2 receiver. */
	ispSink_ = isp_->entity()->getPadByIndex(0);
	if (!ispSink_ || ispSink_->links().empty())
		return false;

	pad = ispSink_->links().at(0)->source();
	if (pad->entity()->function() == MEDIA_ENT_F_VID_IF_BRIDGE) {
		csi_ = std::make_unique<V4L2Subdevice>(pad->entity());
		if (csi_->open() < 0)
			return false;

		ispSink_ = csi_->entity()->getPadByIndex(0);
		if (!ispSink_)
			return false;
	}

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

	mainPath_.bufferReady().connect(this, &PipelineHandlerRkISP1::bufferReady);
	if (hasSelfPath_)
		selfPath_.bufferReady().connect(this, &PipelineHandlerRkISP1::bufferReady);
	stat_->bufferReady.connect(this, &PipelineHandlerRkISP1::statReady);
	param_->bufferReady.connect(this, &PipelineHandlerRkISP1::paramReady);

	/*
	 * Enumerate all sensors connected to the ISP and create one
	 * camera instance for each of them.
	 */
	bool registered = false;
	for (MediaLink *link : ispSink_->links()) {
		if (!createCamera(link->source()->entity()))
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

void PipelineHandlerRkISP1::bufferReady(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	RkISP1CameraData *data = cameraData(activeCamera_);

	RkISP1FrameInfo *info = data->frameInfo_.find(buffer);
	if (!info)
		return;

	const FrameMetadata &metadata = buffer->metadata();
	Request *request = buffer->request();

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
			data->ipa_->processStatsBuffer(info->frame, 0, ctrls);
		}
	} else {
		if (isRaw_)
			info->metadataProcessed = true;
	}

	completeBuffer(request, buffer);
	tryCompleteRequest(info);
}

void PipelineHandlerRkISP1::paramReady(FrameBuffer *buffer)
{
	ASSERT(activeCamera_);
	RkISP1CameraData *data = cameraData(activeCamera_);

	RkISP1FrameInfo *info = data->frameInfo_.find(buffer);
	if (!info)
		return;

	info->paramDequeued = true;
	tryCompleteRequest(info);
}

void PipelineHandlerRkISP1::statReady(FrameBuffer *buffer)
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

	data->ipa_->processStatsBuffer(info->frame, info->statBuffer->cookie(),
				       data->delayedCtrls_->get(buffer->metadata().sequence));
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerRkISP1)

} /* namespace libcamera */
