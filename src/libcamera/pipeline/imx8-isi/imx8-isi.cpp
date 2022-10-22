/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022 - Jacopo Mondi <jacopo@jmondi.org>
 *
 * imx8-isi.cpp - Pipeline handler for ISI interface found on NXP i.MX8 SoC
 */

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera_manager.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "linux/media-bus-format.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISI)

class PipelineHandlerISI;

class ISICameraData : public Camera::Private
{
public:
	ISICameraData(PipelineHandler *ph)
		: Camera::Private(ph)
	{
		/*
		 * \todo Assume 2 channels only for now, as that's the number of
		 * available channels on i.MX8MP.
		 */
		streams_.resize(2);
	}

	PipelineHandlerISI *pipe();

	int init();

	unsigned int pipeIndex(const Stream *stream)
	{
		return stream - &*streams_.begin();
	}

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<V4L2Subdevice> csis_;

	std::vector<Stream> streams_;

	std::vector<Stream *> enabledStreams_;

	unsigned int xbarSink_;
};

class ISICameraConfiguration : public CameraConfiguration
{
public:
	/*
	 * formatsMap_ records the association between an output pixel format
	 * and the combination of V4L2 pixel format and media bus codes that have
	 * to be applied to the pipeline.
	 */
	struct PipeFormat {
		unsigned int isiCode;
		unsigned int sensorCode;
	};

	using FormatMap = std::map<PixelFormat, PipeFormat>;

	ISICameraConfiguration(ISICameraData *data)
		: data_(data)
	{
	}

	Status validate() override;

	static const FormatMap formatsMap_;

	V4L2SubdeviceFormat sensorFormat_;

private:
	CameraConfiguration::Status
	validateRaw(std::set<Stream *> &availableStreams, const Size &maxResolution);
	CameraConfiguration::Status
	validateYuv(std::set<Stream *> &availableStreams, const Size &maxResolution);

	const ISICameraData *data_;
};

class PipelineHandlerISI : public PipelineHandler
{
public:
	PipelineHandlerISI(CameraManager *manager);

	bool match(DeviceEnumerator *enumerator) override;

	std::unique_ptr<CameraConfiguration>
	generateConfiguration(Camera *camera, const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;

protected:
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

private:
	static constexpr Size kPreviewSize = { 1920, 1080 };
	static constexpr Size kMinISISize = { 1, 1 };

	struct Pipe {
		std::unique_ptr<V4L2Subdevice> isi;
		std::unique_ptr<V4L2VideoDevice> capture;
	};

	ISICameraData *cameraData(Camera *camera)
	{
		return static_cast<ISICameraData *>(camera->_d());
	}

	Pipe *pipeFromStream(Camera *camera, const Stream *stream);

	void bufferReady(FrameBuffer *buffer);

	MediaDevice *isiDev_;

	std::unique_ptr<V4L2Subdevice> crossbar_;
	std::vector<Pipe> pipes_;
};

/* -----------------------------------------------------------------------------
 * Camera Data
 */

PipelineHandlerISI *ISICameraData::pipe()
{
	return static_cast<PipelineHandlerISI *>(Camera::Private::pipe());
}

/* Open and initialize pipe components. */
int ISICameraData::init()
{
	int ret = sensor_->init();
	if (ret)
		return ret;

	ret = csis_->open();
	if (ret)
		return ret;

	properties_ = sensor_->properties();

	return 0;
}

/* -----------------------------------------------------------------------------
 * Camera Configuration
 */

/**
 * \todo Do not associate the sensor format to non-RAW pixelformats, as
 * the ISI can do colorspace conversion.
 */
const ISICameraConfiguration::FormatMap ISICameraConfiguration::formatsMap_ = {
	{
		formats::YUYV,
		{ MEDIA_BUS_FMT_YUV8_1X24,
		  MEDIA_BUS_FMT_UYVY8_1X16 },
	},
	{
		formats::AVUY8888,
		{ MEDIA_BUS_FMT_YUV8_1X24,
		  MEDIA_BUS_FMT_UYVY8_1X16 },
	},
	{
		formats::NV12,
		{ MEDIA_BUS_FMT_YUV8_1X24,
		  MEDIA_BUS_FMT_UYVY8_1X16 },
	},
	{
		formats::NV16,
		{ MEDIA_BUS_FMT_YUV8_1X24,
		  MEDIA_BUS_FMT_UYVY8_1X16 },
	},
	{
		formats::YUV444,
		{ MEDIA_BUS_FMT_YUV8_1X24,
		  MEDIA_BUS_FMT_UYVY8_1X16 },
	},
	{
		formats::RGB565,
		{ MEDIA_BUS_FMT_RGB888_1X24,
		  MEDIA_BUS_FMT_RGB565_1X16 },
	},
	{
		formats::BGR888,
		{ MEDIA_BUS_FMT_RGB888_1X24,
		  MEDIA_BUS_FMT_RGB565_1X16 },
	},
	{
		formats::RGB888,
		{ MEDIA_BUS_FMT_RGB888_1X24,
		  MEDIA_BUS_FMT_RGB565_1X16 },
	},
	{
		formats::XRGB8888,
		{ MEDIA_BUS_FMT_RGB888_1X24,
		  MEDIA_BUS_FMT_RGB565_1X16 },
	},
	{
		formats::ABGR8888,
		{ MEDIA_BUS_FMT_RGB888_1X24,
		  MEDIA_BUS_FMT_RGB565_1X16 },
	},
	{
		formats::SBGGR8,
		{ MEDIA_BUS_FMT_SBGGR8_1X8,
		  MEDIA_BUS_FMT_SBGGR8_1X8 },
	},
	{
		formats::SGBRG8,
		{ MEDIA_BUS_FMT_SGBRG8_1X8,
		  MEDIA_BUS_FMT_SGBRG8_1X8 },
	},
	{
		formats::SGRBG8,
		{ MEDIA_BUS_FMT_SGRBG8_1X8,
		  MEDIA_BUS_FMT_SGRBG8_1X8 },
	},
	{
		formats::SRGGB8,
		{ MEDIA_BUS_FMT_SRGGB8_1X8,
		  MEDIA_BUS_FMT_SRGGB8_1X8 },
	},
	{
		formats::SBGGR10,
		{ MEDIA_BUS_FMT_SBGGR10_1X10,
		  MEDIA_BUS_FMT_SBGGR10_1X10 },
	},
	{
		formats::SGBRG10,
		{ MEDIA_BUS_FMT_SGBRG10_1X10,
		  MEDIA_BUS_FMT_SGBRG10_1X10 },
	},
	{
		formats::SGRBG10,
		{ MEDIA_BUS_FMT_SGRBG10_1X10,
		  MEDIA_BUS_FMT_SGRBG10_1X10 },
	},
	{
		formats::SRGGB10,
		{ MEDIA_BUS_FMT_SRGGB10_1X10,
		  MEDIA_BUS_FMT_SRGGB10_1X10 },
	},
	{
		formats::SBGGR12,
		{ MEDIA_BUS_FMT_SBGGR12_1X12,
		  MEDIA_BUS_FMT_SBGGR12_1X12 },
	},
	{
		formats::SGBRG12,
		{ MEDIA_BUS_FMT_SGBRG12_1X12,
		  MEDIA_BUS_FMT_SGBRG12_1X12 },
	},
	{
		formats::SGRBG12,
		{ MEDIA_BUS_FMT_SGRBG12_1X12,
		  MEDIA_BUS_FMT_SGRBG12_1X12 },
	},
	{
		formats::SRGGB12,
		{ MEDIA_BUS_FMT_SRGGB12_1X12,
		  MEDIA_BUS_FMT_SRGGB12_1X12 },
	},
};

/*
 * Adjust stream configuration when the first requested stream is RAW: all the
 * streams will have the same RAW pixelformat and size.
 */
CameraConfiguration::Status
ISICameraConfiguration::validateRaw(std::set<Stream *> &availableStreams,
				    const Size &maxResolution)
{
	CameraConfiguration::Status status = Valid;

	/*
	 * Make sure the requested RAW format is supported by the
	 * pipeline, otherwise adjust it.
	 */
	std::vector<unsigned int> mbusCodes = data_->sensor_->mbusCodes();
	StreamConfiguration &rawConfig = config_[0];

	bool supported = false;
	auto it = formatsMap_.find(rawConfig.pixelFormat);
	if (it != formatsMap_.end()) {
		unsigned int mbusCode = it->second.sensorCode;

		if (std::count(mbusCodes.begin(), mbusCodes.end(), mbusCode))
			supported = true;
	}

	if (!supported) {
		/*
		 * Adjust to the first mbus code supported by both the
		 * sensor and the pipeline.
		 */
		const FormatMap::value_type *pipeConfig = nullptr;
		for (unsigned int code : mbusCodes) {
			const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(code);
			if (!bayerFormat.isValid())
				continue;

			auto fmt = std::find_if(ISICameraConfiguration::formatsMap_.begin(),
						ISICameraConfiguration::formatsMap_.end(),
						[code](const auto &isiFormat) {
							const auto &pipe = isiFormat.second;
							return pipe.sensorCode == code;
						});

			if (fmt == ISICameraConfiguration::formatsMap_.end())
				continue;

			pipeConfig = &(*fmt);
			break;
		}

		if (!pipeConfig) {
			LOG(ISI, Error) << "Cannot adjust RAW format "
					<< rawConfig.pixelFormat;
			return Invalid;
		}

		rawConfig.pixelFormat = pipeConfig->first;
		LOG(ISI, Debug) << "RAW pixelformat adjusted to "
				<< pipeConfig->first;
		status = Adjusted;
	}

	/* Cap the RAW stream size to the maximum resolution. */
	const Size configSize = rawConfig.size;
	rawConfig.size.boundTo(maxResolution);
	if (rawConfig.size != configSize) {
		LOG(ISI, Debug) << "RAW size adjusted to "
				<< rawConfig.size;
		status = Adjusted;
	}

	/* Adjust all other streams to RAW. */
	for (const auto &[i, cfg] : utils::enumerate(config_)) {

		LOG(ISI, Debug) << "Stream " << i << ": " << cfg.toString();
		const PixelFormat pixFmt = cfg.pixelFormat;
		const Size size = cfg.size;

		cfg.pixelFormat = rawConfig.pixelFormat;
		cfg.size = rawConfig.size;

		if (cfg.pixelFormat != pixFmt || cfg.size != size) {
			LOG(ISI, Debug) << "Stream " << i << " adjusted to "
					<< cfg.toString();
			status = Adjusted;
		}

		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
		cfg.stride = info.stride(cfg.size.width, 0);
		cfg.frameSize = info.frameSize(cfg.size, info.bitsPerPixel);

		/* Assign streams in the order they are presented. */
		auto stream = availableStreams.extract(availableStreams.begin());
		cfg.setStream(stream.value());
	}

	return status;
}

/*
 * Adjust stream configuration when the first requested stream is not RAW: all
 * the streams will be either YUV or RGB processed formats.
 */
CameraConfiguration::Status
ISICameraConfiguration::validateYuv(std::set<Stream *> &availableStreams,
				    const Size &maxResolution)
{
	CameraConfiguration::Status status = Valid;

	for (const auto &[i, cfg] : utils::enumerate(config_)) {

		LOG(ISI, Debug) << "Stream " << i << ": " << cfg.toString();

		/* If the stream is RAW or not supported default it to YUYV. */
		const PixelFormatInfo &cfgInfo = PixelFormatInfo::info(cfg.pixelFormat);
		if (cfgInfo.colourEncoding == PixelFormatInfo::ColourEncodingRAW ||
		    !formatsMap_.count(cfg.pixelFormat)) {

			LOG(ISI, Debug) << "Stream " << i << " format: "
					<< cfg.pixelFormat << " adjusted to YUYV";

			cfg.pixelFormat = formats::YUYV;
			status = Adjusted;
		}

		/* Cap the streams size to the maximum accepted resolution. */
		Size configSize = cfg.size;
		cfg.size.boundTo(maxResolution);
		if (cfg.size != configSize) {
			LOG(ISI, Debug)
				<< "Stream " << i << " adjusted to " << cfg.size;
			status = Adjusted;
		}

		/* Re-fetch the pixel format info in case it has been adjusted. */
		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);

		/* \todo Multiplane ? */
		cfg.stride = info.stride(cfg.size.width, 0);
		cfg.frameSize = info.frameSize(cfg.size, info.bitsPerPixel);

		/* Assign streams in the order they are presented. */
		auto stream = availableStreams.extract(availableStreams.begin());
		cfg.setStream(stream.value());
	}

	return status;
}

CameraConfiguration::Status ISICameraConfiguration::validate()
{
	Status status = Valid;

	std::set<Stream *> availableStreams;
	std::transform(data_->streams_.begin(), data_->streams_.end(),
		       std::inserter(availableStreams, availableStreams.end()),
		       [](const Stream &s) { return const_cast<Stream *>(&s); });

	if (config_.empty())
		return Invalid;

	/* Cap the number of streams to the number of available ISI pipes. */
	if (config_.size() > availableStreams.size()) {
		config_.resize(availableStreams.size());
		status = Adjusted;
	}

	/*
	 * If more than a single stream is requested, the maximum allowed input
	 * image width is 2048. Cap the maximum image size accordingly.
	 *
	 * \todo The (size > 1) check only applies to i.MX8MP which has 2 ISI
	 * channels. SoCs with more channels than the i.MX8MP are capable of
	 * supporting more streams with input width > 2048 by chaining
	 * successive channels together. Define a policy for channels allocation
	 * to fully support other SoCs.
	 */
	CameraSensor *sensor = data_->sensor_.get();
	Size maxResolution = sensor->resolution();
	if (config_.size() > 1)
		maxResolution.width = std::min(2048U, maxResolution.width);

	/* Validate streams according to the format of the first one. */
	const PixelFormatInfo info = PixelFormatInfo::info(config_[0].pixelFormat);

	Status validationStatus;
	if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW)
		validationStatus = validateRaw(availableStreams, maxResolution);
	else
		validationStatus = validateYuv(availableStreams, maxResolution);

	if (validationStatus == Invalid)
		return Invalid;

	if (validationStatus == Adjusted)
		status = Adjusted;

	/*
	 * Sensor format selection policy: the first stream selects the media
	 * bus code to use, the largest stream selects the size.
	 *
	 * \todo The sensor format selection policy could be changed to
	 * prefer operating the sensor at full resolution to prioritize
	 * image quality in exchange of a usually slower frame rate.
	 * Usage of the STILL_CAPTURE role could be consider for this.
	 */
	const PipeFormat &pipeFmt = formatsMap_.at(config_[0].pixelFormat);

	Size maxSize;
	for (const auto &cfg : config_) {
		if (cfg.size > maxSize)
			maxSize = cfg.size;
	}

	V4L2SubdeviceFormat sensorFormat{};
	sensorFormat.mbus_code = pipeFmt.sensorCode;
	sensorFormat.size = maxSize;

	LOG(ISI, Debug) << "Computed sensor configuration: " << sensorFormat;

	/*
	 * We can't use CameraSensor::getFormat() as it might return a
	 * format larger than our strict width limit, as that function
	 * prioritizes formats with the same aspect ratio over formats with less
	 * difference in size.
	 *
	 * Manually walk all the sensor supported sizes searching for
	 * the smallest larger format without considering the aspect ratio
	 * as the ISI can freely scale.
	 */
	auto sizes = sensor->sizes(sensorFormat.mbus_code);
	Size bestSize;

	for (const Size &s : sizes) {
		/* Ignore smaller sizes. */
		if (s.width < sensorFormat.size.width ||
		    s.height < sensorFormat.size.height)
			continue;

		/* Make sure the width stays in the limits. */
		if (s.width > maxResolution.width)
			continue;

		bestSize = s;
		break;
	}

	/*
	 * This should happen only if the sensor can only produce formats that
	 * exceed the maximum allowed input width.
	 */
	if (bestSize.isNull()) {
		LOG(ISI, Error) << "Unable to find a suitable sensor format";
		return Invalid;
	}

	sensorFormat_.mbus_code = sensorFormat.mbus_code;
	sensorFormat_.size = bestSize;

	LOG(ISI, Debug) << "Selected sensor format: " << sensorFormat_;

	return status;
}

/* -----------------------------------------------------------------------------
 * Pipeline Handler
 */

PipelineHandlerISI::PipelineHandlerISI(CameraManager *manager)
	: PipelineHandler(manager)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerISI::generateConfiguration(Camera *camera,
					  const StreamRoles &roles)
{
	ISICameraData *data = cameraData(camera);
	std::unique_ptr<ISICameraConfiguration> config =
		std::make_unique<ISICameraConfiguration>(data);

	if (roles.empty())
		return config;

	if (roles.size() > data->streams_.size()) {
		LOG(ISI, Error) << "Only up to " << data->streams_.size()
				<< " streams are supported";
		return nullptr;
	}

	bool isRaw = false;
	for (const auto &role : roles) {
		/*
		 * Prefer the following formats
		 * - Still Capture: Full resolution YUYV
		 * - ViewFinder/VideoRecording: 1080p YUYV
		 * - RAW: sensor's native format and resolution
		 */
		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		PixelFormat pixelFormat;
		Size size;

		switch (role) {
		case StreamRole::StillCapture:
			/*
			 * \todo Make sure the sensor can produce non-RAW formats
			 * compatible with the ones supported by the pipeline.
			 */
			size = data->sensor_->resolution();
			pixelFormat = formats::YUYV;
			break;

		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording:
			/*
			 * \todo Make sure the sensor can produce non-RAW formats
			 * compatible with the ones supported by the pipeline.
			 */
			size = PipelineHandlerISI::kPreviewSize;
			pixelFormat = formats::YUYV;
			break;

		case StreamRole::Raw: {
			/*
			 * Make sure the sensor can generate a RAW format and
			 * prefer the ones with a larger bitdepth.
			 */
			const ISICameraConfiguration::FormatMap::value_type *rawPipeFormat = nullptr;
			unsigned int maxDepth = 0;

			for (unsigned int code : data->sensor_->mbusCodes()) {
				const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(code);
				if (!bayerFormat.isValid())
					continue;

				/* Make sure the format is supported by the pipeline handler. */
				auto it = std::find_if(ISICameraConfiguration::formatsMap_.begin(),
						       ISICameraConfiguration::formatsMap_.end(),
						       [code](auto &isiFormat) {
							        auto &pipe = isiFormat.second;
							        return pipe.sensorCode == code;
						       });
				if (it == ISICameraConfiguration::formatsMap_.end())
					continue;

				if (bayerFormat.bitDepth > maxDepth) {
					maxDepth = bayerFormat.bitDepth;
					rawPipeFormat = &(*it);
				}
			}

			if (!rawPipeFormat) {
				LOG(ISI, Error)
					<< "Cannot generate a configuration for RAW stream";
				return nullptr;
			}

			size = data->sensor_->resolution();
			pixelFormat = rawPipeFormat->first;

			streamFormats[pixelFormat] = { { kMinISISize, size } };
			isRaw = true;

			break;
		}

		default:
			LOG(ISI, Error) << "Requested stream role not supported: " << role;
			return nullptr;
		}

		/*
		 * For non-RAW configurations the ISI can perform colorspace
		 * conversion. List all the supported output formats here.
		 */
		if (!isRaw) {
			for (const auto &[pixFmt, pipeFmt] : ISICameraConfiguration::formatsMap_) {
				const PixelFormatInfo &info = PixelFormatInfo::info(pixFmt);
				if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW)
					continue;

				streamFormats[pixFmt] = { { kMinISISize, size } };
			}
		}

		StreamFormats formats(streamFormats);

		StreamConfiguration cfg(formats);
		cfg.pixelFormat = pixelFormat;
		cfg.size = size;
		cfg.bufferCount = 4;
		config->addConfiguration(cfg);
	}

	config->validate();

	return config;
}

int PipelineHandlerISI::configure(Camera *camera, CameraConfiguration *c)
{
	ISICameraConfiguration *camConfig = static_cast<ISICameraConfiguration *>(c);
	ISICameraData *data = cameraData(camera);

	/* All links are immutable except the sensor -> csis link. */
	const MediaPad *sensorSrc = data->sensor_->entity()->getPadByIndex(0);
	sensorSrc->links()[0]->setEnabled(true);

	/*
	 * Reset the crossbar switch routing and enable one route for each
	 * requested stream configuration.
	 *
	 * \todo Handle concurrent usage of multiple cameras by adjusting the
	 * routing table instead of resetting it.
	 */
	V4L2Subdevice::Routing routing = {};
	unsigned int xbarFirstSource = crossbar_->entity()->pads().size() / 2 + 1;

	for (const auto &[idx, config] : utils::enumerate(*c)) {
		struct v4l2_subdev_route route = {
			.sink_pad = data->xbarSink_,
			.sink_stream = 0,
			.source_pad = static_cast<uint32_t>(xbarFirstSource + idx),
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
			.reserved = {}
		};

		routing.push_back(route);
	}

	int ret = crossbar_->setRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret)
		return ret;

	/* Apply format to the sensor and CSIS receiver. */
	V4L2SubdeviceFormat format = camConfig->sensorFormat_;
	ret = data->sensor_->setFormat(&format);
	if (ret)
		return ret;

	ret = data->csis_->setFormat(0, &format);
	if (ret)
		return ret;

	ret = crossbar_->setFormat(data->xbarSink_, &format);
	if (ret)
		return ret;

	/* Now configure the ISI and video node instances, one per stream. */
	data->enabledStreams_.clear();
	for (const auto &config : *c) {
		Pipe *pipe = pipeFromStream(camera, config.stream());

		/*
		 * Set the format on the ISI sink pad: it must match what is
		 * received by the CSIS.
		 */
		ret = pipe->isi->setFormat(0, &format);
		if (ret)
			return ret;

		/*
		 * Configure the ISI sink compose rectangle to downscale the
		 * image.
		 *
		 * \todo Additional cropping could be applied on the ISI source
		 * pad to further reduce the output image size.
		 */
		Rectangle isiScale(config.size);
		ret = pipe->isi->setSelection(0, V4L2_SEL_TGT_COMPOSE, &isiScale);
		if (ret)
			return ret;

		/*
		 * Set the format on ISI source pad: only the media bus code
		 * is relevant as it configures format conversion, while the
		 * size is taken from the sink's COMPOSE (or source's CROP,
		 * if any) rectangles.
		 */
		const ISICameraConfiguration::PipeFormat &pipeFormat =
			ISICameraConfiguration::formatsMap_.at(config.pixelFormat);

		V4L2SubdeviceFormat isiFormat{};
		isiFormat.mbus_code = pipeFormat.isiCode;
		isiFormat.size = config.size;

		ret = pipe->isi->setFormat(1, &isiFormat);
		if (ret)
			return ret;

		V4L2DeviceFormat captureFmt{};
		captureFmt.fourcc = pipe->capture->toV4L2PixelFormat(config.pixelFormat);
		captureFmt.size = config.size;

		/* \todo Set stride and format. */
		ret = pipe->capture->setFormat(&captureFmt);
		if (ret)
			return ret;

		/* Store the list of enabled streams for later use. */
		data->enabledStreams_.push_back(config.stream());
	}

	return 0;
}

int PipelineHandlerISI::exportFrameBuffers(Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	unsigned int count = stream->configuration().bufferCount;
	Pipe *pipe = pipeFromStream(camera, stream);

	return pipe->capture->exportBuffers(count, buffers);
}

int PipelineHandlerISI::start(Camera *camera,
			      [[maybe_unused]] const ControlList *controls)
{
	ISICameraData *data = cameraData(camera);

	for (const auto &stream : data->enabledStreams_) {
		Pipe *pipe = pipeFromStream(camera, stream);
		const StreamConfiguration &config = stream->configuration();

		int ret = pipe->capture->importBuffers(config.bufferCount);
		if (ret)
			return ret;

		ret = pipe->capture->streamOn();
		if (ret)
			return ret;
	}

	return 0;
}

void PipelineHandlerISI::stopDevice(Camera *camera)
{
	ISICameraData *data = cameraData(camera);

	for (const auto &stream : data->enabledStreams_) {
		Pipe *pipe = pipeFromStream(camera, stream);

		pipe->capture->streamOff();
		pipe->capture->releaseBuffers();
	}
}

int PipelineHandlerISI::queueRequestDevice(Camera *camera, Request *request)
{
	for (auto &[stream, buffer] : request->buffers()) {
		Pipe *pipe = pipeFromStream(camera, stream);

		int ret = pipe->capture->queueBuffer(buffer);
		if (ret)
			return ret;
	}

	return 0;
}

bool PipelineHandlerISI::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("mxc-isi");
	dm.add("crossbar");
	dm.add("mxc_isi.0");
	dm.add("mxc_isi.0.capture");

	isiDev_ = acquireMediaDevice(enumerator, dm);
	if (!isiDev_)
		return false;

	/*
	 * Acquire the subdevs and video nodes for the crossbar switch and the
	 * processing pipelines.
	 */
	crossbar_ = V4L2Subdevice::fromEntityName(isiDev_, "crossbar");
	if (!crossbar_)
		return false;

	int ret = crossbar_->open();
	if (ret)
		return false;

	for (unsigned int i = 0; ; ++i) {
		std::string entityName = "mxc_isi." + std::to_string(i);
		std::unique_ptr<V4L2Subdevice> isi =
			V4L2Subdevice::fromEntityName(isiDev_, entityName);
		if (!isi)
			break;

		ret = isi->open();
		if (ret)
			return false;

		entityName += ".capture";
		std::unique_ptr<V4L2VideoDevice> capture =
			V4L2VideoDevice::fromEntityName(isiDev_, entityName);
		if (!capture)
			return false;

		capture->bufferReady.connect(this, &PipelineHandlerISI::bufferReady);

		ret = capture->open();
		if (ret)
			return ret;

		pipes_.push_back({ std::move(isi), std::move(capture) });
	}

	if (pipes_.empty()) {
		LOG(ISI, Error) << "Unable to enumerate pipes";
		return false;
	}

	/*
	 * Loop over all the crossbar switch sink pads to find connected CSI-2
	 * receivers and camera sensors.
	 */
	unsigned int numCameras = 0;
	unsigned int numSinks = 0;
	for (MediaPad *pad : crossbar_->entity()->pads()) {
		unsigned int sink = numSinks;

		if (!(pad->flags() & MEDIA_PAD_FL_SINK) || pad->links().empty())
			continue;

		/*
		 * Count each crossbar sink pad to correctly configure
		 * routing and format for this camera.
		 */
		numSinks++;

		MediaEntity *csi = pad->links()[0]->source()->entity();
		if (csi->pads().size() != 2) {
			LOG(ISI, Debug) << "Skip unsupported CSI-2 receiver "
					<< csi->name();
			continue;
		}

		pad = csi->pads()[0];
		if (!(pad->flags() & MEDIA_PAD_FL_SINK) || pad->links().empty())
			continue;

		MediaEntity *sensor = pad->links()[0]->source()->entity();
		if (sensor->function() != MEDIA_ENT_F_CAM_SENSOR) {
			LOG(ISI, Debug) << "Skip unsupported subdevice "
					<< sensor->name();
			continue;
		}

		/* Create the camera data. */
		std::unique_ptr<ISICameraData> data =
			std::make_unique<ISICameraData>(this);

		data->sensor_ = std::make_unique<CameraSensor>(sensor);
		data->csis_ = std::make_unique<V4L2Subdevice>(csi);
		data->xbarSink_ = sink;

		ret = data->init();
		if (ret) {
			LOG(ISI, Error) << "Failed to initialize camera data";
			return false;
		}

		/* Register the camera. */
		const std::string &id = data->sensor_->id();
		std::set<Stream *> streams;
		std::transform(data->streams_.begin(), data->streams_.end(),
			       std::inserter(streams, streams.end()),
			       [](Stream &s) { return &s; });

		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), id, streams);

		registerCamera(std::move(camera));
		numCameras++;
	}

	return numCameras > 0;
}

PipelineHandlerISI::Pipe *PipelineHandlerISI::pipeFromStream(Camera *camera,
							     const Stream *stream)
{
	ISICameraData *data = cameraData(camera);
	unsigned int pipeIndex = data->pipeIndex(stream);

	ASSERT(pipeIndex < pipes_.size());

	return &pipes_[pipeIndex];
}

void PipelineHandlerISI::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	/* Record the sensor's timestamp in the request metadata. */
	ControlList &metadata = request->metadata();
	if (!metadata.contains(controls::SensorTimestamp.id()))
		metadata.set(controls::SensorTimestamp,
			     buffer->metadata().timestamp);

	completeBuffer(request, buffer);
	if (request->hasPendingBuffers())
		return;

	completeRequest(request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerISI)

} /* namespace libcamera */
