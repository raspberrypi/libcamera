/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022 - Jacopo Mondi <jacopo@jmondi.org>
 *
 * Pipeline handler for ISI interface found on NXP i.MX8 SoC
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

	unsigned int getRawMediaBusFormat(PixelFormat *pixelFormat) const;
	unsigned int getYuvMediaBusFormat(const PixelFormat &pixelFormat) const;
	unsigned int getMediaBusFormat(PixelFormat *pixelFormat) const;

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<V4L2Subdevice> csis_;

	std::vector<Stream> streams_;

	std::vector<Stream *> enabledStreams_;

	unsigned int xbarSink_;
};

class ISICameraConfiguration : public CameraConfiguration
{
public:
	ISICameraConfiguration(ISICameraData *data)
		: data_(data)
	{
	}

	Status validate() override;

	static const std::map<PixelFormat, unsigned int> formatsMap_;

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
	generateConfiguration(Camera *camera, Span<const StreamRole> roles) override;
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

	StreamConfiguration generateYUVConfiguration(Camera *camera,
						     const Size &size);
	StreamConfiguration generateRawConfiguration(Camera *camera);

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
	if (!sensor_)
		return -ENODEV;

	int ret = csis_->open();
	if (ret)
		return ret;

	properties_ = sensor_->properties();

	return 0;
}

/*
 * Get a RAW Bayer media bus format compatible with the requested pixelFormat.
 *
 * If the requested pixelFormat cannot be produced by the sensor adjust it to
 * the one corresponding to the media bus format with the largest bit-depth.
 */
unsigned int ISICameraData::getRawMediaBusFormat(PixelFormat *pixelFormat) const
{
	std::vector<unsigned int> mbusCodes = sensor_->mbusCodes();

	static const std::map<PixelFormat, unsigned int> rawFormats = {
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
		{ formats::SBGGR14, MEDIA_BUS_FMT_SBGGR14_1X14 },
		{ formats::SGBRG14, MEDIA_BUS_FMT_SGBRG14_1X14 },
		{ formats::SGRBG14, MEDIA_BUS_FMT_SGRBG14_1X14 },
		{ formats::SRGGB14, MEDIA_BUS_FMT_SRGGB14_1X14 },
	};

	/*
	 * Make sure the requested PixelFormat is supported in the above
	 * map and the sensor can produce the compatible mbus code.
	 */
	auto it = rawFormats.find(*pixelFormat);
	if (it != rawFormats.end() &&
	    std::count(mbusCodes.begin(), mbusCodes.end(), it->second))
		return it->second;

	if (it == rawFormats.end())
		LOG(ISI, Warning) << pixelFormat
				  << " not supported in ISI formats map.";

	/*
	 * The desired pixel format cannot be produced. Adjust it to the one
	 * corresponding to the raw media bus format with the largest bit-depth
	 * the sensor provides.
	 */
	unsigned int sensorCode = 0;
	unsigned int maxDepth = 0;
	*pixelFormat = {};

	for (unsigned int code : mbusCodes) {
		/* Make sure the media bus format is RAW Bayer. */
		const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(code);
		if (!bayerFormat.isValid())
			continue;

		/* Make sure the media format is supported. */
		it = std::find_if(rawFormats.begin(), rawFormats.end(),
				  [code](auto &rawFormat) {
					  return rawFormat.second == code;
				  });

		if (it == rawFormats.end()) {
			LOG(ISI, Warning) << bayerFormat
					  << " not supported in ISI formats map.";
			continue;
		}

		/* Pick the one with the largest bit depth. */
		if (bayerFormat.bitDepth > maxDepth) {
			maxDepth = bayerFormat.bitDepth;
			*pixelFormat = it->first;
			sensorCode = code;
		}
	}

	if (!pixelFormat->isValid())
		LOG(ISI, Error) << "Cannot find a supported RAW format";

	return sensorCode;
}

/*
 * Get a YUV/RGB media bus format from which the ISI can produce a processed
 * stream, preferring codes with the same colour encoding as the requested
 * pixelformat.
 *
 * If the sensor does not provide any YUV/RGB media bus format the ISI cannot
 * generate any processed pixel format as it cannot debayer.
 */
unsigned int ISICameraData::getYuvMediaBusFormat(const PixelFormat &pixelFormat) const
{
	std::vector<unsigned int> mbusCodes = sensor_->mbusCodes();

	/*
	 * The ISI can produce YUV/RGB pixel formats from any non-RAW Bayer
	 * media bus formats.
	 *
	 * Keep the list in sync with the mxc_isi_bus_formats[] array in
	 * the ISI driver.
	 */
	std::vector<unsigned int> yuvCodes = {
		MEDIA_BUS_FMT_UYVY8_1X16,
		MEDIA_BUS_FMT_YUV8_1X24,
		MEDIA_BUS_FMT_RGB565_1X16,
		MEDIA_BUS_FMT_RGB888_1X24,
	};

	std::sort(mbusCodes.begin(), mbusCodes.end());
	std::sort(yuvCodes.begin(), yuvCodes.end());

	std::vector<unsigned int> supportedCodes;
	std::set_intersection(mbusCodes.begin(), mbusCodes.end(),
			      yuvCodes.begin(), yuvCodes.end(),
			      std::back_inserter(supportedCodes));

	if (supportedCodes.empty()) {
		LOG(ISI, Warning) << "Cannot find a supported YUV/RGB format";

		return 0;
	}

	/* Prefer codes with the same encoding as the requested pixel format. */
	const PixelFormatInfo &info = PixelFormatInfo::info(pixelFormat);
	for (unsigned int code : supportedCodes) {
		if (info.colourEncoding == PixelFormatInfo::ColourEncodingYUV &&
		    (code == MEDIA_BUS_FMT_UYVY8_1X16 ||
		     code == MEDIA_BUS_FMT_YUV8_1X24))
			return code;

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRGB &&
		    (code == MEDIA_BUS_FMT_RGB565_1X16 ||
		     code == MEDIA_BUS_FMT_RGB888_1X24))
			return code;
	}

	/* Otherwise return the first found code. */
	return supportedCodes[0];
}

unsigned int ISICameraData::getMediaBusFormat(PixelFormat *pixelFormat) const
{
	if (PixelFormatInfo::info(*pixelFormat).colourEncoding ==
	    PixelFormatInfo::ColourEncodingRAW)
		return getRawMediaBusFormat(pixelFormat);

	return getYuvMediaBusFormat(*pixelFormat);
}

/* -----------------------------------------------------------------------------
 * Camera Configuration
 */

/*
 * ISICameraConfiguration::formatsMap_ records the association between an output
 * pixel format and the ISI source pixel format to be applied to the pipeline.
 */
const std::map<PixelFormat, unsigned int> ISICameraConfiguration::formatsMap_ = {
	{ formats::YUYV, MEDIA_BUS_FMT_YUV8_1X24 },
	{ formats::AVUY8888, MEDIA_BUS_FMT_YUV8_1X24 },
	{ formats::NV12, MEDIA_BUS_FMT_YUV8_1X24 },
	{ formats::NV16, MEDIA_BUS_FMT_YUV8_1X24 },
	{ formats::YUV444, MEDIA_BUS_FMT_YUV8_1X24 },
	{ formats::RGB565, MEDIA_BUS_FMT_RGB888_1X24 },
	{ formats::BGR888, MEDIA_BUS_FMT_RGB888_1X24 },
	{ formats::RGB888, MEDIA_BUS_FMT_RGB888_1X24 },
	{ formats::XRGB8888, MEDIA_BUS_FMT_RGB888_1X24 },
	{ formats::ABGR8888, MEDIA_BUS_FMT_RGB888_1X24 },
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
	StreamConfiguration &rawConfig = config_[0];
	PixelFormat rawFormat = rawConfig.pixelFormat;

	unsigned int sensorCode = data_->getRawMediaBusFormat(&rawFormat);
	if (!sensorCode) {
		LOG(ISI, Error) << "Cannot adjust RAW pixelformat "
				<< rawConfig.pixelFormat;
		return Invalid;
	}

	if (rawFormat != rawConfig.pixelFormat) {
		LOG(ISI, Debug) << "RAW pixelformat adjusted to "
				<< rawFormat;
		rawConfig.pixelFormat = rawFormat;
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

	StreamConfiguration &yuvConfig = config_[0];
	PixelFormat yuvPixelFormat = yuvConfig.pixelFormat;

	/*
	 * Make sure the sensor can produce a compatible YUV/RGB media bus
	 * format. If the sensor can only produce RAW Bayer we can only fail
	 * here as we can't adjust to anything but RAW.
	 */
	unsigned int yuvMediaBusCode = data_->getYuvMediaBusFormat(yuvPixelFormat);
	if (!yuvMediaBusCode) {
		LOG(ISI, Error) << "Cannot adjust pixelformat "
				<< yuvConfig.pixelFormat;
		return Invalid;
	}

	/* Adjust all the other streams. */
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
	Size maxSize;
	for (const auto &cfg : config_) {
		if (cfg.size > maxSize)
			maxSize = cfg.size;
	}

	PixelFormat pixelFormat = config_[0].pixelFormat;

	V4L2SubdeviceFormat sensorFormat{};
	sensorFormat.code = data_->getMediaBusFormat(&pixelFormat);
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
	auto sizes = sensor->sizes(sensorFormat.code);
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

	sensorFormat_.code = sensorFormat.code;
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

/*
 * Generate a StreamConfiguration for YUV/RGB use case.
 *
 * Verify it the sensor can produce a YUV/RGB media bus format and collect
 * all the processed pixel formats the ISI can generate as supported stream
 * configurations.
 */
StreamConfiguration PipelineHandlerISI::generateYUVConfiguration(Camera *camera,
								 const Size &size)
{
	ISICameraData *data = cameraData(camera);
	PixelFormat pixelFormat = formats::YUYV;
	unsigned int mbusCode;

	mbusCode = data->getYuvMediaBusFormat(pixelFormat);
	if (!mbusCode)
		return {};

	/* Adjust the requested size to the sensor's capabilities. */
	V4L2SubdeviceFormat sensorFmt;
	sensorFmt.code = mbusCode;
	sensorFmt.size = size;

	int ret = data->sensor_->tryFormat(&sensorFmt);
	if (ret) {
		LOG(ISI, Error) << "Failed to try sensor format.";
		return {};
	}

	Size sensorSize = sensorFmt.size;

	/*
	 * Populate the StreamConfiguration.
	 *
	 * As the sensor supports at least one YUV/RGB media bus format all the
	 * processed ones in formatsMap_ can be generated from it.
	 */
	std::map<PixelFormat, std::vector<SizeRange>> streamFormats;

	for (const auto &[pixFmt, pipeFmt] : ISICameraConfiguration::formatsMap_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(pixFmt);
		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW)
			continue;

		streamFormats[pixFmt] = { { kMinISISize, sensorSize } };
	}

	StreamFormats formats(streamFormats);

	StreamConfiguration cfg(formats);
	cfg.pixelFormat = pixelFormat;
	cfg.size = sensorSize;
	cfg.bufferCount = 4;

	return cfg;
}

/*
 * Generate a StreamConfiguration for Raw Bayer use case. Verify if the sensor
 * can produce the requested RAW bayer format and eventually adjust it to
 * the one with the largest bit-depth the sensor can produce.
 */
StreamConfiguration PipelineHandlerISI::generateRawConfiguration(Camera *camera)
{
	static const std::map<unsigned int, PixelFormat> rawFormats = {
		{ MEDIA_BUS_FMT_SBGGR8_1X8, formats::SBGGR8 },
		{ MEDIA_BUS_FMT_SGBRG8_1X8, formats::SGBRG8 },
		{ MEDIA_BUS_FMT_SGRBG8_1X8, formats::SGRBG8 },
		{ MEDIA_BUS_FMT_SRGGB8_1X8, formats::SRGGB8 },
		{ MEDIA_BUS_FMT_SBGGR10_1X10, formats::SBGGR10 },
		{ MEDIA_BUS_FMT_SGBRG10_1X10, formats::SGBRG10 },
		{ MEDIA_BUS_FMT_SGRBG10_1X10, formats::SGRBG10 },
		{ MEDIA_BUS_FMT_SRGGB10_1X10, formats::SRGGB10 },
		{ MEDIA_BUS_FMT_SBGGR12_1X12, formats::SBGGR12 },
		{ MEDIA_BUS_FMT_SGBRG12_1X12, formats::SGBRG12 },
		{ MEDIA_BUS_FMT_SGRBG12_1X12, formats::SGRBG12 },
		{ MEDIA_BUS_FMT_SRGGB12_1X12, formats::SRGGB12 },
		{ MEDIA_BUS_FMT_SBGGR14_1X14, formats::SBGGR14 },
		{ MEDIA_BUS_FMT_SGBRG14_1X14, formats::SGBRG14 },
		{ MEDIA_BUS_FMT_SGRBG14_1X14, formats::SGRBG14 },
		{ MEDIA_BUS_FMT_SRGGB14_1X14, formats::SRGGB14 },
	};

	ISICameraData *data = cameraData(camera);
	PixelFormat pixelFormat = formats::SBGGR10;
	unsigned int mbusCode;

	/* pixelFormat will be adjusted, if the sensor can produce RAW. */
	mbusCode = data->getRawMediaBusFormat(&pixelFormat);
	if (!mbusCode)
		return {};

	/*
	 * Populate the StreamConfiguration with all the supported Bayer
	 * formats the sensor can produce.
	 */
	std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
	const CameraSensor *sensor = data->sensor_.get();

	for (unsigned int code : sensor->mbusCodes()) {
		/* Find a Bayer media bus code from the sensor. */
		const BayerFormat &bayerFormat = BayerFormat::fromMbusCode(code);
		if (!bayerFormat.isValid())
			continue;

		auto it = rawFormats.find(code);
		if (it == rawFormats.end()) {
			LOG(ISI, Warning) << bayerFormat
					  << " not supported in ISI formats map.";
			continue;
		}

		streamFormats[it->second] = { { sensor->resolution(), sensor->resolution() } };
	}

	StreamFormats formats(streamFormats);

	StreamConfiguration cfg(formats);
	cfg.size = sensor->resolution();
	cfg.pixelFormat = pixelFormat;
	cfg.bufferCount = 4;

	return cfg;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerISI::generateConfiguration(Camera *camera,
					  Span<const StreamRole> roles)
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

	for (const auto &role : roles) {
		/*
		 * Prefer the following formats:
		 * - Still Capture: Full resolution YUYV
		 * - ViewFinder/VideoRecording: 1080p YUYV
		 * - RAW: Full resolution Bayer
		 */
		StreamConfiguration cfg;

		switch (role) {
		case StreamRole::StillCapture:
		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording: {
			Size size = role == StreamRole::StillCapture
				  ? data->sensor_->resolution()
				  : PipelineHandlerISI::kPreviewSize;
			cfg = generateYUVConfiguration(camera, size);
			if (cfg.pixelFormat.isValid())
				break;

			/*
			 * Fallback to use a Bayer format if that's what the
			 * sensor supports.
			 */
			[[fallthrough]];
		}

		case StreamRole::Raw: {
			cfg = generateRawConfiguration(camera);
			break;
		}

		default:
			LOG(ISI, Error) << "Requested stream role not supported: " << role;
			return nullptr;
		}

		if (!cfg.pixelFormat.isValid()) {
			LOG(ISI, Error)
				<< "Cannot generate configuration for role: " << role;
			return nullptr;
		}

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
	unsigned int xbarFirstSource = crossbar_->entity()->pads().size() - pipes_.size();

	for (const auto &[idx, config] : utils::enumerate(*c)) {
		uint32_t sourcePad = xbarFirstSource + idx;
		routing.emplace_back(V4L2Subdevice::Stream{ data->xbarSink_, 0 },
				     V4L2Subdevice::Stream{ sourcePad, 0 },
				     V4L2_SUBDEV_ROUTE_FL_ACTIVE);
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
		unsigned int isiCode = ISICameraConfiguration::formatsMap_.at(config.pixelFormat);

		V4L2SubdeviceFormat isiFormat{};
		isiFormat.code = isiCode;
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
			return false;

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

		data->sensor_ = CameraSensorFactoryBase::create(sensor);
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

REGISTER_PIPELINE_HANDLER(PipelineHandlerISI, "imx8-isi")

} /* namespace libcamera */
