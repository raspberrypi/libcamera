/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Rockchip ISP1 path helper
 */

#include "rkisp1_path.h"

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RkISP1)

namespace {

/* Keep in sync with the supported raw formats in rkisp1.cpp. */
const std::map<PixelFormat, uint32_t> formatToMediaBus = {
	{ formats::UYVY, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::YUYV, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::NV12, MEDIA_BUS_FMT_YUYV8_1_5X8 },
	{ formats::NV21, MEDIA_BUS_FMT_YUYV8_1_5X8 },
	{ formats::NV16, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::NV61, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::YUV420, MEDIA_BUS_FMT_YUYV8_1_5X8 },
	{ formats::YVU420, MEDIA_BUS_FMT_YUYV8_1_5X8 },
	{ formats::YUV422, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::YVU422, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::R8, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::RGB565, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::XRGB8888, MEDIA_BUS_FMT_YUYV8_2X8 },
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

RkISP1Path::RkISP1Path(const char *name, const Span<const PixelFormat> &formats,
		       const Size &minResolution, const Size &maxResolution)
	: name_(name), running_(false), formats_(formats),
	  minResolution_(minResolution), maxResolution_(maxResolution),
	  link_(nullptr)
{
}

bool RkISP1Path::init(MediaDevice *media)
{
	std::string resizer = std::string("rkisp1_resizer_") + name_ + "path";
	std::string video = std::string("rkisp1_") + name_ + "path";

	resizer_ = V4L2Subdevice::fromEntityName(media, resizer);
	if (resizer_->open() < 0)
		return false;

	video_ = V4L2VideoDevice::fromEntityName(media, video);
	if (video_->open() < 0)
		return false;

	populateFormats();

	link_ = media->link("rkisp1_isp", 2, resizer, 0);
	if (!link_)
		return false;

	return true;
}

void RkISP1Path::populateFormats()
{
	V4L2VideoDevice::Formats v4l2Formats = video_->formats();
	if (v4l2Formats.empty()) {
		LOG(RkISP1, Info)
			<< "Failed to enumerate supported formats and sizes, using defaults";

		for (const PixelFormat &format : formats_)
			streamFormats_.insert(format);
		return;
	}

	minResolution_ = { 65535, 65535 };
	maxResolution_ = { 0, 0 };

	std::vector<PixelFormat> formats;
	for (const auto &[format, sizes] : v4l2Formats) {
		const PixelFormat pixelFormat = format.toPixelFormat();

		/*
		 * As a defensive measure, skip any pixel format exposed by the
		 * driver that we don't know about. This ensures that looking up
		 * formats in formatToMediaBus using a key from streamFormats_
		 * will never fail in any of the other functions.
		 */
		if (!formatToMediaBus.count(pixelFormat)) {
			LOG(RkISP1, Warning)
				<< "Unsupported pixel format " << pixelFormat;
			continue;
		}

		streamFormats_.insert(pixelFormat);

		for (const auto &size : sizes) {
			if (minResolution_ > size.min)
				minResolution_ = size.min;
			if (maxResolution_ < size.max)
				maxResolution_ = size.max;
		}
	}
}

/**
 * \brief Filter the sensor resolutions that can be supported
 * \param[in] sensor The camera sensor
 *
 * This function retrieves all the sizes supported by the sensor and
 * filters all the resolutions that can be supported on the pipeline.
 * It is possible that the sensor's maximum output resolution is higher
 * than the ISP maximum input. In that case, this function filters out all
 * the resolution incapable of being supported and returns the maximum
 * sensor resolution that can be supported by the pipeline.
 *
 * \return Maximum sensor size supported on the pipeline
 */
Size RkISP1Path::filterSensorResolution(const CameraSensor *sensor)
{
	auto iter = sensorSizesMap_.find(sensor);
	if (iter != sensorSizesMap_.end())
		return iter->second.back();

	std::vector<Size> &sizes = sensorSizesMap_[sensor];
	for (unsigned int code : sensor->mbusCodes()) {
		for (const Size &size : sensor->sizes(code)) {
			if (size.width > maxResolution_.width ||
			    size.height > maxResolution_.height)
				continue;

			sizes.push_back(size);
		}
	}

	/* Sort in increasing order and remove duplicates. */
	std::sort(sizes.begin(), sizes.end());
	auto last = std::unique(sizes.begin(), sizes.end());
	sizes.erase(last, sizes.end());

	return sizes.back();
}

StreamConfiguration
RkISP1Path::generateConfiguration(const CameraSensor *sensor, const Size &size,
				  StreamRole role)
{
	const std::vector<unsigned int> &mbusCodes = sensor->mbusCodes();
	Size resolution = filterSensorResolution(sensor);

	/* Min and max resolutions to populate the available stream formats. */
	Size maxResolution = maxResolution_.boundedToAspectRatio(resolution)
					   .boundedTo(resolution);
	Size minResolution = minResolution_.expandedToAspectRatio(resolution);

	/* The desired stream size, bound to the max resolution. */
	Size streamSize = size.boundedTo(maxResolution);

	/* Create the list of supported stream formats. */
	std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
	unsigned int rawBitsPerPixel = 0;
	PixelFormat rawFormat;

	for (const auto &format : streamFormats_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(format);

		/* Populate stream formats for non-RAW configurations. */
		if (info.colourEncoding != PixelFormatInfo::ColourEncodingRAW) {
			if (role == StreamRole::Raw)
				continue;

			streamFormats[format] = { { minResolution, maxResolution } };
			continue;
		}

		/* Skip RAW formats for non-raw roles. */
		if (role != StreamRole::Raw)
			continue;

		/* Populate stream formats for RAW configurations. */
		uint32_t mbusCode = formatToMediaBus.at(format);
		if (std::find(mbusCodes.begin(), mbusCodes.end(), mbusCode) ==
		    mbusCodes.end())
			/* Skip formats not supported by sensor. */
			continue;

		/* Add all the RAW sizes the sensor can produce for this code. */
		for (const auto &rawSize : sensor->sizes(mbusCode)) {
			if (rawSize.width > maxResolution_.width ||
			    rawSize.height > maxResolution_.height)
				continue;

			streamFormats[format].push_back({ rawSize, rawSize });
		}

		/*
		 * Store the raw format with the highest bits per pixel for
		 * later usage.
		 */
		if (info.bitsPerPixel > rawBitsPerPixel) {
			rawBitsPerPixel = info.bitsPerPixel;
			rawFormat = format;
		}
	}

	/*
	 * Pick a suitable pixel format for the role. Raw streams need to use a
	 * raw format, processed streams use NV12 by default.
	 */
	PixelFormat format;

	if (role == StreamRole::Raw) {
		if (!rawFormat.isValid()) {
			LOG(RkISP1, Error)
				<< "Sensor " << sensor->model()
				<< " doesn't support raw capture";
			return {};
		}

		format = rawFormat;
	} else {
		format = formats::NV12;
	}

	StreamFormats formats(streamFormats);
	StreamConfiguration cfg(formats);
	cfg.pixelFormat = format;
	cfg.size = streamSize;
	cfg.bufferCount = RKISP1_BUFFER_COUNT;

	return cfg;
}

CameraConfiguration::Status
RkISP1Path::validate(const CameraSensor *sensor,
		     const std::optional<SensorConfiguration> &sensorConfig,
		     StreamConfiguration *cfg)
{
	const std::vector<unsigned int> &mbusCodes = sensor->mbusCodes();
	Size resolution = filterSensorResolution(sensor);

	const StreamConfiguration reqCfg = *cfg;
	CameraConfiguration::Status status = CameraConfiguration::Valid;

	/*
	 * Validate the pixel format. If the requested format isn't supported,
	 * default to either NV12 (all versions of the ISP are guaranteed to
	 * support NV12 on both the main and self paths) if the requested format
	 * is not a raw format, or to the supported raw format with the highest
	 * bits per pixel otherwise.
	 */
	unsigned int rawBitsPerPixel = 0;
	PixelFormat rawFormat;
	bool found = false;

	for (const auto &format : streamFormats_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(format);

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			/* Skip raw formats not supported by the sensor. */
			uint32_t mbusCode = formatToMediaBus.at(format);
			if (std::find(mbusCodes.begin(), mbusCodes.end(), mbusCode) ==
			    mbusCodes.end())
				continue;

			/*
			 * If the bits per pixel is supplied from the sensor
			 * configuration, choose a raw format that complies with
			 * it. Otherwise, store the raw format with the highest
			 * bits per pixel for later usage.
			 */
			if (sensorConfig && info.bitsPerPixel != sensorConfig->bitDepth)
				continue;

			if (info.bitsPerPixel > rawBitsPerPixel) {
				rawBitsPerPixel = info.bitsPerPixel;
				rawFormat = format;
			}
		}

		if (cfg->pixelFormat == format) {
			found = true;
			break;
		}
	}

	if (sensorConfig && !rawFormat.isValid())
		return CameraConfiguration::Invalid;

	bool isRaw = PixelFormatInfo::info(cfg->pixelFormat).colourEncoding ==
		     PixelFormatInfo::ColourEncodingRAW;

	/*
	 * If no raw format supported by the sensor has been found, use a
	 * processed format.
	 */
	if (!rawFormat.isValid())
		isRaw = false;

	if (!found)
		cfg->pixelFormat = isRaw ? rawFormat : formats::NV12;

	Size minResolution;
	Size maxResolution;

	if (isRaw) {
		/*
		 * Use the sensor output size closest to the requested stream
		 * size while ensuring the output size doesn't exceed ISP limits.
		 *
		 * As 'resolution' is the largest sensor resolution
		 * supported by the ISP, CameraSensor::getFormat() will never
		 * return a V4L2SubdeviceFormat with a larger size.
		 */
		uint32_t mbusCode = formatToMediaBus.at(cfg->pixelFormat);
		cfg->size.boundTo(resolution);

		Size rawSize = sensorConfig ? sensorConfig->outputSize
					    : cfg->size;

		V4L2SubdeviceFormat sensorFormat =
			sensor->getFormat({ mbusCode }, rawSize);

		if (sensorConfig &&
		    sensorConfig->outputSize != sensorFormat.size)
			return CameraConfiguration::Invalid;

		minResolution = sensorFormat.size;
		maxResolution = sensorFormat.size;
	} else if (sensorConfig) {
		/*
		 * We have already ensured 'rawFormat' has the matching bit
		 * depth with sensorConfig.bitDepth hence, only validate the
		 * sensorConfig's output size here.
		 */
		Size sensorSize = sensorConfig->outputSize;

		if (sensorSize > resolution)
			return CameraConfiguration::Invalid;

		uint32_t mbusCode = formatToMediaBus.at(rawFormat);
		V4L2SubdeviceFormat sensorFormat =
			sensor->getFormat({ mbusCode }, sensorSize);

		if (sensorFormat.size != sensorSize)
			return CameraConfiguration::Invalid;

		minResolution = minResolution_.expandedToAspectRatio(sensorSize);
		maxResolution = maxResolution_.boundedTo(sensorSize)
					      .boundedToAspectRatio(sensorSize);
	} else {
		/*
		 * Adjust the size based on the sensor resolution and absolute
		 * limits of the ISP.
		 */
		minResolution = minResolution_.expandedToAspectRatio(resolution);
		maxResolution = maxResolution_.boundedToAspectRatio(resolution)
					      .boundedTo(resolution);
	}

	cfg->size.boundTo(maxResolution);
	cfg->size.expandTo(minResolution);
	cfg->bufferCount = RKISP1_BUFFER_COUNT;

	V4L2DeviceFormat format;
	format.fourcc = video_->toV4L2PixelFormat(cfg->pixelFormat);
	format.size = cfg->size;

	int ret = video_->tryFormat(&format);
	if (ret)
		return CameraConfiguration::Invalid;

	cfg->stride = format.planes[0].bpl;
	cfg->frameSize = format.planes[0].size;

	if (cfg->pixelFormat != reqCfg.pixelFormat || cfg->size != reqCfg.size) {
		LOG(RkISP1, Debug)
			<< "Adjusting format from " << reqCfg.toString()
			<< " to " << cfg->toString();
		status = CameraConfiguration::Adjusted;
	}

	return status;
}

int RkISP1Path::configure(const StreamConfiguration &config,
			  const V4L2SubdeviceFormat &inputFormat)
{
	int ret;

	V4L2SubdeviceFormat ispFormat = inputFormat;

	ret = resizer_->setFormat(0, &ispFormat);
	if (ret < 0)
		return ret;

	/*
	 * Crop on the resizer input to maintain FOV before downscaling.
	 *
	 * Note that this does not apply to devices without DUAL_CROP support
	 * (like imx8mp) , where the cropping needs to be done on the
	 * ImageStabilizer block on the ISP source pad and therefore is
	 * configured before this stage. For simplicity we still set the crop.
	 * This gets ignored by the kernel driver because the hardware is
	 * missing the capability.
	 *
	 * Alignment to a multiple of 2 pixels is required by the resizer.
	 */
	Size ispCrop = inputFormat.size.boundedToAspectRatio(config.size)
				       .alignedUpTo(2, 2);
	Rectangle rect = ispCrop.centeredTo(Rectangle(inputFormat.size).center());
	ret = resizer_->setSelection(0, V4L2_SEL_TGT_CROP, &rect);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "Configured " << name_ << " resizer input pad with "
		<< ispFormat << " crop " << rect;

	ispFormat.size = config.size;

	LOG(RkISP1, Debug)
		<< "Configuring " << name_ << " resizer output pad with "
		<< ispFormat;

	/*
	 * The configuration has been validated, the pixel format is guaranteed
	 * to be supported and thus found in formatToMediaBus.
	 */
	ispFormat.code = formatToMediaBus.at(config.pixelFormat);

	ret = resizer_->setFormat(1, &ispFormat);
	if (ret < 0)
		return ret;

	LOG(RkISP1, Debug)
		<< "Configured " << name_ << " resizer output pad with "
		<< ispFormat;

	const PixelFormatInfo &info = PixelFormatInfo::info(config.pixelFormat);
	V4L2DeviceFormat outputFormat;
	outputFormat.fourcc = video_->toV4L2PixelFormat(config.pixelFormat);
	outputFormat.size = config.size;
	outputFormat.planesCount = info.numPlanes();

	ret = video_->setFormat(&outputFormat);
	if (ret)
		return ret;

	if (outputFormat.size != config.size ||
	    outputFormat.fourcc != video_->toV4L2PixelFormat(config.pixelFormat)) {
		LOG(RkISP1, Error)
			<< "Unable to configure capture in " << config.toString();
		return -EINVAL;
	}

	return 0;
}

int RkISP1Path::start()
{
	int ret;

	if (running_)
		return -EBUSY;

	/* \todo Make buffer count user configurable. */
	ret = video_->importBuffers(RKISP1_BUFFER_COUNT);
	if (ret)
		return ret;

	ret = video_->streamOn();
	if (ret) {
		LOG(RkISP1, Error)
			<< "Failed to start " << name_ << " path";

		video_->releaseBuffers();
		return ret;
	}

	running_ = true;

	return 0;
}

void RkISP1Path::stop()
{
	if (!running_)
		return;

	if (video_->streamOff())
		LOG(RkISP1, Warning) << "Failed to stop " << name_ << " path";

	video_->releaseBuffers();

	running_ = false;
}

/*
 * \todo Remove the hardcoded resolutions and formats once kernels older than
 * v6.4 will stop receiving LTS support (scheduled for December 2027 for v6.1).
 */
namespace {
constexpr Size RKISP1_RSZ_MP_SRC_MIN{ 32, 16 };
constexpr Size RKISP1_RSZ_MP_SRC_MAX{ 4416, 3312 };
constexpr std::array<PixelFormat, 18> RKISP1_RSZ_MP_FORMATS{
	formats::YUYV,
	formats::NV16,
	formats::NV61,
	formats::NV21,
	formats::NV12,
	formats::R8,
	formats::SBGGR8,
	formats::SGBRG8,
	formats::SGRBG8,
	formats::SRGGB8,
	formats::SBGGR10,
	formats::SGBRG10,
	formats::SGRBG10,
	formats::SRGGB10,
	formats::SBGGR12,
	formats::SGBRG12,
	formats::SGRBG12,
	formats::SRGGB12,
};

constexpr Size RKISP1_RSZ_SP_SRC_MIN{ 32, 16 };
constexpr Size RKISP1_RSZ_SP_SRC_MAX{ 1920, 1920 };
constexpr std::array<PixelFormat, 8> RKISP1_RSZ_SP_FORMATS{
	formats::YUYV,
	formats::NV16,
	formats::NV61,
	formats::NV21,
	formats::NV12,
	formats::R8,
	formats::RGB565,
	formats::XRGB8888,
};
} /* namespace */

RkISP1MainPath::RkISP1MainPath()
	: RkISP1Path("main", RKISP1_RSZ_MP_FORMATS,
		     RKISP1_RSZ_MP_SRC_MIN, RKISP1_RSZ_MP_SRC_MAX)
{
}

RkISP1SelfPath::RkISP1SelfPath()
	: RkISP1Path("self", RKISP1_RSZ_SP_FORMATS,
		     RKISP1_RSZ_SP_SRC_MIN, RKISP1_RSZ_SP_SRC_MAX)
{
}

} /* namespace libcamera */
