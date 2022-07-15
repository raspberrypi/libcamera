/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * rkisp1path.cpp - Rockchip ISP1 path helper
 */

#include "rkisp1_path.h"

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RkISP1)

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

	link_ = media->link("rkisp1_isp", 2, resizer, 0);
	if (!link_)
		return false;

	return true;
}

StreamConfiguration RkISP1Path::generateConfiguration(const Size &resolution)
{
	Size maxResolution = maxResolution_.boundedToAspectRatio(resolution)
					   .boundedTo(resolution);
	Size minResolution = minResolution_.expandedToAspectRatio(resolution);

	std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
	for (const PixelFormat &format : formats_)
		streamFormats[format] = { { minResolution, maxResolution } };

	StreamFormats formats(streamFormats);
	StreamConfiguration cfg(formats);
	cfg.pixelFormat = formats::NV12;
	cfg.size = maxResolution;
	cfg.bufferCount = RKISP1_BUFFER_COUNT;

	return cfg;
}

CameraConfiguration::Status RkISP1Path::validate(StreamConfiguration *cfg)
{
	const StreamConfiguration reqCfg = *cfg;
	CameraConfiguration::Status status = CameraConfiguration::Valid;

	if (std::find(formats_.begin(), formats_.end(), cfg->pixelFormat) ==
	    formats_.end())
		cfg->pixelFormat = formats::NV12;

	cfg->size.boundTo(maxResolution_);
	cfg->size.expandTo(minResolution_);
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

	Rectangle rect(0, 0, ispFormat.size);
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

	switch (config.pixelFormat) {
	case formats::NV12:
	case formats::NV21:
		ispFormat.mbus_code = MEDIA_BUS_FMT_YUYV8_1_5X8;
		break;
	default:
		ispFormat.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8;
		break;
	}

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

namespace {
constexpr Size RKISP1_RSZ_MP_SRC_MIN{ 32, 16 };
constexpr Size RKISP1_RSZ_MP_SRC_MAX{ 4416, 3312 };
constexpr std::array<PixelFormat, 6> RKISP1_RSZ_MP_FORMATS{
	formats::YUYV,
	formats::NV16,
	formats::NV61,
	formats::NV21,
	formats::NV12,
	formats::R8,
	/* \todo Add support for RAW formats. */
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
