/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2023, Raspberry Pi Ltd
 *
 * pipeline_base.cpp - Pipeline handler base class for Raspberry Pi devices
 */

#include "pipeline_base.h"

#include <chrono>

#include <linux/media-bus-format.h>
#include <linux/videodev2.h>

#include <libcamera/base/file.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>

#include "libcamera/internal/camera_lens.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/v4l2_subdevice.h"

using namespace std::chrono_literals;

namespace libcamera {

using namespace RPi;

LOG_DEFINE_CATEGORY(RPI)

using StreamFlag = RPi::Stream::StreamFlag;

namespace {

constexpr unsigned int defaultRawBitDepth = 12;

bool isRaw(const PixelFormat &pixFmt)
{
	/* This test works for both Bayer and raw mono formats. */
	return BayerFormat::fromPixelFormat(pixFmt).isValid();
}

PixelFormat mbusCodeToPixelFormat(unsigned int mbus_code,
				  BayerFormat::Packing packingReq)
{
	BayerFormat bayer = BayerFormat::fromMbusCode(mbus_code);

	ASSERT(bayer.isValid());

	bayer.packing = packingReq;
	PixelFormat pix = bayer.toPixelFormat();

	/*
	 * Not all formats (e.g. 8-bit or 16-bit Bayer formats) can have packed
	 * variants. So if the PixelFormat returns as invalid, use the non-packed
	 * conversion instead.
	 */
	if (!pix.isValid()) {
		bayer.packing = BayerFormat::Packing::None;
		pix = bayer.toPixelFormat();
	}

	return pix;
}

SensorFormats populateSensorFormats(std::unique_ptr<CameraSensor> &sensor)
{
	SensorFormats formats;

	for (auto const mbusCode : sensor->mbusCodes())
		formats.emplace(mbusCode, sensor->sizes(mbusCode));

	return formats;
}

bool isMonoSensor(std::unique_ptr<CameraSensor> &sensor)
{
	unsigned int mbusCode = sensor->mbusCodes()[0];
	const BayerFormat &bayer = BayerFormat::fromMbusCode(mbusCode);

	return bayer.order == BayerFormat::Order::MONO;
}

double scoreFormat(double desired, double actual)
{
	double score = desired - actual;
	/* Smaller desired dimensions are preferred. */
	if (score < 0.0)
		score = (-score) / 8;
	/* Penalise non-exact matches. */
	if (actual != desired)
		score *= 2;

	return score;
}

V4L2SubdeviceFormat findBestFormat(const SensorFormats &formatsMap, const Size &req, unsigned int bitDepth)
{
	double bestScore = std::numeric_limits<double>::max(), score;
	V4L2SubdeviceFormat bestFormat;
	bestFormat.colorSpace = ColorSpace::Raw;

	constexpr float penaltyAr = 1500.0;
	constexpr float penaltyBitDepth = 500.0;

	/* Calculate the closest/best mode from the user requested size. */
	for (const auto &iter : formatsMap) {
		const unsigned int mbusCode = iter.first;
		const PixelFormat format = mbusCodeToPixelFormat(mbusCode,
								 BayerFormat::Packing::None);
		const PixelFormatInfo &info = PixelFormatInfo::info(format);

		for (const Size &size : iter.second) {
			double reqAr = static_cast<double>(req.width) / req.height;
			double fmtAr = static_cast<double>(size.width) / size.height;

			/* Score the dimensions for closeness. */
			score = scoreFormat(req.width, size.width);
			score += scoreFormat(req.height, size.height);
			score += penaltyAr * scoreFormat(reqAr, fmtAr);

			/* Add any penalties... this is not an exact science! */
			score += utils::abs_diff(info.bitsPerPixel, bitDepth) * penaltyBitDepth;

			if (score <= bestScore) {
				bestScore = score;
				bestFormat.mbus_code = mbusCode;
				bestFormat.size = size;
			}

			LOG(RPI, Debug) << "Format: " << size
					<< " fmt " << format
					<< " Score: " << score
					<< " (best " << bestScore << ")";
		}
	}

	return bestFormat;
}

const std::vector<ColorSpace> validColorSpaces = {
	ColorSpace::Sycc,
	ColorSpace::Smpte170m,
	ColorSpace::Rec709
};

std::optional<ColorSpace> findValidColorSpace(const ColorSpace &colourSpace)
{
	for (auto cs : validColorSpaces) {
		if (colourSpace.primaries == cs.primaries &&
		    colourSpace.transferFunction == cs.transferFunction)
			return cs;
	}

	return std::nullopt;
}

bool isRgb(const PixelFormat &pixFmt)
{
	const PixelFormatInfo &info = PixelFormatInfo::info(pixFmt);
	return info.colourEncoding == PixelFormatInfo::ColourEncodingRGB;
}

bool isYuv(const PixelFormat &pixFmt)
{
	/* The code below would return true for raw mono streams, so weed those out first. */
	if (isRaw(pixFmt))
		return false;

	const PixelFormatInfo &info = PixelFormatInfo::info(pixFmt);
	return info.colourEncoding == PixelFormatInfo::ColourEncodingYUV;
}

} /* namespace */

/*
 * Raspberry Pi drivers expect the following colour spaces:
 * - V4L2_COLORSPACE_RAW for raw streams.
 * - One of V4L2_COLORSPACE_JPEG, V4L2_COLORSPACE_SMPTE170M, V4L2_COLORSPACE_REC709 for
 *   non-raw streams. Other fields such as transfer function, YCbCr encoding and
 *   quantisation are not used.
 *
 * The libcamera colour spaces that we wish to use corresponding to these are therefore:
 * - ColorSpace::Raw for V4L2_COLORSPACE_RAW
 * - ColorSpace::Sycc for V4L2_COLORSPACE_JPEG
 * - ColorSpace::Smpte170m for V4L2_COLORSPACE_SMPTE170M
 * - ColorSpace::Rec709 for V4L2_COLORSPACE_REC709
 */
CameraConfiguration::Status RPiCameraConfiguration::validateColorSpaces([[maybe_unused]] ColorSpaceFlags flags)
{
	Status status = Valid;
	yuvColorSpace_.reset();

	for (auto cfg : config_) {
		/* First fix up raw streams to have the "raw" colour space. */
		if (isRaw(cfg.pixelFormat)) {
			/* If there was no value here, that doesn't count as "adjusted". */
			if (cfg.colorSpace && cfg.colorSpace != ColorSpace::Raw)
				status = Adjusted;
			cfg.colorSpace = ColorSpace::Raw;
			continue;
		}

		/* Next we need to find our shared colour space. The first valid one will do. */
		if (cfg.colorSpace && !yuvColorSpace_)
			yuvColorSpace_ = findValidColorSpace(cfg.colorSpace.value());
	}

	/* If no colour space was given anywhere, choose sYCC. */
	if (!yuvColorSpace_)
		yuvColorSpace_ = ColorSpace::Sycc;

	/* Note the version of this that any RGB streams will have to use. */
	rgbColorSpace_ = yuvColorSpace_;
	rgbColorSpace_->ycbcrEncoding = ColorSpace::YcbcrEncoding::None;
	rgbColorSpace_->range = ColorSpace::Range::Full;

	/* Go through the streams again and force everyone to the same colour space. */
	for (auto cfg : config_) {
		if (cfg.colorSpace == ColorSpace::Raw)
			continue;

		if (isYuv(cfg.pixelFormat) && cfg.colorSpace != yuvColorSpace_) {
			/* Again, no value means "not adjusted". */
			if (cfg.colorSpace)
				status = Adjusted;
			cfg.colorSpace = yuvColorSpace_;
		}
		if (isRgb(cfg.pixelFormat) && cfg.colorSpace != rgbColorSpace_) {
			/* Be nice, and let the YUV version count as non-adjusted too. */
			if (cfg.colorSpace && cfg.colorSpace != yuvColorSpace_)
				status = Adjusted;
			cfg.colorSpace = rgbColorSpace_;
		}
	}

	return status;
}

CameraConfiguration::Status RPiCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	status = validateColorSpaces(ColorSpaceFlag::StreamsShareColorSpace);

	/*
	 * Validate the requested transform against the sensor capabilities and
	 * rotation and store the final combined transform that configure() will
	 * need to apply to the sensor to save us working it out again.
	 */
	Transform requestedTransform = transform;
	combinedTransform_ = data_->sensor_->validateTransform(&transform);
	if (transform != requestedTransform)
		status = Adjusted;

	std::vector<CameraData::StreamParams> rawStreams, outStreams;
	for (const auto &[index, cfg] : utils::enumerate(config_)) {
		if (isRaw(cfg.pixelFormat))
			rawStreams.emplace_back(index, &cfg);
		else
			outStreams.emplace_back(index, &cfg);
	}

	/* Sort the streams so the highest resolution is first. */
	std::sort(rawStreams.begin(), rawStreams.end(),
		  [](auto &l, auto &r) { return l.cfg->size > r.cfg->size; });

	std::sort(outStreams.begin(), outStreams.end(),
		  [](auto &l, auto &r) { return l.cfg->size > r.cfg->size; });

	/* Do any platform specific fixups. */
	status = data_->platformValidate(rawStreams, outStreams);
	if (status == Invalid)
		return Invalid;

	/* Further fixups on the RAW streams. */
	for (auto &raw : rawStreams) {
		StreamConfiguration &cfg = config_.at(raw.index);
		V4L2DeviceFormat rawFormat;

		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
		unsigned int bitDepth = info.isValid() ? info.bitsPerPixel : defaultRawBitDepth;
		V4L2SubdeviceFormat sensorFormat = findBestFormat(data_->sensorFormats_, cfg.size, bitDepth);

		rawFormat.size = sensorFormat.size;
		rawFormat.fourcc = raw.dev->toV4L2PixelFormat(cfg.pixelFormat);

		int ret = raw.dev->tryFormat(&rawFormat);
		if (ret)
			return Invalid;
		/*
		 * Some sensors change their Bayer order when they are h-flipped
		 * or v-flipped, according to the transform. If this one does, we
		 * must advertise the transformed Bayer order in the raw stream.
		 * Note how we must fetch the "native" (i.e. untransformed) Bayer
		 * order, because the sensor may currently be flipped!
		 */
		V4L2PixelFormat fourcc = rawFormat.fourcc;
		if (data_->flipsAlterBayerOrder_) {
			BayerFormat bayer = BayerFormat::fromV4L2PixelFormat(fourcc);
			bayer.order = data_->nativeBayerOrder_;
			bayer = bayer.transform(combinedTransform_);
			fourcc = bayer.toV4L2PixelFormat();
		}

		PixelFormat inputPixFormat = fourcc.toPixelFormat();
		if (raw.cfg->size != rawFormat.size || raw.cfg->pixelFormat != inputPixFormat) {
			raw.cfg->size = rawFormat.size;
			raw.cfg->pixelFormat = inputPixFormat;
			status = Adjusted;
		}

		raw.cfg->stride = rawFormat.planes[0].bpl;
		raw.cfg->frameSize = rawFormat.planes[0].size;
	}

	/* Further fixups on the ISP output streams. */
	for (auto &out : outStreams) {
		StreamConfiguration &cfg = config_.at(out.index);
		PixelFormat &cfgPixFmt = cfg.pixelFormat;
		V4L2VideoDevice::Formats fmts = out.dev->formats();

		if (fmts.find(out.dev->toV4L2PixelFormat(cfgPixFmt)) == fmts.end()) {
			/* If we cannot find a native format, use a default one. */
			cfgPixFmt = formats::NV12;
			status = Adjusted;
		}

		V4L2DeviceFormat format;
		format.fourcc = out.dev->toV4L2PixelFormat(cfg.pixelFormat);
		format.size = cfg.size;
		/* We want to send the associated YCbCr info through to the driver. */
		format.colorSpace = yuvColorSpace_;

		LOG(RPI, Debug)
			<< "Try color space " << ColorSpace::toString(cfg.colorSpace);

		int ret = out.dev->tryFormat(&format);
		if (ret)
			return Invalid;

		/*
		 * But for RGB streams, the YCbCr info gets overwritten on the way back
		 * so we must check against what the stream cfg says, not what we actually
		 * requested (which carefully included the YCbCr info)!
		 */
		if (cfg.colorSpace != format.colorSpace) {
			status = Adjusted;
			LOG(RPI, Debug)
				<< "Color space changed from "
				<< ColorSpace::toString(cfg.colorSpace) << " to "
				<< ColorSpace::toString(format.colorSpace);
		}

		cfg.colorSpace = format.colorSpace;
		cfg.stride = format.planes[0].bpl;
		cfg.frameSize = format.planes[0].size;
	}

	return status;
}

V4L2DeviceFormat PipelineHandlerBase::toV4L2DeviceFormat(const V4L2VideoDevice *dev,
							 const V4L2SubdeviceFormat &format,
							 BayerFormat::Packing packingReq)
{
	unsigned int mbus_code = format.mbus_code;
	const PixelFormat pix = mbusCodeToPixelFormat(mbus_code, packingReq);
	V4L2DeviceFormat deviceFormat;

	deviceFormat.fourcc = dev->toV4L2PixelFormat(pix);
	deviceFormat.size = format.size;
	deviceFormat.colorSpace = format.colorSpace;
	return deviceFormat;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerBase::generateConfiguration(Camera *camera, const StreamRoles &roles)
{
	CameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<RPiCameraConfiguration>(data);
	V4L2SubdeviceFormat sensorFormat;
	unsigned int bufferCount;
	PixelFormat pixelFormat;
	V4L2VideoDevice::Formats fmts;
	Size size;
	std::optional<ColorSpace> colorSpace;

	if (roles.empty())
		return config;

	Size sensorSize = data->sensor_->resolution();
	for (const StreamRole role : roles) {
		switch (role) {
		case StreamRole::Raw:
			size = sensorSize;
			sensorFormat = findBestFormat(data->sensorFormats_, size, defaultRawBitDepth);
			pixelFormat = mbusCodeToPixelFormat(sensorFormat.mbus_code,
							    BayerFormat::Packing::CSI2);
			ASSERT(pixelFormat.isValid());
			colorSpace = ColorSpace::Raw;
			bufferCount = 2;
			break;

		case StreamRole::StillCapture:
			fmts = data->ispFormats();
			pixelFormat = formats::NV12;
			/*
			 * Still image codecs usually expect the sYCC color space.
			 * Even RGB codecs will be fine as the RGB we get with the
			 * sYCC color space is the same as sRGB.
			 */
			colorSpace = ColorSpace::Sycc;
			/* Return the largest sensor resolution. */
			size = sensorSize;
			bufferCount = 1;
			break;

		case StreamRole::VideoRecording:
			/*
			 * The colour denoise algorithm requires the analysis
			 * image, produced by the second ISP output, to be in
			 * YUV420 format. Select this format as the default, to
			 * maximize chances that it will be picked by
			 * applications and enable usage of the colour denoise
			 * algorithm.
			 */
			fmts = data->ispFormats();
			pixelFormat = formats::YUV420;
			/*
			 * Choose a color space appropriate for video recording.
			 * Rec.709 will be a good default for HD resolutions.
			 */
			colorSpace = ColorSpace::Rec709;
			size = { 1920, 1080 };
			bufferCount = 4;
			break;

		case StreamRole::Viewfinder:
			fmts = data->ispFormats();
			pixelFormat = formats::ARGB8888;
			colorSpace = ColorSpace::Sycc;
			size = { 800, 600 };
			bufferCount = 4;
			break;

		default:
			LOG(RPI, Error) << "Requested stream role not supported: "
					<< role;
			return nullptr;
		}

		std::map<PixelFormat, std::vector<SizeRange>> deviceFormats;
		if (role == StreamRole::Raw) {
			/* Translate the MBUS codes to a PixelFormat. */
			for (const auto &format : data->sensorFormats_) {
				PixelFormat pf = mbusCodeToPixelFormat(format.first,
								       BayerFormat::Packing::CSI2);
				if (pf.isValid())
					deviceFormats.emplace(std::piecewise_construct, std::forward_as_tuple(pf),
							      std::forward_as_tuple(format.second.begin(), format.second.end()));
			}
		} else {
			/*
			 * Translate the V4L2PixelFormat to PixelFormat. Note that we
			 * limit the recommended largest ISP output size to match the
			 * sensor resolution.
			 */
			for (const auto &format : fmts) {
				PixelFormat pf = format.first.toPixelFormat();
				if (pf.isValid()) {
					const SizeRange &ispSizes = format.second[0];
					deviceFormats[pf].emplace_back(ispSizes.min, sensorSize,
								       ispSizes.hStep, ispSizes.vStep);
				}
			}
		}

		/* Add the stream format based on the device node used for the use case. */
		StreamFormats formats(deviceFormats);
		StreamConfiguration cfg(formats);
		cfg.size = size;
		cfg.pixelFormat = pixelFormat;
		cfg.colorSpace = colorSpace;
		cfg.bufferCount = bufferCount;
		config->addConfiguration(cfg);
	}

	config->validate();

	return config;
}

int PipelineHandlerBase::configure(Camera *camera, CameraConfiguration *config)
{
	CameraData *data = cameraData(camera);
	int ret;

	/* Start by freeing all buffers and reset the stream states. */
	data->freeBuffers();
	for (auto const stream : data->streams_)
		stream->clearFlags(StreamFlag::External);

	std::vector<CameraData::StreamParams> rawStreams, ispStreams;
	std::optional<BayerFormat::Packing> packing;
	unsigned int bitDepth = defaultRawBitDepth;

	for (unsigned i = 0; i < config->size(); i++) {
		StreamConfiguration *cfg = &config->at(i);

		if (isRaw(cfg->pixelFormat))
			rawStreams.emplace_back(i, cfg);
		else
			ispStreams.emplace_back(i, cfg);
	}

	/* Sort the streams so the highest resolution is first. */
	std::sort(rawStreams.begin(), rawStreams.end(),
		  [](auto &l, auto &r) { return l.cfg->size > r.cfg->size; });

	std::sort(ispStreams.begin(), ispStreams.end(),
		  [](auto &l, auto &r) { return l.cfg->size > r.cfg->size; });

	/*
	 * Calculate the best sensor mode we can use based on the user's request,
	 * and apply it to the sensor with the cached tranform, if any.
	 *
	 * If we have been given a RAW stream, use that size for setting up the sensor.
	 */
	if (!rawStreams.empty()) {
		BayerFormat bayerFormat = BayerFormat::fromPixelFormat(rawStreams[0].cfg->pixelFormat);
		/* Replace the user requested packing/bit-depth. */
		packing = bayerFormat.packing;
		bitDepth = bayerFormat.bitDepth;
	}

	V4L2SubdeviceFormat sensorFormat = findBestFormat(data->sensorFormats_,
							  rawStreams.empty() ? ispStreams[0].cfg->size
									     : rawStreams[0].cfg->size,
							  bitDepth);
	/* Apply any cached transform. */
	const RPiCameraConfiguration *rpiConfig = static_cast<const RPiCameraConfiguration *>(config);

	/* Then apply the format on the sensor. */
	ret = data->sensor_->setFormat(&sensorFormat, rpiConfig->combinedTransform_);
	if (ret)
		return ret;

	/*
	 * Platform specific internal stream configuration. This also assigns
	 * external streams which get configured below.
	 */
	ret = data->platformConfigure(sensorFormat, packing, rawStreams, ispStreams);
	if (ret)
		return ret;

	ipa::RPi::ConfigResult result;
	ret = data->configureIPA(config, &result);
	if (ret) {
		LOG(RPI, Error) << "Failed to configure the IPA: " << ret;
		return ret;
	}

	/*
	 * Set the scaler crop to the value we are using (scaled to native sensor
	 * coordinates).
	 */
	data->scalerCrop_ = data->scaleIspCrop(data->ispCrop_);

	/*
	 * Update the ScalerCropMaximum to the correct value for this camera mode.
	 * For us, it's the same as the "analogue crop".
	 *
	 * \todo Make this property the ScalerCrop maximum value when dynamic
	 * controls are available and set it at validate() time
	 */
	data->properties_.set(properties::ScalerCropMaximum, data->sensorInfo_.analogCrop);

	/* Store the mode sensitivity for the application. */
	data->properties_.set(properties::SensorSensitivity, result.modeSensitivity);

	/* Update the controls that the Raspberry Pi IPA can handle. */
	ControlInfoMap::Map ctrlMap;
	for (auto const &c : result.controlInfo)
		ctrlMap.emplace(c.first, c.second);

	/* Add the ScalerCrop control limits based on the current mode. */
	Rectangle ispMinCrop = data->scaleIspCrop(Rectangle(data->ispMinCropSize_));
	ctrlMap[&controls::ScalerCrop] = ControlInfo(ispMinCrop, data->sensorInfo_.analogCrop, data->scalerCrop_);

	data->controlInfo_ = ControlInfoMap(std::move(ctrlMap), result.controlInfo.idmap());

	/* Setup the Video Mux/Bridge entities. */
	for (auto &[device, link] : data->bridgeDevices_) {
		/*
		 * Start by disabling all the sink pad links on the devices in the
		 * cascade, with the exception of the link connecting the device.
		 */
		for (const MediaPad *p : device->entity()->pads()) {
			if (!(p->flags() & MEDIA_PAD_FL_SINK))
				continue;

			for (MediaLink *l : p->links()) {
				if (l != link)
					l->setEnabled(false);
			}
		}

		/*
		 * Next, enable the entity -> entity links, and setup the pad format.
		 *
		 * \todo Some bridge devices may chainge the media bus code, so we
		 * ought to read the source pad format and propagate it to the sink pad.
		 */
		link->setEnabled(true);
		const MediaPad *sinkPad = link->sink();
		ret = device->setFormat(sinkPad->index(), &sensorFormat);
		if (ret) {
			LOG(RPI, Error) << "Failed to set format on " << device->entity()->name()
					<< " pad " << sinkPad->index()
					<< " with format  " << sensorFormat
					<< ": " << ret;
			return ret;
		}

		LOG(RPI, Debug) << "Configured media link on device " << device->entity()->name()
				<< " on pad " << sinkPad->index();
	}

	return 0;
}

int PipelineHandlerBase::exportFrameBuffers([[maybe_unused]] Camera *camera, libcamera::Stream *stream,
					    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	RPi::Stream *s = static_cast<RPi::Stream *>(stream);
	unsigned int count = stream->configuration().bufferCount;
	int ret = s->dev()->exportBuffers(count, buffers);

	s->setExportedBuffers(buffers);

	return ret;
}

int PipelineHandlerBase::start(Camera *camera, const ControlList *controls)
{
	CameraData *data = cameraData(camera);
	int ret;

	/* Check if a ScalerCrop control was specified. */
	if (controls)
		data->applyScalerCrop(*controls);

	/* Start the IPA. */
	ipa::RPi::StartResult result;
	data->ipa_->start(controls ? *controls : ControlList{ controls::controls },
			  &result);

	/* Apply any gain/exposure settings that the IPA may have passed back. */
	if (!result.controls.empty())
		data->setSensorControls(result.controls);

	/* Configure the number of dropped frames required on startup. */
	data->dropFrameCount_ = data->config_.disableStartupFrameDrops
			      ? 0 : result.dropFrameCount;

	for (auto const stream : data->streams_)
		stream->resetBuffers();

	if (!data->buffersAllocated_) {
		/* Allocate buffers for internal pipeline usage. */
		ret = prepareBuffers(camera);
		if (ret) {
			LOG(RPI, Error) << "Failed to allocate buffers";
			data->freeBuffers();
			stop(camera);
			return ret;
		}
		data->buffersAllocated_ = true;
	}

	/* We need to set the dropFrameCount_ before queueing buffers. */
	ret = queueAllBuffers(camera);
	if (ret) {
		LOG(RPI, Error) << "Failed to queue buffers";
		stop(camera);
		return ret;
	}

	/*
	 * Reset the delayed controls with the gain and exposure values set by
	 * the IPA.
	 */
	data->delayedCtrls_->reset(0);
	data->state_ = CameraData::State::Idle;

	/* Enable SOF event generation. */
	data->frontendDevice()->setFrameStartEnabled(true);

	data->platformStart();

	/* Start all streams. */
	for (auto const stream : data->streams_) {
		ret = stream->dev()->streamOn();
		if (ret) {
			stop(camera);
			return ret;
		}
	}

	return 0;
}

void PipelineHandlerBase::stopDevice(Camera *camera)
{
	CameraData *data = cameraData(camera);

	data->state_ = CameraData::State::Stopped;
	data->platformStop();

	for (auto const stream : data->streams_)
		stream->dev()->streamOff();

	/* Disable SOF event generation. */
	data->frontendDevice()->setFrameStartEnabled(false);

	data->clearIncompleteRequests();

	/* Stop the IPA. */
	data->ipa_->stop();
}

void PipelineHandlerBase::releaseDevice(Camera *camera)
{
	CameraData *data = cameraData(camera);
	data->freeBuffers();
}

int PipelineHandlerBase::queueRequestDevice(Camera *camera, Request *request)
{
	CameraData *data = cameraData(camera);

	if (!data->isRunning())
		return -EINVAL;

	LOG(RPI, Debug) << "queueRequestDevice: New request.";

	/* Push all buffers supplied in the Request to the respective streams. */
	for (auto stream : data->streams_) {
		if (!(stream->getFlags() & StreamFlag::External))
			continue;

		FrameBuffer *buffer = request->findBuffer(stream);
		if (buffer && !stream->getBufferId(buffer)) {
			/*
			 * This buffer is not recognised, so it must have been allocated
			 * outside the v4l2 device. Store it in the stream buffer list
			 * so we can track it.
			 */
			stream->setExternalBuffer(buffer);
		}

		/*
		 * If no buffer is provided by the request for this stream, we
		 * queue a nullptr to the stream to signify that it must use an
		 * internally allocated buffer for this capture request. This
		 * buffer will not be given back to the application, but is used
		 * to support the internal pipeline flow.
		 *
		 * The below queueBuffer() call will do nothing if there are not
		 * enough internal buffers allocated, but this will be handled by
		 * queuing the request for buffers in the RPiStream object.
		 */
		int ret = stream->queueBuffer(buffer);
		if (ret)
			return ret;
	}

	/* Push the request to the back of the queue. */
	data->requestQueue_.push(request);
	data->handleState();

	return 0;
}

int PipelineHandlerBase::registerCamera(std::unique_ptr<RPi::CameraData> &cameraData,
					MediaDevice *frontend, const std::string &frontendName,
					MediaDevice *backend, MediaEntity *sensorEntity)
{
	CameraData *data = cameraData.get();
	int ret;

	data->sensor_ = std::make_unique<CameraSensor>(sensorEntity);
	if (!data->sensor_)
		return -EINVAL;

	if (data->sensor_->init())
		return -EINVAL;

	data->sensorFormats_ = populateSensorFormats(data->sensor_);

	/*
	 * Enumerate all the Video Mux/Bridge devices across the sensor -> Fr
	 * chain. There may be a cascade of devices in this chain!
	 */
	MediaLink *link = sensorEntity->getPadByIndex(0)->links()[0];
	data->enumerateVideoDevices(link, frontendName);

	ipa::RPi::InitResult result;
	if (data->loadIPA(&result)) {
		LOG(RPI, Error) << "Failed to load a suitable IPA library";
		return -EINVAL;
	}

	/*
	 * Setup our delayed control writer with the sensor default
	 * gain and exposure delays. Mark VBLANK for priority write.
	 */
	std::unordered_map<uint32_t, RPi::DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { result.sensorConfig.gainDelay, false } },
		{ V4L2_CID_EXPOSURE, { result.sensorConfig.exposureDelay, false } },
		{ V4L2_CID_HBLANK, { result.sensorConfig.hblankDelay, false } },
		{ V4L2_CID_VBLANK, { result.sensorConfig.vblankDelay, true } }
	};
	data->delayedCtrls_ = std::make_unique<RPi::DelayedControls>(data->sensor_->device(), params);
	data->sensorMetadata_ = result.sensorConfig.sensorMetadata;

	/* Register initial controls that the Raspberry Pi IPA can handle. */
	data->controlInfo_ = std::move(result.controlInfo);

	/* Initialize the camera properties. */
	data->properties_ = data->sensor_->properties();

	/*
	 * The V4L2_CID_NOTIFY_GAINS control, if present, is used to inform the
	 * sensor of the colour gains. It is defined to be a linear gain where
	 * the default value represents a gain of exactly one.
	 */
	auto it = data->sensor_->controls().find(V4L2_CID_NOTIFY_GAINS);
	if (it != data->sensor_->controls().end())
		data->notifyGainsUnity_ = it->second.def().get<int32_t>();

	/*
	 * Set a default value for the ScalerCropMaximum property to show
	 * that we support its use, however, initialise it to zero because
	 * it's not meaningful until a camera mode has been chosen.
	 */
	data->properties_.set(properties::ScalerCropMaximum, Rectangle{});

	/*
	 * We cache two things about the sensor in relation to transforms
	 * (meaning horizontal and vertical flips): if they affect the Bayer
	 * ordering, and what the "native" Bayer order is, when no transforms
	 * are applied.
	 *
	 * If flips are supported verify if they affect the Bayer ordering
	 * and what the "native" Bayer order is, when no transforms are
	 * applied.
	 *
	 * We note that the sensor's cached list of supported formats is
	 * already in the "native" order, with any flips having been undone.
	 */
	const V4L2Subdevice *sensor = data->sensor_->device();
	const struct v4l2_query_ext_ctrl *hflipCtrl = sensor->controlInfo(V4L2_CID_HFLIP);
	if (hflipCtrl) {
		/* We assume it will support vflips too... */
		data->flipsAlterBayerOrder_ = hflipCtrl->flags & V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	}

	/* Look for a valid Bayer format. */
	BayerFormat bayerFormat;
	for (const auto &iter : data->sensorFormats_) {
		bayerFormat = BayerFormat::fromMbusCode(iter.first);
		if (bayerFormat.isValid())
			break;
	}

	if (!bayerFormat.isValid()) {
		LOG(RPI, Error) << "No Bayer format found";
		return -EINVAL;
	}
	data->nativeBayerOrder_ = bayerFormat.order;

	ret = platformRegister(cameraData, frontend, backend);
	if (ret)
		return ret;

	ret = data->loadPipelineConfiguration();
	if (ret) {
		LOG(RPI, Error) << "Unable to load pipeline configuration";
		return ret;
	}

	/* Setup the general IPA signal handlers. */
	data->frontendDevice()->dequeueTimeout.connect(data, &RPi::CameraData::cameraTimeout);
	data->frontendDevice()->frameStart.connect(data, &RPi::CameraData::frameStarted);
	data->ipa_->setDelayedControls.connect(data, &CameraData::setDelayedControls);
	data->ipa_->setLensControls.connect(data, &CameraData::setLensControls);
	data->ipa_->metadataReady.connect(data, &CameraData::metadataReady);

	return 0;
}

void PipelineHandlerBase::mapBuffers(Camera *camera, const BufferMap &buffers, unsigned int mask)
{
	CameraData *data = cameraData(camera);
	std::vector<IPABuffer> bufferIds;
	/*
	 * Link the FrameBuffers with the id (key value) in the map stored in
	 * the RPi stream object - along with an identifier mask.
	 *
	 * This will allow us to identify buffers passed between the pipeline
	 * handler and the IPA.
	 */
	for (auto const &it : buffers) {
		bufferIds.push_back(IPABuffer(mask | it.first,
					      it.second->planes()));
		data->bufferIds_.insert(mask | it.first);
	}

	data->ipa_->mapBuffers(bufferIds);
}

int PipelineHandlerBase::queueAllBuffers(Camera *camera)
{
	CameraData *data = cameraData(camera);
	int ret;

	for (auto const stream : data->streams_) {
		if (!(stream->getFlags() & StreamFlag::External)) {
			ret = stream->queueAllBuffers();
			if (ret < 0)
				return ret;
		} else {
			/*
			 * For external streams, we must queue up a set of internal
			 * buffers to handle the number of drop frames requested by
			 * the IPA. This is done by passing nullptr in queueBuffer().
			 *
			 * The below queueBuffer() call will do nothing if there
			 * are not enough internal buffers allocated, but this will
			 * be handled by queuing the request for buffers in the
			 * RPiStream object.
			 */
			unsigned int i;
			for (i = 0; i < data->dropFrameCount_; i++) {
				ret = stream->queueBuffer(nullptr);
				if (ret)
					return ret;
			}
		}
	}

	return 0;
}

void CameraData::freeBuffers()
{
	if (ipa_) {
		/*
		 * Copy the buffer ids from the unordered_set to a vector to
		 * pass to the IPA.
		 */
		std::vector<unsigned int> bufferIds(bufferIds_.begin(),
						    bufferIds_.end());
		ipa_->unmapBuffers(bufferIds);
		bufferIds_.clear();
	}

	for (auto const stream : streams_)
		stream->releaseBuffers();

	platformFreeBuffers();

	buffersAllocated_ = false;
}

/*
 * enumerateVideoDevices() iterates over the Media Controller topology, starting
 * at the sensor and finishing at the frontend. For each sensor, CameraData stores
 * a unique list of any intermediate video mux or bridge devices connected in a
 * cascade, together with the entity to entity link.
 *
 * Entity pad configuration and link enabling happens at the end of configure().
 * We first disable all pad links on each entity device in the chain, and then
 * selectively enabling the specific links to link sensor to the frontend across
 * all intermediate muxes and bridges.
 *
 * In the cascaded topology below, if Sensor1 is used, the Mux2 -> Mux1 link
 * will be disabled, and Sensor1 -> Mux1 -> Frontend links enabled. Alternatively,
 * if Sensor3 is used, the Sensor2 -> Mux2 and Sensor1 -> Mux1 links are disabled,
 * and Sensor3 -> Mux2 -> Mux1 -> Frontend links are enabled. All other links will
 * remain unchanged.
 *
 *  +----------+
 *  |     FE   |
 *  +-----^----+
 *        |
 *    +---+---+
 *    | Mux1  |<------+
 *    +--^----        |
 *       |            |
 * +-----+---+    +---+---+
 * | Sensor1 |    |  Mux2 |<--+
 * +---------+    +-^-----+   |
 *                  |         |
 *          +-------+-+   +---+-----+
 *          | Sensor2 |   | Sensor3 |
 *          +---------+   +---------+
 */
void CameraData::enumerateVideoDevices(MediaLink *link, const std::string &frontend)
{
	const MediaPad *sinkPad = link->sink();
	const MediaEntity *entity = sinkPad->entity();
	bool frontendFound = false;

	/* We only deal with Video Mux and Bridge devices in cascade. */
	if (entity->function() != MEDIA_ENT_F_VID_MUX &&
	    entity->function() != MEDIA_ENT_F_VID_IF_BRIDGE)
		return;

	/* Find the source pad for this Video Mux or Bridge device. */
	const MediaPad *sourcePad = nullptr;
	for (const MediaPad *pad : entity->pads()) {
		if (pad->flags() & MEDIA_PAD_FL_SOURCE) {
			/*
			 * We can only deal with devices that have a single source
			 * pad. If this device has multiple source pads, ignore it
			 * and this branch in the cascade.
			 */
			if (sourcePad)
				return;

			sourcePad = pad;
		}
	}

	LOG(RPI, Debug) << "Found video mux device " << entity->name()
			<< " linked to sink pad " << sinkPad->index();

	bridgeDevices_.emplace_back(std::make_unique<V4L2Subdevice>(entity), link);
	bridgeDevices_.back().first->open();

	/*
	 * Iterate through all the sink pad links down the cascade to find any
	 * other Video Mux and Bridge devices.
	 */
	for (MediaLink *l : sourcePad->links()) {
		enumerateVideoDevices(l, frontend);
		/* Once we reach the Frontend entity, we are done. */
		if (l->sink()->entity()->name() == frontend) {
			frontendFound = true;
			break;
		}
	}

	/* This identifies the end of our entity enumeration recursion. */
	if (link->source()->entity()->function() == MEDIA_ENT_F_CAM_SENSOR) {
		/*
		 * If the frontend is not at the end of this cascade, we cannot
		 * configure this topology automatically, so remove all entity
		 * references.
		 */
		if (!frontendFound) {
			LOG(RPI, Warning) << "Cannot automatically configure this MC topology!";
			bridgeDevices_.clear();
		}
	}
}

int CameraData::loadPipelineConfiguration()
{
	config_ = {
		.disableStartupFrameDrops = false,
		.cameraTimeoutValue = 0,
	};

	/* Initial configuration of the platform, in case no config file is present */
	platformPipelineConfigure({});

	char const *configFromEnv = utils::secure_getenv("LIBCAMERA_RPI_CONFIG_FILE");
	if (!configFromEnv || *configFromEnv == '\0')
		return 0;

	std::string filename = std::string(configFromEnv);
	File file(filename);

	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(RPI, Error) << "Failed to open configuration file '" << filename << "'";
		return -EIO;
	}

	LOG(RPI, Info) << "Using configuration file '" << filename << "'";

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	if (!root) {
		LOG(RPI, Warning) << "Failed to parse configuration file, using defaults";
		return 0;
	}

	std::optional<double> ver = (*root)["version"].get<double>();
	if (!ver || *ver != 1.0) {
		LOG(RPI, Error) << "Unexpected configuration file version reported";
		return -EINVAL;
	}

	const YamlObject &phConfig = (*root)["pipeline_handler"];

	config_.disableStartupFrameDrops =
		phConfig["disable_startup_frame_drops"].get<bool>(config_.disableStartupFrameDrops);

	config_.cameraTimeoutValue =
		phConfig["camera_timeout_value_ms"].get<unsigned int>(config_.cameraTimeoutValue);

	if (config_.cameraTimeoutValue) {
		/* Disable the IPA signal to control timeout and set the user requested value. */
		ipa_->setCameraTimeout.disconnect();
		frontendDevice()->setDequeueTimeout(config_.cameraTimeoutValue * 1ms);
	}

	return platformPipelineConfigure(root);
}

int CameraData::loadIPA(ipa::RPi::InitResult *result)
{
	int ret;

	ipa_ = IPAManager::createIPA<ipa::RPi::IPAProxyRPi>(pipe(), 1, 1);

	if (!ipa_)
		return -ENOENT;

	/*
	 * The configuration (tuning file) is made from the sensor name unless
	 * the environment variable overrides it.
	 */
	std::string configurationFile;
	char const *configFromEnv = utils::secure_getenv("LIBCAMERA_RPI_TUNING_FILE");
	if (!configFromEnv || *configFromEnv == '\0') {
		std::string model = sensor_->model();
		if (isMonoSensor(sensor_))
			model += "_mono";
		configurationFile = ipa_->configurationFile(model + ".json");
	} else {
		configurationFile = std::string(configFromEnv);
	}

	IPASettings settings(configurationFile, sensor_->model());
	ipa::RPi::InitParams params;

	ret = sensor_->sensorInfo(&params.sensorInfo);
	if (ret) {
		LOG(RPI, Error) << "Failed to retrieve camera sensor info";
		return ret;
	}

	params.lensPresent = !!sensor_->focusLens();
	ret = platformInitIpa(params);
	if (ret)
		return ret;

	return ipa_->init(settings, params, result);
}

int CameraData::configureIPA(const CameraConfiguration *config, ipa::RPi::ConfigResult *result)
{
	std::map<unsigned int, ControlInfoMap> entityControls;
	ipa::RPi::ConfigParams params;
	int ret;

	params.sensorControls = sensor_->controls();
	if (sensor_->focusLens())
		params.lensControls = sensor_->focusLens()->controls();

	ret = platformConfigureIpa(params);
	if (ret)
		return ret;

	/* We store the IPACameraSensorInfo for digital zoom calculations. */
	ret = sensor_->sensorInfo(&sensorInfo_);
	if (ret) {
		LOG(RPI, Error) << "Failed to retrieve camera sensor info";
		return ret;
	}

	/* Always send the user transform to the IPA. */
	params.transform = static_cast<unsigned int>(config->transform);

	/* Ready the IPA - it must know about the sensor resolution. */
	ret = ipa_->configure(sensorInfo_, params, result);
	if (ret < 0) {
		LOG(RPI, Error) << "IPA configuration failed!";
		return -EPIPE;
	}

	if (!result->sensorControls.empty())
		setSensorControls(result->sensorControls);
	if (!result->lensControls.empty())
		setLensControls(result->lensControls);

	return 0;
}

void CameraData::metadataReady(const ControlList &metadata)
{
	if (!isRunning())
		return;

	/* Add to the Request metadata buffer what the IPA has provided. */
	/* Last thing to do is to fill up the request metadata. */
	Request *request = requestQueue_.front();
	request->metadata().merge(metadata);

	/*
	 * Inform the sensor of the latest colour gains if it has the
	 * V4L2_CID_NOTIFY_GAINS control (which means notifyGainsUnity_ is set).
	 */
	const auto &colourGains = metadata.get(libcamera::controls::ColourGains);
	if (notifyGainsUnity_ && colourGains) {
		/* The control wants linear gains in the order B, Gb, Gr, R. */
		ControlList ctrls(sensor_->controls());
		std::array<int32_t, 4> gains{
			static_cast<int32_t>((*colourGains)[1] * *notifyGainsUnity_),
			*notifyGainsUnity_,
			*notifyGainsUnity_,
			static_cast<int32_t>((*colourGains)[0] * *notifyGainsUnity_)
		};
		ctrls.set(V4L2_CID_NOTIFY_GAINS, Span<const int32_t>{ gains });

		sensor_->setControls(&ctrls);
	}
}

void CameraData::setDelayedControls(const ControlList &controls, uint32_t delayContext)
{
	if (!delayedCtrls_->push(controls, delayContext))
		LOG(RPI, Error) << "V4L2 DelayedControl set failed";
}

void CameraData::setLensControls(const ControlList &controls)
{
	CameraLens *lens = sensor_->focusLens();

	if (lens && controls.contains(V4L2_CID_FOCUS_ABSOLUTE)) {
		ControlValue const &focusValue = controls.get(V4L2_CID_FOCUS_ABSOLUTE);
		lens->setFocusPosition(focusValue.get<int32_t>());
	}
}

void CameraData::setSensorControls(ControlList &controls)
{
	/*
	 * We need to ensure that if both VBLANK and EXPOSURE are present, the
	 * former must be written ahead of, and separately from EXPOSURE to avoid
	 * V4L2 rejecting the latter. This is identical to what DelayedControls
	 * does with the priority write flag.
	 *
	 * As a consequence of the below logic, VBLANK gets set twice, and we
	 * rely on the v4l2 framework to not pass the second control set to the
	 * driver as the actual control value has not changed.
	 */
	if (controls.contains(V4L2_CID_EXPOSURE) && controls.contains(V4L2_CID_VBLANK)) {
		ControlList vblank_ctrl;

		vblank_ctrl.set(V4L2_CID_VBLANK, controls.get(V4L2_CID_VBLANK));
		sensor_->setControls(&vblank_ctrl);
	}

	sensor_->setControls(&controls);
}

Rectangle CameraData::scaleIspCrop(const Rectangle &ispCrop) const
{
	/*
	 * Scale a crop rectangle defined in the ISP's coordinates into native sensor
	 * coordinates.
	 */
	Rectangle nativeCrop = ispCrop.scaledBy(sensorInfo_.analogCrop.size(),
						sensorInfo_.outputSize);
	nativeCrop.translateBy(sensorInfo_.analogCrop.topLeft());
	return nativeCrop;
}

void CameraData::applyScalerCrop(const ControlList &controls)
{
	const auto &scalerCrop = controls.get<Rectangle>(controls::ScalerCrop);
	if (scalerCrop) {
		Rectangle nativeCrop = *scalerCrop;

		if (!nativeCrop.width || !nativeCrop.height)
			nativeCrop = { 0, 0, 1, 1 };

		/* Create a version of the crop scaled to ISP (camera mode) pixels. */
		Rectangle ispCrop = nativeCrop.translatedBy(-sensorInfo_.analogCrop.topLeft());
		ispCrop.scaleBy(sensorInfo_.outputSize, sensorInfo_.analogCrop.size());

		/*
		 * The crop that we set must be:
		 * 1. At least as big as ispMinCropSize_, once that's been
		 *    enlarged to the same aspect ratio.
		 * 2. With the same mid-point, if possible.
		 * 3. But it can't go outside the sensor area.
		 */
		Size minSize = ispMinCropSize_.expandedToAspectRatio(nativeCrop.size());
		Size size = ispCrop.size().expandedTo(minSize);
		ispCrop = size.centeredTo(ispCrop.center()).enclosedIn(Rectangle(sensorInfo_.outputSize));

		if (ispCrop != ispCrop_) {
			ispCrop_ = ispCrop;
			platformSetIspCrop();

			/*
			 * Also update the ScalerCrop in the metadata with what we actually
			 * used. But we must first rescale that from ISP (camera mode) pixels
			 * back into sensor native pixels.
			 */
			scalerCrop_ = scaleIspCrop(ispCrop_);
		}
	}
}

void CameraData::cameraTimeout()
{
	LOG(RPI, Error) << "Camera frontend has timed out!";
	LOG(RPI, Error) << "Please check that your camera sensor connector is attached securely.";
	LOG(RPI, Error) << "Alternatively, try another cable and/or sensor.";

	state_ = CameraData::State::Error;
	platformStop();

	/*
	 * To allow the application to attempt a recovery from this timeout,
	 * stop all devices streaming, and return any outstanding requests as
	 * incomplete and cancelled.
	 */
	for (auto const stream : streams_)
		stream->dev()->streamOff();

	clearIncompleteRequests();
}

void CameraData::frameStarted(uint32_t sequence)
{
	LOG(RPI, Debug) << "Frame start " << sequence;

	/* Write any controls for the next frame as soon as we can. */
	delayedCtrls_->applyControls(sequence);
}

void CameraData::clearIncompleteRequests()
{
	/*
	 * All outstanding requests (and associated buffers) must be returned
	 * back to the application.
	 */
	while (!requestQueue_.empty()) {
		Request *request = requestQueue_.front();

		for (auto &b : request->buffers()) {
			FrameBuffer *buffer = b.second;
			/*
			 * Has the buffer already been handed back to the
			 * request? If not, do so now.
			 */
			if (buffer->request()) {
				buffer->_d()->cancel();
				pipe()->completeBuffer(request, buffer);
			}
		}

		pipe()->completeRequest(request);
		requestQueue_.pop();
	}
}

void CameraData::handleStreamBuffer(FrameBuffer *buffer, RPi::Stream *stream)
{
	/*
	 * It is possible to be here without a pending request, so check
	 * that we actually have one to action, otherwise we just return
	 * buffer back to the stream.
	 */
	Request *request = requestQueue_.empty() ? nullptr : requestQueue_.front();
	if (!dropFrameCount_ && request && request->findBuffer(stream) == buffer) {
		/*
		 * Check if this is an externally provided buffer, and if
		 * so, we must stop tracking it in the pipeline handler.
		 */
		handleExternalBuffer(buffer, stream);
		/*
		 * Tag the buffer as completed, returning it to the
		 * application.
		 */
		pipe()->completeBuffer(request, buffer);
	} else {
		/*
		 * This buffer was not part of the Request (which happens if an
		 * internal buffer was used for an external stream, or
		 * unconditionally for internal streams), or there is no pending
		 * request, so we can recycle it.
		 */
		stream->returnBuffer(buffer);
	}
}

void CameraData::handleState()
{
	switch (state_) {
	case State::Stopped:
	case State::Busy:
	case State::Error:
		break;

	case State::IpaComplete:
		/* If the request is completed, we will switch to Idle state. */
		checkRequestCompleted();
		/*
		 * No break here, we want to try running the pipeline again.
		 * The fallthrough clause below suppresses compiler warnings.
		 */
		[[fallthrough]];

	case State::Idle:
		tryRunPipeline();
		break;
	}
}

void CameraData::handleExternalBuffer(FrameBuffer *buffer, RPi::Stream *stream)
{
	unsigned int id = stream->getBufferId(buffer);

	if (!(id & MaskExternalBuffer))
		return;

	/* Stop the Stream object from tracking the buffer. */
	stream->removeExternalBuffer(buffer);
}

void CameraData::checkRequestCompleted()
{
	bool requestCompleted = false;
	/*
	 * If we are dropping this frame, do not touch the request, simply
	 * change the state to IDLE when ready.
	 */
	if (!dropFrameCount_) {
		Request *request = requestQueue_.front();
		if (request->hasPendingBuffers())
			return;

		/* Must wait for metadata to be filled in before completing. */
		if (state_ != State::IpaComplete)
			return;

		pipe()->completeRequest(request);
		requestQueue_.pop();
		requestCompleted = true;
	}

	/*
	 * Make sure we have three outputs completed in the case of a dropped
	 * frame.
	 */
	if (state_ == State::IpaComplete &&
	    ((ispOutputCount_ == ispOutputTotal_ && dropFrameCount_) ||
	     requestCompleted)) {
		state_ = State::Idle;
		if (dropFrameCount_) {
			dropFrameCount_--;
			LOG(RPI, Debug) << "Dropping frame at the request of the IPA ("
					<< dropFrameCount_ << " left)";
		}
	}
}

void CameraData::fillRequestMetadata(const ControlList &bufferControls, Request *request)
{
	request->metadata().set(controls::SensorTimestamp,
				bufferControls.get(controls::SensorTimestamp).value_or(0));

	request->metadata().set(controls::ScalerCrop, scalerCrop_);
}

} /* namespace libcamera */
