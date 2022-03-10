/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Ltd.
 *
 * raspberrypi.cpp - Pipeline handler for Raspberry Pi devices
 */
#include <algorithm>
#include <assert.h>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_set>
#include <utility>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/ipa/raspberrypi.h>
#include <libcamera/ipa/pisp_ipa_interface.h>
#include <libcamera/ipa/pisp_ipa_proxy.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>

#include <linux/bcm2835-isp.h>
#include <linux/media-bus-format.h>
#include <linux/videodev2.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "backend/backend.h"
#include "frontend/frontend.h"
#include "variants/pisp_variant.h"

#include "dma_heaps.h"
#include "rpi_stream.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(PISP)

namespace {

constexpr unsigned int defaultRawBitDepth = 12;

/* Map of mbus codes to supported sizes reported by the sensor. */
using SensorFormats = std::map<unsigned int, std::vector<Size>>;

SensorFormats populateSensorFormats(std::unique_ptr<CameraSensor> &sensor)
{
	SensorFormats formats;

	for (auto const mbusCode : sensor->mbusCodes())
		formats.emplace(mbusCode, sensor->sizes(mbusCode));

	return formats;
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

V4L2DeviceFormat toV4L2DeviceFormat(const V4L2SubdeviceFormat &format,
				    BayerFormat::Packing packingReq)
{
	const PixelFormat pix = mbusCodeToPixelFormat(format.mbus_code, packingReq);
	V4L2DeviceFormat deviceFormat;

	deviceFormat.fourcc = V4L2PixelFormat::fromPixelFormat(pix);
	deviceFormat.size = format.size;
	deviceFormat.colorSpace = format.colorSpace;
	return deviceFormat;
}

bool isRaw(const PixelFormat &pixFmt)
{
	/*
	 * The isRaw test might be redundant right now the pipeline handler only
	 * supports RAW sensors. Leave it in for now, just as a sanity check.
	 */
	if (!pixFmt.isValid())
		return false;

	const PixelFormatInfo &info = PixelFormatInfo::info(pixFmt);
	if (!info.isValid())
		return false;

	return info.colourEncoding == PixelFormatInfo::ColourEncodingRAW;
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

			LOG(PISP, Debug) << "Format: " << size.toString()
					 << " fmt " << format.toString()
					 << " Score: " << score
					 << " (best " << bestScore << ")";
		}
	}

	return bestFormat;
}

enum class Cfe : unsigned int { Output0, Embedded, Stats, Config, Max };
enum class Isp : unsigned int { Input, Output0, Output1, Config, Max };

} /* namespace */

using ::PiSP::BackEnd;
using ::PiSP::FrontEnd;
using ::PiSP::BCM2712_HW;

class PiSPCameraData : public Camera::Private
{
public:
	PiSPCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe),
		  fe_(true, BCM2712_HW),
		  be_(BackEnd::Config(0, 0, BackEnd::Config::Flags::NONE), BCM2712_HW),
		  state_(State::Stopped), supportsFlips_(false), flipsAlterBayerOrder_(false),
		  dropFrameCount_(0), ispOutputCount_(0)
	{
	}

	void initPiSP();

	void frameStarted(uint32_t sequence);

	int loadIPA(ipa::PiSP::SensorConfig *sensorConfig);
	int configureIPA(const CameraConfiguration *config);
	int configureCfe();
	int configureBe();

	void enumerateVideoDevices(MediaLink *link);

	void statsMetadataComplete(uint32_t bufferId, const ControlList &controls);
	void prepareCfe();
	void runBackend(uint32_t bufferId);
	void embeddedComplete(uint32_t bufferId);
	void setIspControls(const ControlList &controls);
	void setDelayedControls(const ControlList &controls);
	void setSensorControls(ControlList &controls);

	/* bufferComplete signal handlers. */
	void cfeBufferDequeue(FrameBuffer *buffer);
	void ispInputDequeue(FrameBuffer *buffer);
	void ispOutputDequeue(FrameBuffer *buffer);

	void clearIncompleteRequests();
	void handleStreamBuffer(FrameBuffer *buffer, PiSP::Stream *stream);
	void handleExternalBuffer(FrameBuffer *buffer, PiSP::Stream *stream);
	void handleState();
	void applyScalerCrop(const ControlList &controls);

	std::unique_ptr<ipa::PiSP::IPAProxyPiSP> ipa_;

	std::unique_ptr<CameraSensor> sensor_;
	SensorFormats sensorFormats_;
	/* Array of CFE and ISP device streams and associated buffers/streams. */
	PiSP::Device<Cfe> cfe_;
	PiSP::Device<Isp> isp_;
	/* The vector below is just for convenience when iterating over all streams. */
	std::vector<PiSP::Stream *> streams_;
	/* Stores the ids of the buffers mapped in the IPA. */
	std::unordered_set<unsigned int> ipaBuffers_;

	std::map<unsigned int, MappedFrameBuffer> feConfigBuffers_;
	std::map<unsigned int, MappedFrameBuffer> beConfigBuffers_;

	/*
	 * Stores a cascade of Video Mux or Bridge devices between the sensor and
	 * Unicam together with media link across the entities.
	 */
	std::vector<std::pair<std::unique_ptr<V4L2Subdevice>, MediaLink *>> bridgeDevices_;

	RPi::DmaHeapObject<FrontEnd> fe_;
	RPi::DmaHeapObject<BackEnd> be_;

	std::unique_ptr<DelayedControls> delayedCtrls_;
	bool sensorMetadata_;

	/*
	 * All the functions in this class are called from a single calling
	 * thread. So, we do not need to have any mutex to protect access to any
	 * of the variables below.
	 */
	enum class State { Stopped, Idle, Busy, IpaComplete };
	State state_;

	struct BayerFrame {
		FrameBuffer *buffer;
		ControlList controls;
	};

	std::queue<BayerFrame> bayerQueue_;
	std::queue<FrameBuffer *> embeddedQueue_;
	std::queue<Request *> requestQueue_;

	/*
	 * Manage horizontal and vertical flips supported (or not) by the
	 * sensor. Also store the "native" Bayer order (that is, with no
	 * transforms applied).
	 */
	bool supportsFlips_;
	bool flipsAlterBayerOrder_;
	BayerFormat::Order nativeBayerOrder_;

	/* For handling digital zoom. */
	IPACameraSensorInfo sensorInfo_;
	Rectangle ispCrop_; /* crop in ISP (camera mode) pixels */
	Rectangle scalerCrop_; /* crop in sensor native pixels */
	Size ispMinCropSize_;

	unsigned int dropFrameCount_;

	/*
	 * If set, this stores the value that represets a gain of one for
	 * the V4L2_CID_NOTIFY_GAINS control.
	 */
	std::optional<int32_t> notifyGainsUnity_;

private:
	void checkRequestCompleted();
	void fillRequestMetadata(const ControlList &bufferControls,
				 Request *request);
	void tryRunPipeline();
	bool findMatchingBuffers(BayerFrame &bayerFrame, FrameBuffer *&embeddedBuffer);

	unsigned int ispOutputCount_;
};

class PiSPCameraConfiguration : public CameraConfiguration
{
public:
	PiSPCameraConfiguration(const PiSPCameraData *data)
		: CameraConfiguration(), data_(data)
	{
	}

	Status validate() override;

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;

private:
	const PiSPCameraData *data_;
};

class PipelineHandlerPiSP : public PipelineHandler
{
public:
	PipelineHandlerPiSP(CameraManager *manager)
		: PipelineHandler(manager)
	{
	}

	CameraConfiguration *generateConfiguration(Camera *camera, const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	PiSPCameraData *cameraData(Camera *camera)
	{
		return static_cast<PiSPCameraData *>(camera->_d());
	}

	int registerCamera(MediaDevice *cfe, MediaDevice *isp, MediaEntity *sensorEntity);
	int queueAllBuffers(Camera *camera);
	int prepareBuffers(Camera *camera);
	void freeBuffers(Camera *camera);
	void mapBuffers(Camera *camera, const PiSP::BufferMap &buffers, unsigned int mask);
};

CameraConfiguration::Status PiSPCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	status = validateColorSpaces(ColorSpaceFlag::StreamsShareColorSpace);

	/*
	 * What if the platform has a non-90 degree rotation? We can't even
	 * "adjust" the configuration and carry on. Alternatively, raising an
	 * error means the platform can never run. Let's just print a warning
	 * and continue regardless; the rotation is effectively set to zero.
	 */
	int32_t rotation = data_->sensor_->properties().get(properties::Rotation);
	bool success;
	Transform rotationTransform = transformFromRotation(rotation, &success);
	if (!success)
		LOG(PISP, Warning) << "Invalid rotation of " << rotation
				  << " degrees - ignoring";
	Transform combined = transform * rotationTransform;

	/*
	 * We combine the platform and user transform, but must "adjust away"
	 * any combined result that includes a transform, as we can't do those.
	 * In this case, flipping only the transpose bit is helpful to
	 * applications - they either get the transform they requested, or have
	 * to do a simple transpose themselves (they don't have to worry about
	 * the other possible cases).
	 */
	if (!!(combined & Transform::Transpose)) {
		/*
		 * Flipping the transpose bit in "transform" flips it in the
		 * combined result too (as it's the last thing that happens),
		 * which is of course clearing it.
		 */
		transform ^= Transform::Transpose;
		combined &= ~Transform::Transpose;
		status = Adjusted;
	}

	/*
	 * We also check if the sensor doesn't do h/vflips at all, in which
	 * case we clear them, and the application will have to do everything.
	 */
	if (!data_->supportsFlips_ && !!combined) {
		/*
		 * If the sensor can do no transforms, then combined must be
		 * changed to the identity. The only user transform that gives
		 * rise to this the inverse of the rotation. (Recall that
		 * combined = transform * rotationTransform.)
		 */
		transform = -rotationTransform;
		combined = Transform::Identity;
		status = Adjusted;
	}

	/*
	 * Store the final combined transform that configure() will need to
	 * apply to the sensor to save us working it out again.
	 */
	combinedTransform_ = combined;

	unsigned int rawCount = 0, outCount = 0, count = 0, maxIndex = 0;
	std::pair<int, Size> outSize[2];
	Size maxSize;
	for (StreamConfiguration &cfg : config_) {
		if (isRaw(cfg.pixelFormat)) {
			/*
			 * Calculate the best sensor mode we can use based on
			 * the user request.
			 */
			const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
			unsigned int bitDepth = info.isValid() ? info.bitsPerPixel : defaultRawBitDepth;
			V4L2SubdeviceFormat sensorFormat = findBestFormat(data_->sensorFormats_, cfg.size, bitDepth);
			BayerFormat::Packing packing = BayerFormat::Packing::CSI2;
			//if (info.isValid() && !info.packed)
			//	packing = BayerFormat::Packing::None;
			V4L2DeviceFormat cfeFormat = toV4L2DeviceFormat(sensorFormat,
									   packing);
			int ret = data_->cfe_[Cfe::Output0].dev()->tryFormat(&cfeFormat);
			if (ret)
				return Invalid;

			/*
			 * Some sensors change their Bayer order when they are
			 * h-flipped or v-flipped, according to the transform.
			 * If this one does, we must advertise the transformed
			 * Bayer order in the raw stream. Note how we must
			 * fetch the "native" (i.e. untransformed) Bayer order,
			 * because the sensor may currently be flipped!
			 */
			V4L2PixelFormat fourcc = cfeFormat.fourcc;
			if (data_->flipsAlterBayerOrder_) {
				BayerFormat bayer = BayerFormat::fromV4L2PixelFormat(fourcc);
				bayer.order = data_->nativeBayerOrder_;
				bayer = bayer.transform(combined);
				fourcc = bayer.toV4L2PixelFormat();
			}

			PixelFormat cfePixFormat = fourcc.toPixelFormat();
			if (cfg.size != cfeFormat.size ||
			    cfg.pixelFormat != cfePixFormat) {
				cfg.size = cfeFormat.size;
				cfg.pixelFormat = cfePixFormat;
				status = Adjusted;
			}

			cfg.stride = cfeFormat.planes[0].bpl;
			cfg.frameSize = cfeFormat.planes[0].size;

			rawCount++;
		} else {
			outSize[outCount] = std::make_pair(count, cfg.size);
			/* Record the largest resolution for fixups later. */
			if (maxSize < cfg.size) {
				maxSize = cfg.size;
				maxIndex = outCount;
			}
			outCount++;
		}

		count++;

		/* Can only output 1 RAW stream, or 2 YUV/RGB streams. */
		if (rawCount > 1 || outCount > 2) {
			LOG(PISP, Error) << "Invalid number of streams requested";
			return Invalid;
		}
	}

	/*
	 * Now do any fixups needed. For the two ISP outputs, one stream must be
	 * equal or smaller than the other in all dimensions.
	 */
	for (unsigned int i = 0; i < outCount; i++) {
		outSize[i].second.width = std::min(outSize[i].second.width,
						   maxSize.width);
		outSize[i].second.height = std::min(outSize[i].second.height,
						    maxSize.height);

		if (config_.at(outSize[i].first).size != outSize[i].second) {
			config_.at(outSize[i].first).size = outSize[i].second;
			status = Adjusted;
		}

		/*
		 * Also validate the correct pixel formats here.
		 * Note that Output0 and Output1 support a different
		 * set of formats.
		 *
		 * Output 0 must be for the largest resolution. We will
		 * have that fixed up in the code above.
		 *
		 */
		StreamConfiguration &cfg = config_.at(outSize[i].first);
		PixelFormat &cfgPixFmt = cfg.pixelFormat;
		V4L2VideoDevice *dev;

		if (i == maxIndex)
			dev = data_->isp_[Isp::Output0].dev();
		else
			dev = data_->isp_[Isp::Output1].dev();

		V4L2VideoDevice::Formats fmts = dev->formats();

		if (fmts.find(V4L2PixelFormat::fromPixelFormat(cfgPixFmt)) == fmts.end()) {
			/* If we cannot find a native format, use a default one. */
			cfgPixFmt = formats::NV12;
			status = Adjusted;
		}

		V4L2DeviceFormat format;
		format.fourcc = V4L2PixelFormat::fromPixelFormat(cfg.pixelFormat);
		format.size = cfg.size;
		format.colorSpace = cfg.colorSpace;

		LOG(PISP, Debug)
			<< "Try color space " << ColorSpace::toString(cfg.colorSpace);

		int ret = dev->tryFormat(&format);
		if (ret)
			return Invalid;

		if (cfg.colorSpace != format.colorSpace) {
			status = Adjusted;
			LOG(PISP, Debug)
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


CameraConfiguration *PipelineHandlerPiSP::generateConfiguration(Camera *camera,
							       const StreamRoles &roles)
{
	PiSPCameraData *data = cameraData(camera);
	CameraConfiguration *config = new PiSPCameraConfiguration(data);
	V4L2SubdeviceFormat sensorFormat;
	unsigned int bufferCount;
	PixelFormat pixelFormat;
	V4L2VideoDevice::Formats fmts;
	Size size;
	std::optional<ColorSpace> colorSpace;

	if (roles.empty())
		return config;

	unsigned int rawCount = 0;
	unsigned int outCount = 0;
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
			rawCount++;
			break;

		case StreamRole::StillCapture:
			fmts = data->isp_[Isp::Output0].dev()->formats();
			pixelFormat = formats::NV12;
			/*
			 * Still image codecs usually expect the JPEG color space.
			 * Even RGB codecs will be fine as the RGB we get with the
			 * JPEG color space is the same as sRGB.
			 */
			colorSpace = ColorSpace::Jpeg;
			/* Return the largest sensor resolution. */
			size = sensorSize;
			bufferCount = 1;
			outCount++;
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
			fmts = data->isp_[Isp::Output0].dev()->formats();
			pixelFormat = formats::YUV420;
			/*
			 * Choose a color space appropriate for video recording.
			 * Rec.709 will be a good default for HD resolutions.
			 */
			colorSpace = ColorSpace::Rec709;
			size = { 1920, 1080 };
			bufferCount = 4;
			outCount++;
			break;

		case StreamRole::Viewfinder:
			fmts = data->isp_[Isp::Output0].dev()->formats();
			pixelFormat = formats::ARGB8888;
			colorSpace = ColorSpace::Jpeg;
			size = { 800, 600 };
			bufferCount = 4;
			outCount++;
			break;

		default:
			LOG(PISP, Error) << "Requested stream role not supported: "
					 << role;
			delete config;
			return nullptr;
		}

		if (rawCount > 1 || outCount > 2) {
			LOG(PISP, Error) << "Invalid stream roles requested";
			delete config;
			return nullptr;
		}

		std::map<PixelFormat, std::vector<SizeRange>> deviceFormats;
		if (role == StreamRole::Raw) {
			/* Translate the MBUS codes to a PixelFormat. */
			for (const auto &format : data->sensorFormats_) {
				PixelFormat pf = mbusCodeToPixelFormat(format.first,
								       BayerFormat::Packing::CSI2);
				if (pf.isValid())
					deviceFormats.emplace(std::piecewise_construct,	std::forward_as_tuple(pf),
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

int PipelineHandlerPiSP::configure(Camera *camera, CameraConfiguration *config)
{
	PiSPCameraData *data = cameraData(camera);
	int ret;

	/* Start by resetting the CFE and ISP stream states. */
	for (auto const stream : data->streams_)
		stream->reset();

	BayerFormat::Packing packing = BayerFormat::Packing::CSI2;
	Size maxSize, sensorSize;
	unsigned int maxIndex = 0;
	bool rawStream = false;
	unsigned int bitDepth = defaultRawBitDepth;

	/*
	 * Look for the RAW stream (if given) size as well as the largest
	 * ISP output size.
	 */
	for (unsigned i = 0; i < config->size(); i++) {
		StreamConfiguration &cfg = config->at(i);

		if (isRaw(cfg.pixelFormat)) {
			/*
			 * If we have been given a RAW stream, use that size
			 * for setting up the sensor.
			 */
			sensorSize = cfg.size;
			rawStream = true;
			/* Check if the user has explicitly set an unpacked format. */
			BayerFormat bayerFormat = BayerFormat::fromPixelFormat(cfg.pixelFormat);
			packing = bayerFormat.packing;
			bitDepth = bayerFormat.bitDepth;
		} else {
			if (cfg.size > maxSize) {
				maxSize = config->at(i).size;
				maxIndex = i;
			}
		}
	}

	/*
	 * Configure the H/V flip controls based on the combination of
	 * the sensor and user transform.
	 */
	if (data->supportsFlips_) {
		const PiSPCameraConfiguration *rpiConfig =
			static_cast<const PiSPCameraConfiguration *>(config);
		ControlList controls;

		controls.set(V4L2_CID_HFLIP,
			     static_cast<int32_t>(!!(rpiConfig->combinedTransform_ & Transform::HFlip)));
		controls.set(V4L2_CID_VFLIP,
			     static_cast<int32_t>(!!(rpiConfig->combinedTransform_ & Transform::VFlip)));
		data->setSensorControls(controls);
	}

	/* First calculate the best sensor mode we can use based on the user request. */
	V4L2SubdeviceFormat sensorFormat = findBestFormat(data->sensorFormats_, rawStream ? sensorSize : maxSize, bitDepth);
	ret = data->sensor_->setFormat(&sensorFormat);
	if (ret)
		return ret;

	V4L2DeviceFormat cfeFormat = toV4L2DeviceFormat(sensorFormat, packing);
	ret = data->cfe_[Cfe::Output0].dev()->setFormat(&cfeFormat);
	if (ret)
		return ret;

	LOG(PISP, Info) << "Sensor: " << camera->id()
		        << " - Selected sensor format: " << sensorFormat.toString()
		        << " - Selected cfe format: " << cfeFormat.toString();

	ret = data->isp_[Isp::Input].dev()->setFormat(&cfeFormat);
	if (ret)
		return ret;

	/*
	 * See which streams are requested, and route the user
	 * StreamConfiguration appropriately.
	 */
	V4L2DeviceFormat format;
	bool output0Set = false, output1Set = false;
	for (unsigned i = 0; i < config->size(); i++) {
		StreamConfiguration &cfg = config->at(i);

		if (isRaw(cfg.pixelFormat)) {
			cfg.setStream(&data->cfe_[Cfe::Output0]);
			data->cfe_[Cfe::Output0].setExternal(true);
			continue;
		}

		/* The largest resolution gets routed to the ISP Output 0 node. */
		PiSP::Stream *stream = i == maxIndex ? &data->isp_[Isp::Output0]
						    : &data->isp_[Isp::Output1];

		V4L2PixelFormat fourcc = V4L2PixelFormat::fromPixelFormat(cfg.pixelFormat);
		format.size = cfg.size;
		format.fourcc = fourcc;
		format.colorSpace = cfg.colorSpace;

		LOG(PISP, Debug) << "Setting " << stream->name() << " to "
				 << format.toString();

		ret = stream->dev()->setFormat(&format);
		if (ret)
			return -EINVAL;

		if (format.size != cfg.size || format.fourcc != fourcc) {
			LOG(PISP, Error)
				<< "Failed to set requested format on " << stream->name()
				<< ", returned " << format.toString();
			return -EINVAL;
		}

		LOG(PISP, Debug)
			<< "Stream " << stream->name() << " has color space "
			<< ColorSpace::toString(cfg.colorSpace);


		cfg.setStream(stream);
		stream->setExternal(true);

		if (i != maxIndex)
			output1Set = true;
		else
			output0Set = true;
	}

	/*
	 * If ISP::Output0 stream has not been configured by the application,
	 * we must allow the hardware to generate an output so that the data
	 * flow in the pipeline handler remains consistent, and we still generate
	 * statistics for the IPA to use. So enable the output at a very low
	 * resolution for internal use.
	 *
	 * \todo Allow the pipeline to work correctly without Output0 and only
	 * statistics coming from the hardware.
	 */
	if (!output0Set) {
		maxSize = Size(320, 240);
		format = {};
		format.size = maxSize;
		format.fourcc = V4L2PixelFormat::fromPixelFormat(formats::YUV420);
		/* No one asked for output, so the color space doesn't matter. */
		format.colorSpace = ColorSpace::Jpeg;
		ret = data->isp_[Isp::Output0].dev()->setFormat(&format);
		if (ret) {
			LOG(PISP, Error)
				<< "Failed to set default format on ISP Output0: "
				<< ret;
			return -EINVAL;
		}

		LOG(PISP, Debug) << "Defaulting ISP Output0 format to "
				 << format.toString();
	}

	/*
	 * If ISP::Output1 stream has not been requested by the application, we
	 * set it up for internal use now. This second stream will be used for
	 * fast colour denoise, and must be a quarter resolution of the ISP::Output0
	 * stream. However, also limit the maximum size to 1200 pixels in the
	 * larger dimension, just to avoid being wasteful with buffer allocations
	 * and memory bandwidth.
	 *
	 * \todo If Output 1 format is not YUV420, Output 1 ought to be disabled as
	 * colour denoise will not run.
	 */
	if (!output1Set) {
		V4L2DeviceFormat output1Format = format;
/*
		constexpr Size maxDimensions(1200, 1200);
		const Size limit = maxDimensions.boundedToAspectRatio(format.size);

		output1Format.size = (format.size / 2).boundedTo(limit).alignedDownTo(2, 2);
		output1Format.fourcc = format.fourcc;
		output1Format.colorSpace = format.colorSpace;
*/
		LOG(PISP, Debug) << "Setting ISP Output1 (internal) to "
				<< output1Format.toString();

		ret = data->isp_[Isp::Output1].dev()->setFormat(&output1Format);
		if (ret) {
			LOG(PISP, Error) << "Failed to set format on ISP Output1: "
					<< ret;
			return -EINVAL;
		}
	}

	/* CFE statistics output format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_RPI_FE_STATS);
	ret = data->cfe_[Cfe::Stats].dev()->setFormat(&format);
	if (ret) {
		LOG(PISP, Error) << "Failed to set format on CFE stats stream: "
				 << format.toString();
		return ret;
	}

	/* CFE config format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_RPI_FE_CFG);
	ret = data->cfe_[Cfe::Config].dev()->setFormat(&format);
	if (ret) {
		LOG(PISP, Error) << "Failed to set format on CFE config stream: "
				 << format.toString();
		return ret;
	}

	/* Figure out the smallest selection the ISP will allow. */
	Rectangle testCrop(0, 0, 1, 1);
	//data->isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &testCrop);
	data->ispMinCropSize_ = testCrop.size();

	/* Adjust aspect ratio by providing crops on the input image. */
	Size size = cfeFormat.size.boundedToAspectRatio(maxSize);
	Rectangle crop = size.centeredTo(Rectangle(cfeFormat.size).center());
	data->ispCrop_ = crop;

	//data->isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &crop);

	ret = data->configureIPA(config);
	if (ret)
		LOG(PISP, Error) << "Failed to configure the IPA: " << ret;

	/*
	 * Configure the CFE embedded data output format only if the sensor
	 * supports it.
	 */
	if (data->sensorMetadata_) {
		V4L2SubdeviceFormat embeddedFormat;

		data->sensor_->device()->getFormat(1, &embeddedFormat);
		format.fourcc = V4L2PixelFormat(V4L2_META_FMT_SENSOR_DATA);
		format.planes[0].size = embeddedFormat.size.width * embeddedFormat.size.height;

		LOG(PISP, Debug) << "Setting embedded data format.";
		ret = data->cfe_[Cfe::Embedded].dev()->setFormat(&format);
		if (ret) {
			LOG(PISP, Error) << "Failed to set format on CFE embedded: "
					<< format.toString();
			return ret;
		}
	}

	/*
	 * Update the ScalerCropMaximum to the correct value for this camera mode.
	 * For us, it's the same as the "analogue crop".
	 *
	 * \todo Make this property the ScalerCrop maximum value when dynamic
	 * controls are available and set it at validate() time
	 */
	data->properties_.set(properties::ScalerCropMaximum, data->sensorInfo_.analogCrop);

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
			LOG(PISP, Error) << "Failed to set format on " << device->entity()->name()
					 << " pad " << sinkPad->index()
					 << " with format  " << format.toString()
					 << ": " << ret;
			return ret;
		}

		LOG(PISP, Debug) << "Configured media link on device " << device->entity()->name()
				 << " on pad " << sinkPad->index();
	}

	data->configureCfe();
	data->configureBe();

	return ret;
}

int PipelineHandlerPiSP::exportFrameBuffers([[maybe_unused]] Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	PiSP::Stream *s = static_cast<PiSP::Stream *>(stream);
	unsigned int count = stream->configuration().bufferCount;
	int ret = s->dev()->exportBuffers(count, buffers);

	s->setExportedBuffers(buffers);

	return ret;
}

int PipelineHandlerPiSP::start(Camera *camera, const ControlList *controls)
{
	PiSPCameraData *data = cameraData(camera);
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = prepareBuffers(camera);
	if (ret) {
		LOG(PISP, Error) << "Failed to allocate buffers";
		stop(camera);
		return ret;
	}

	/* Check if a ScalerCrop control was specified. */
	if (controls)
		data->applyScalerCrop(*controls);

	/* Start the IPA. */
	ipa::PiSP::StartConfig startConfig;
	data->ipa_->start(controls ? *controls : ControlList{ controls::controls },
			  &startConfig);

	/* Apply any gain/exposure settings that the IPA may have passed back. */
	if (!startConfig.controls.empty())
		data->setSensorControls(startConfig.controls);

	/* Configure the number of dropped frames required on startup. */
	data->dropFrameCount_ = startConfig.dropFrameCount;

	/* We need to set the dropFrameCount_ before queueing buffers. */
	ret = queueAllBuffers(camera);
	if (ret) {
		LOG(PISP, Error) << "Failed to queue buffers";
		stop(camera);
		return ret;
	}

	/* Enable SOF event generation. */
	data->cfe_[Cfe::Output0].dev()->setFrameStartEnabled(true);

	/*
	 * Reset the delayed controls with the gain and exposure values set by
	 * the IPA.
	 */
	data->delayedCtrls_->reset();

	data->prepareCfe();
	data->state_ = PiSPCameraData::State::Idle;

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

void PipelineHandlerPiSP::stopDevice(Camera *camera)
{
	PiSPCameraData *data = cameraData(camera);

	data->state_ = PiSPCameraData::State::Stopped;

	/* Disable SOF event generation. */
	data->cfe_[Cfe::Output0].dev()->setFrameStartEnabled(false);

	for (auto const stream : data->streams_)
		stream->dev()->streamOff();

	data->clearIncompleteRequests();
	data->bayerQueue_ = {};
	data->embeddedQueue_ = {};

	/* Stop the IPA. */
	//data->ipa_->stop();

	freeBuffers(camera);
}

int PipelineHandlerPiSP::queueRequestDevice(Camera *camera, Request *request)
{
	PiSPCameraData *data = cameraData(camera);

	if (data->state_ == PiSPCameraData::State::Stopped)
		return -EINVAL;

	LOG(PISP, Debug) << "queueRequestDevice: New request.";

	/* Push all buffers supplied in the Request to the respective streams. */
	for (auto stream : data->streams_) {
		if (!stream->isExternal())
			continue;

		FrameBuffer *buffer = request->findBuffer(stream);
		if (buffer && stream->getBufferId(buffer) == -1) {
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
		 * queuing the request for buffers in the PiSPStream object.
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

bool PipelineHandlerPiSP::match(DeviceEnumerator *enumerator)
{
	DeviceMatch cfe("py-cfe");
	MediaDevice *cfeDevice = acquireMediaDevice(enumerator, cfe);

	if (!cfeDevice) {
		LOG(PISP, Debug) << "Unable to acquire a CFE instance";
		return false;
	}

	DeviceMatch isp("pispbe");
	MediaDevice *ispDevice = acquireMediaDevice(enumerator, isp);

	if (!ispDevice) {
		LOG(PISP, Debug) << "Unable to acquire ISP instance";
		return false;
	}

	/*
	 * The loop below is used to register multiple cameras behind one or more
	 * video mux devices that are attached to a particular CFE instance.
	 * Obviously these cameras cannot be used simultaneously.
	 */
	unsigned int numCameras = 0;
	for (MediaEntity *entity : cfeDevice->entities()) {
		if (entity->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		int ret = registerCamera(cfeDevice, ispDevice, entity);
		if (ret)
			LOG(PISP, Error) << "Failed to register camera "
					 << entity->name() << ": " << ret;
		else
			numCameras++;
	}

	return !!numCameras;
}

int PipelineHandlerPiSP::registerCamera(MediaDevice *cfe, MediaDevice *isp, MediaEntity *sensorEntity)
{
	std::unique_ptr<PiSPCameraData> data = std::make_unique<PiSPCameraData>(this);

	MediaEntity *cfeImage = cfe->getEntityByName("py-cfe-fe_image0");
	MediaEntity *cfeStats = cfe->getEntityByName("py-cfe-fe_stats");
	MediaEntity *cfeConfig = cfe->getEntityByName("py-cfe-fe_config");
	MediaEntity *ispInput = isp->getEntityByName("pispbe-input");
	MediaEntity *IpaPrepare = isp->getEntityByName("pispbe-config");
	MediaEntity *ispOutput0 = isp->getEntityByName("pispbe-output0");
	MediaEntity *ispOutput1 = isp->getEntityByName("pispbe-output1");
	//MediaEntity *ispHogOutput = isp->getEntityByName("pispbe-hog_output");
	//MediaEntity *ispTdnOutput = isp->getEntityByName("pispbe-tdn_output");
	//MediaEntity *ispStitchOutput = isp->getEntityByName("pispbe-stitch_output");

	if (!cfeImage || !cfeStats || !cfeConfig || !IpaPrepare || !ispOutput0 || !ispOutput1)
		return -ENOENT;

	/* Locate and open the cfe video streams. */
	data->cfe_[Cfe::Output0] = PiSP::Stream("CFE Image", cfeImage);
	data->cfe_[Cfe::Stats] = PiSP::Stream("CFE Stats", cfeStats);
	data->cfe_[Cfe::Config] = PiSP::Stream("CFE Config", cfeConfig, false, true);

	/* An embedded data node will not be present if the sensor does not support it. */
	MediaEntity *cfeEmbedded = cfe->getEntityByName("cfe-embedded");
	if (cfeEmbedded) {
		data->cfe_[Cfe::Embedded] = PiSP::Stream("CFE Embedded", cfeEmbedded);
		data->cfe_[Cfe::Embedded].dev()->bufferReady.connect(data.get(),
									   &PiSPCameraData::cfeBufferDequeue);
	}

	/* Tag the ISP input stream as an import stream. */
	data->isp_[Isp::Input] = PiSP::Stream("ISP Input", ispInput, true, false);
	data->isp_[Isp::Config] = PiSP::Stream("ISP Config", IpaPrepare, false, true);
	data->isp_[Isp::Output0] = PiSP::Stream("ISP Output0", ispOutput0);
	data->isp_[Isp::Output1] = PiSP::Stream("ISP Output1", ispOutput1);

	/* Wire up all the buffer connections. */
	data->cfe_[Cfe::Output0].dev()->frameStart.connect(data.get(), &PiSPCameraData::frameStarted);
	data->cfe_[Cfe::Output0].dev()->bufferReady.connect(data.get(), &PiSPCameraData::cfeBufferDequeue);
	data->cfe_[Cfe::Stats].dev()->bufferReady.connect(data.get(), &PiSPCameraData::cfeBufferDequeue);
	data->cfe_[Cfe::Config].dev()->bufferReady.connect(data.get(), &PiSPCameraData::cfeBufferDequeue);

	data->isp_[Isp::Input].dev()->bufferReady.connect(data.get(), &PiSPCameraData::ispInputDequeue);
	data->isp_[Isp::Config].dev()->bufferReady.connect(data.get(), &PiSPCameraData::ispOutputDequeue);
	data->isp_[Isp::Output0].dev()->bufferReady.connect(data.get(), &PiSPCameraData::ispOutputDequeue);
	data->isp_[Isp::Output1].dev()->bufferReady.connect(data.get(), &PiSPCameraData::ispOutputDequeue);

	data->sensor_ = std::make_unique<CameraSensor>(sensorEntity);
	if (!data->sensor_)
		return -EINVAL;

	if (data->sensor_->init())
		return -EINVAL;

	/*
	 * Enumerate all the Video Mux/Bridge devices across the sensor -> cfe
	 * link. There may be a cascade of devices in this link!
	 */
	MediaLink *link = sensorEntity->getPadByIndex(0)->links()[0];
	data->enumerateVideoDevices(link);

	data->sensorFormats_ = populateSensorFormats(data->sensor_);

	ipa::PiSP::SensorConfig sensorConfig;
	if (data->loadIPA(&sensorConfig)) {
		LOG(PISP, Error) << "Failed to load a suitable IPA library";
		return -EINVAL;
	}

	if (sensorConfig.sensorMetadata ^ !!cfeEmbedded) {
		LOG(PISP, Warning) << "Mismatch between CFE and CamHelper for embedded data usage!";
		sensorConfig.sensorMetadata = false;
		if (cfeEmbedded)
			data->cfe_[Cfe::Embedded].dev()->bufferReady.disconnect();
	}

	/*
	 * Open all CFE and ISP streams. The exception is the embedded data
	 * stream, which only gets opened below if the IPA reports that the sensor
	 * supports embedded data.
	 *
	 * The below grouping is just for convenience so that we can easily
	 * iterate over all streams in one go.
	 */
	data->streams_.push_back(&data->cfe_[Cfe::Output0]);
	data->streams_.push_back(&data->cfe_[Cfe::Config]);
	data->streams_.push_back(&data->cfe_[Cfe::Stats]);
	if (sensorConfig.sensorMetadata)
		data->streams_.push_back(&data->cfe_[Cfe::Embedded]);

	for (auto &stream : data->isp_)
		data->streams_.push_back(&stream);

	for (auto stream : data->streams_) {
		int ret = stream->dev()->open();
		if (ret)
			return ret;
	}

	if (!data->cfe_[Cfe::Output0].dev()->caps().hasMediaController()) {
		LOG(PISP, Error) << "CFE driver does not use the MediaController, please update your kernel!";
		return -EINVAL;
	}

	/*
	 * Setup our delayed control writer with the sensor default
	 * gain and exposure delays. Mark VBLANK for priority write.
	 */
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { sensorConfig.gainDelay, false } },
		{ V4L2_CID_EXPOSURE, { sensorConfig.exposureDelay, false } },
		{ V4L2_CID_VBLANK, { sensorConfig.vblankDelay, true } }
	};
	data->delayedCtrls_ = std::make_unique<DelayedControls>(data->sensor_->device(), params);
	data->sensorMetadata_ = sensorConfig.sensorMetadata;

	/* Register the controls that the Raspberry Pi IPA can handle. */
	data->controlInfo_ = RPi::Controls;
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
	 * We cache three things about the sensor in relation to transforms
	 * (meaning horizontal and vertical flips).
	 *
	 * Firstly, does it support them?
	 * Secondly, if you use them does it affect the Bayer ordering?
	 * Thirdly, what is the "native" Bayer order, when no transforms are
	 * applied?
	 *
	 * We note that the sensor's cached list of supported formats is
	 * already in the "native" order, with any flips having been undone.
	 */
	const V4L2Subdevice *sensor = data->sensor_->device();
	const struct v4l2_query_ext_ctrl *hflipCtrl = sensor->controlInfo(V4L2_CID_HFLIP);
	if (hflipCtrl) {
		/* We assume it will support vflips too... */
		data->supportsFlips_ = true;
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
		LOG(PISP, Error) << "No Bayer format found";
		return -EINVAL;
	}
	data->nativeBayerOrder_ = bayerFormat.order;

	/*
	 * List the available streams an application may request. At present, we
	 * do not advertise CFE Embedded and ISP Statistics streams, as there
	 * is no mechanism for the application to request non-image buffer formats.
	 */
	std::set<Stream *> streams;
	streams.insert(&data->cfe_[Cfe::Output0]);
	streams.insert(&data->isp_[Isp::Output0]);
	streams.insert(&data->isp_[Isp::Output1]);

	/* Create and register the camera. */
	const std::string &id = data->sensor_->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), id, streams);
	PipelineHandler::registerCamera(std::move(camera));

	LOG(PISP, Info) << "Registered camera " << id
		       << " to CFE device " << cfe->deviceNode()
		       << " and ISP device " << isp->deviceNode();
	return 0;
}

int PipelineHandlerPiSP::queueAllBuffers(Camera *camera)
{
	PiSPCameraData *data = cameraData(camera);
	int ret;

	for (auto const stream : data->streams_) {
		if (!stream->isExternal()) {
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
			 * PiSPStream object.
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

int PipelineHandlerPiSP::prepareBuffers(Camera *camera)
{
	PiSPCameraData *data = cameraData(camera);
	unsigned int numRawBuffers = 0;
	int ret;

	for (Stream *s : camera->streams()) {
		if (isRaw(s->configuration().pixelFormat)) {
			numRawBuffers = s->configuration().bufferCount;
			break;
		}
	}

	/* Decide how many internal buffers to allocate. */
	for (auto const stream : data->streams_) {
		unsigned int numBuffers;
		/*
		 * For CFE, allocate a minimum of 4 buffers as we want
		 * to avoid any frame drops.
		 */
		constexpr unsigned int minBuffers = 4;
		if (stream == &data->cfe_[Cfe::Output0]) {
			/*
			 * If an application has configured a RAW stream, allocate
			 * additional buffers to make up the minimum, but ensure
			 * we have at least 2 sets of internal buffers to use to
			 * minimise frame drops.
			 */
			numBuffers = std::max<int>(2, minBuffers - numRawBuffers);
		} else if (stream == &data->isp_[Isp::Input]) {
			/*
			 * ISP input buffers are imported from Unicam, so follow
			 * similar logic as above to count all the RAW buffers
			 * available.
			 */
			numBuffers = numRawBuffers + std::max<int>(2, minBuffers - numRawBuffers);
		} else if (stream == &data->cfe_[Cfe::Embedded]) {
			/*
			 * Embedded data buffers are (currently) for internal use,
			 * so allocate the minimum required to avoid frame drops.
			 */
			numBuffers = minBuffers;
		} else if (stream == &data->cfe_[Cfe::Config] || stream == &data->isp_[Isp::Config]) {
			numBuffers = 4;
		} else {
			/*
			 * Since the ISP runs synchronous with the IPA and requests,
			 * we only ever need one set of internal buffers. Any buffers
			 * the application wants to hold onto will already be exported
			 * through PipelineHandlerRPi::exportFrameBuffers().
			 */
			numBuffers = 1;
		}

		ret = stream->prepareBuffers(numBuffers);
		if (ret < 0)
			return ret;
	}

	for (auto const fb : data->isp_[Isp::Config].getBuffers()) {
		data->beConfigBuffers_.emplace(fb.first,
			MappedFrameBuffer(fb.second, MappedFrameBuffer::MapFlag::ReadWrite));
	}

	for (auto const fb : data->cfe_[Cfe::Config].getBuffers()) {
		data->feConfigBuffers_.emplace(fb.first,
			MappedFrameBuffer(fb.second, MappedFrameBuffer::MapFlag::ReadWrite));
	}

	/*
	 * Pass the stats and embedded data buffers to the IPA. No other
	 * buffers need to be passed.
	 */
	mapBuffers(camera, data->cfe_[Cfe::Stats].getBuffers(), ipa::PiSP::MaskStats);
	if (data->sensorMetadata_)
		mapBuffers(camera, data->cfe_[Cfe::Embedded].getBuffers(),
			   ipa::PiSP::MaskEmbeddedData);

	return 0;
}

void PipelineHandlerPiSP::mapBuffers(Camera *camera, const PiSP::BufferMap &buffers, unsigned int mask)
{
	PiSPCameraData *data = cameraData(camera);
	std::vector<IPABuffer> ipaBuffers;
	/*
	 * Link the FrameBuffers with the id (key value) in the map stored in
	 * the PiSP stream object - along with an identifier mask.
	 *
	 * This will allow us to identify buffers passed between the pipeline
	 * handler and the IPA.
	 */
	for (auto const &it : buffers) {
		ipaBuffers.push_back(IPABuffer(mask | it.first,
					       it.second->planes()));
		data->ipaBuffers_.insert(mask | it.first);
	}

	data->ipa_->mapBuffers(ipaBuffers);
}

void PipelineHandlerPiSP::freeBuffers(Camera *camera)
{
	PiSPCameraData *data = cameraData(camera);

	/* Copy the buffer ids from the unordered_set to a vector to pass to the IPA. */
	std::vector<unsigned int> ipaBuffers(data->ipaBuffers_.begin(), data->ipaBuffers_.end());
	data->ipa_->unmapBuffers(ipaBuffers);
	data->ipaBuffers_.clear();
	data->beConfigBuffers_.clear();
	data->feConfigBuffers_.clear();

	for (auto const stream : data->streams_)
		stream->releaseBuffers();
}

void PiSPCameraData::frameStarted(uint32_t sequence)
{
	LOG(PISP, Debug) << "frame start " << sequence;

	/* Write any controls for the next frame as soon as we can. */
	delayedCtrls_->applyControls(sequence);
}

int PiSPCameraData::loadIPA(ipa::PiSP::SensorConfig *sensorConfig)
{
	ipa_ = IPAManager::createIPA<ipa::PiSP::IPAProxyPiSP>(pipe(), 1, 1);

	if (!ipa_)
		return -ENOENT;

	ipa_->setDelayedControls.connect(this, &PiSPCameraData::setDelayedControls);

	/*
	 * The configuration (tuning file) is made from the sensor name unless
	 * the environment variable overrides it.
	 */
	std::string configurationFile;
	char const *configFromEnv = utils::secure_getenv("LIBCAMERA_RPI_TUNING_FILE");
	if (!configFromEnv || *configFromEnv == '\0')
		configurationFile = ipa_->configurationFile(sensor_->model() + ".json");
	else
		configurationFile = std::string(configFromEnv);

	IPASettings settings(configurationFile, sensor_->model());
	ipa::PiSP::InitConfig initConfig(fe_.getFD(), be_.getFD());
	return ipa_->init(settings, initConfig, sensorConfig);
	return 0;
}

int PiSPCameraData::configureIPA(const CameraConfiguration *config)
{
	(void)(config);
	std::map<unsigned int, IPAStream> streamConfig;

	/* Inform IPA of stream configuration and sensor controls. */
	unsigned int i = 0;
	for (auto const &stream : isp_) {
		if (stream.isExternal()) {
			streamConfig[i++] = IPAStream(
				stream.configuration().pixelFormat,
				stream.configuration().size);
		}
	}

	/* We store the IPACameraSensorInfo for digital zoom calculations. */
	int ret = sensor_->sensorInfo(&sensorInfo_);
	if (ret) {
		LOG(PISP, Error) << "Failed to retrieve camera sensor info";
		return ret;
	}

	/* Ready the IPA - it must know about the sensor resolution. */
	ControlList controls;
	ret = ipa_->configure(sensorInfo_, streamConfig, sensor_->controls(),
			      static_cast<unsigned int>(config->transform),
			      &controls);
	if (ret < 0) {
		LOG(PISP, Error) << "IPA configuration failed!";
		return -EPIPE;
	}

	if (!controls.empty())
		setSensorControls(controls);

	return 0;
}

int PiSPCameraData::configureCfe()
{
	V4L2DeviceFormat cfeFormat;
	cfe_[Cfe::Output0].dev()->getFormat(&cfeFormat);

	pisp_fe_global_config global;
	global.enables = PISP_FE_ENABLE_AWB_STATS | PISP_FE_ENABLE_AGC_STATS |
			 PISP_FE_ENABLE_CDAF_STATS | PISP_FE_ENABLE_OUTPUT0;

	BayerFormat bayer = BayerFormat::fromV4L2PixelFormat(cfeFormat.fourcc);
	switch (bayer.order) {
	case BayerFormat::Order::BGGR:
		global.bayer_order = PISP_BAYER_ORDER_BGGR;
		break;
	case BayerFormat::Order::GBRG:
		global.bayer_order = PISP_BAYER_ORDER_GBRG;
		break;
	case BayerFormat::Order::GRBG:
		global.bayer_order = PISP_BAYER_ORDER_GRBG;
		break;
	case BayerFormat::Order::RGGB:
		global.bayer_order = PISP_BAYER_ORDER_RGGB;
		break;
	default:
		ASSERT(0);
	}

	pisp_fe_input_config input;
	memset(&input, 0, sizeof(input));
	input.streaming = 1;
	input.format.width = cfeFormat.size.width;
	input.format.height = cfeFormat.size.height;
	input.format.format = PISP_IMAGE_FORMAT_BPS_16;

	pisp_image_format_config image;
	memset(&image, 0, sizeof(image));
	image.width = cfeFormat.size.width;
	image.height = cfeFormat.size.height;
	image.stride = cfeFormat.planes[0].bpl;
	image.format = PISP_IMAGE_FORMAT_BPS_16;

	fe_->SetGlobal(global);
	fe_->SetInput(input);
	fe_->SetOutputFormat(0, image);

	return 0;
}

int PiSPCameraData::configureBe()
{
	pisp_image_format_config inputFormat;
	V4L2DeviceFormat cfeFormat;

	isp_[Isp::Input].dev()->getFormat(&cfeFormat);
	inputFormat.width = cfeFormat.size.width;
	inputFormat.height = cfeFormat.size.height;
	inputFormat.stride = cfeFormat.planes[0].bpl;
	inputFormat.format = PISP_IMAGE_FORMAT_BPS_16 + PISP_IMAGE_FORMAT_UNCOMPRESSED;

	pisp_be_global_config global;
	global.bayer_enables = PISP_BE_BAYER_ENABLE_INPUT + PISP_BE_BAYER_ENABLE_BLC + PISP_BE_BAYER_ENABLE_WBG + PISP_BE_BAYER_ENABLE_DEMOSAIC;
	global.rgb_enables = PISP_BE_RGB_ENABLE_OUTPUT0 + PISP_BE_RGB_ENABLE_CSC0 + PISP_BE_RGB_ENABLE_OUTPUT1;

	BayerFormat bayer = BayerFormat::fromV4L2PixelFormat(cfeFormat.fourcc);
	switch (bayer.order) {
	case BayerFormat::Order::BGGR:
		global.bayer_order = PISP_BAYER_ORDER_BGGR;
		break;
	case BayerFormat::Order::GBRG:
		global.bayer_order = PISP_BAYER_ORDER_GBRG;
		break;
	case BayerFormat::Order::GRBG:
		global.bayer_order = PISP_BAYER_ORDER_GRBG;
		break;
	case BayerFormat::Order::RGGB:
		global.bayer_order = PISP_BAYER_ORDER_RGGB;
		break;
	default:
		ASSERT(0);
	}

	pisp_be_ccm_config csc;
	float coeffs[] = { 0.299, 0.587, 0.114, -0.169, -0.331, 0.500, 0.500, -0.419, -0.081 };
	float offsets[] = { 0, 32768, 32768 };

	for (int i = 0; i < 9; i++)
		csc.coeffs[i] = coeffs[i] * (1 << 10);
	for (int i = 0; i < 3; i++)
		csc.offsets[i] = offsets[i] * (1 << 10);

	V4L2DeviceFormat ispFormat0, ispFormat1;
	pisp_be_output_format_config outputFormat0, outputFormat1;

	memset(&outputFormat0, 0, sizeof(outputFormat0));
	memset(&outputFormat1, 0, sizeof(outputFormat1));

	isp_[Isp::Output0].dev()->getFormat(&ispFormat0);
	outputFormat0.image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL + PISP_IMAGE_FORMAT_BPS_8 +
				     PISP_IMAGE_FORMAT_SAMPLING_420 + PISP_IMAGE_FORMAT_PLANARITY_PLANAR;
	outputFormat0.image.width = ispFormat0.size.width;
	outputFormat0.image.height = ispFormat0.size.height;
	outputFormat0.image.stride = ispFormat0.planes[0].bpl;
	outputFormat0.image.stride2 = ispFormat0.planes[0].bpl / 2;

	isp_[Isp::Output1].dev()->getFormat(&ispFormat1);
	outputFormat1.image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL + PISP_IMAGE_FORMAT_BPS_8 +
				     PISP_IMAGE_FORMAT_SAMPLING_420 + PISP_IMAGE_FORMAT_PLANARITY_PLANAR;
	outputFormat1.image.width = ispFormat1.size.width;
	outputFormat1.image.height = ispFormat1.size.height;
	outputFormat1.image.stride = ispFormat1.planes[0].bpl;
	outputFormat1.image.stride2 = ispFormat1.planes[0].bpl / 2;

	if (ispFormat1.size != ispFormat0.size)
		global.rgb_enables |= PISP_BE_RGB_ENABLE_RESAMPLE1;

	be_->SetInputFormat(inputFormat);
	be_->SetGlobal(global);
	be_->SetBlc({4096, 4096, 4096, 4096, 0, /* pad */ 0});
	be_->SetWbg({(uint16_t)(2 * 1024), (uint16_t)(1.0 * 1024), (uint16_t)(1.5 * 1024), /* pad */ 0});
	be_->SetDemosaic({8, 2, /* pad */ 0});
	be_->SetCsc(0, csc);
	be_->SetCsc(1, csc);
	be_->SetOutputFormat(0, outputFormat0);
	be_->SetOutputFormat(1, outputFormat1);

	return 0;
}

/*
 * enumerateVideoDevices() iterates over the Media Controller topology, starting
 * at the sensor and finishing at Unicam. For each sensor, RPiCameraData stores
 * a unique list of any intermediate video mux or bridge devices connected in a
 * cascade, together with the entity to entity link.
 *
 * Entity pad configuration and link enabling happens at the end of configure().
 * We first disable all pad links on each entity device in the chain, and then
 * selectively enabling the specific links to link sensor to Unicam across all
 * intermediate muxes and bridges.
 *
 * In the cascaded topology below, if Sensor1 is used, the Mux2 -> Mux1 link
 * will be disabled, and Sensor1 -> Mux1 -> Unicam links enabled. Alternatively,
 * if Sensor3 is used, the Sensor2 -> Mux2 and Sensor1 -> Mux1 links are disabled,
 * and Sensor3 -> Mux2 -> Mux1 -> Unicam links are enabled. All other links will
 * remain unchanged.
 *
 *  +----------+
 *  |  Unicam  |
 *  +-----^----+
 *        |
 *    +---+---+
 *    |  Mux1 <-------+
 *    +--^----+       |
 *       |            |
 * +-----+---+    +---+---+
 * | Sensor1 |    |  Mux2 |<--+
 * +---------+    +-^-----+   |
 *                  |         |
 *          +-------+-+   +---+-----+
 *          | Sensor2 |   | Sensor3 |
 *          +---------+   +---------+
 */
void PiSPCameraData::enumerateVideoDevices(MediaLink *link)
{
	const MediaPad *sinkPad = link->sink();
	const MediaEntity *entity = sinkPad->entity();
	bool unicamFound = false;

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

	LOG(PISP, Debug) << "Found video mux device " << entity->name()
			 << " linked to sink pad " << sinkPad->index();

	bridgeDevices_.emplace_back(std::make_unique<V4L2Subdevice>(entity), link);
	bridgeDevices_.back().first->open();

	/*
	 * Iterate through all the sink pad links down the cascade to find any
	 * other Video Mux and Bridge devices.
	 */
	for (MediaLink *l : sourcePad->links()) {
		enumerateVideoDevices(l);
		/* Once we reach the Unicam entity, we are done. */
		if (l->sink()->entity()->name() == "unicam-image") {
			unicamFound = true;
			break;
		}
	}

	/* This identifies the end of our entity enumeration recursion. */
	if (link->source()->entity()->function() == MEDIA_ENT_F_CAM_SENSOR) {
		/*
		* If Unicam is not at the end of this cascade, we cannot configure
		* this topology automatically, so remove all entity references.
		*/
		if (!unicamFound) {
			LOG(PISP, Warning) << "Cannot automatically configure this MC topology!";
			bridgeDevices_.clear();
		}
	}
}

void PiSPCameraData::statsMetadataComplete(uint32_t bufferId, const ControlList &controls)
{
	if (state_ == State::Stopped)
		return;

	FrameBuffer *buffer = cfe_[Cfe::Stats].getBuffers().at(bufferId);

	handleStreamBuffer(buffer, &cfe_[Cfe::Stats]);

	/* Add to the Request metadata buffer what the IPA has provided. */
	Request *request = requestQueue_.front();
	request->metadata().merge(controls);

	/*
	 * Inform the sensor of the latest colour gains if it has the
	 * V4L2_CID_NOTIFY_GAINS control (which means notifyGainsUnity_ is set).
	 */
	if (notifyGainsUnity_ && controls.contains(libcamera::controls::ColourGains)) {
		libcamera::Span<const float> colourGains = controls.get(libcamera::controls::ColourGains);
		/* The control wants linear gains in the order B, Gb, Gr, R. */
		ControlList ctrls(sensor_->controls());
		std::array<int32_t, 4> gains{
			static_cast<int32_t>(colourGains[1] * *notifyGainsUnity_),
			*notifyGainsUnity_,
			*notifyGainsUnity_,
			static_cast<int32_t>(colourGains[0] * *notifyGainsUnity_)
		};
		ctrls.set(V4L2_CID_NOTIFY_GAINS, Span<const int32_t>{ gains });

		sensor_->setControls(&ctrls);
	}

	state_ = State::IpaComplete;
	handleState();
}

void PiSPCameraData::prepareCfe()
{
	FrameBuffer *configBuffer = cfe_[Cfe::Config].getBuffer();
	auto it = feConfigBuffers_.find(cfe_[Cfe::Config].getBufferId(configBuffer));

	ASSERT(it != feConfigBuffers_.end());

	Span<uint8_t> config = it->second.planes()[0];
	fe_->Prepare(reinterpret_cast<pisp_fe_config *>(config.data()));
	cfe_[Cfe::Config].queueBuffer(configBuffer);
}

void PiSPCameraData::runBackend(uint32_t bufferId)
{
	if (state_ == State::Stopped)
		return;

	ispOutputCount_ = 0;

	FrameBuffer *buffer = cfe_[Cfe::Output0].getBuffers().at(bufferId);

	LOG(PISP, Debug) << "Input re-queue to ISP, buffer id " << bufferId
			<< ", timestamp: " << buffer->metadata().timestamp;

	isp_[Isp::Input].queueBuffer(buffer);

	FrameBuffer *configBuffer = isp_[Isp::Config].getBuffer();
	auto it = beConfigBuffers_.find(isp_[Isp::Config].getBufferId(configBuffer));

	ASSERT(it != beConfigBuffers_.end());

	Span<uint8_t> config = it->second.planes()[0];
	be_->Prepare(reinterpret_cast<pisp_be_tiles_config *>(config.data()));
	isp_[Isp::Config].queueBuffer(configBuffer);

	handleState();
}

void PiSPCameraData::embeddedComplete(uint32_t bufferId)
{
	if (state_ == State::Stopped)
		return;

	FrameBuffer *buffer = cfe_[Cfe::Embedded].getBuffers().at(bufferId);
	handleStreamBuffer(buffer, &cfe_[Cfe::Embedded]);
	handleState();
}

void PiSPCameraData::setIspControls(const ControlList &controls)
{
	ControlList ctrls = controls;

	handleState();
}

void PiSPCameraData::setDelayedControls(const ControlList &controls)
{
	if (!delayedCtrls_->push(controls))
		LOG(PISP, Error) << "V4L2 DelayedControl set failed";
	handleState();
}

void PiSPCameraData::setSensorControls(ControlList &controls)
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

void PiSPCameraData::cfeBufferDequeue(FrameBuffer *buffer)
{
	PiSP::Stream *stream = nullptr;
	int index;

	if (state_ == State::Stopped)
		return;

	for (PiSP::Stream &s : cfe_) {
		index = s.getBufferId(buffer);
		if (index != -1) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our streams. */
	ASSERT(stream);

	LOG(PISP, Debug) << "Stream " << stream->name() << " buffer dequeue"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	if (stream == &cfe_[Cfe::Output0]) {
		/*
		 * Lookup the sensor controls used for this frame sequence from
		 * DelayedControl and queue them along with the frame buffer.
		 */
		ControlList ctrl = delayedCtrls_->get(buffer->metadata().sequence);
		/*
		 * Add the frame timestamp to the ControlList for the IPA to use
		 * as it does not receive the FrameBuffer object.
		 */
		ctrl.set(controls::SensorTimestamp, buffer->metadata().timestamp);
		bayerQueue_.push({ buffer, std::move(ctrl) });
	} else if (stream == &cfe_[Cfe::Stats]) {

		//ipa_->signalStatReady(ipa::PiSP::MaskStats | static_cast<unsigned int>(index));
		handleStreamBuffer(buffer, &cfe_[Cfe::Stats]);
	} else if (stream == &cfe_[Cfe::Config]) {
		handleStreamBuffer(buffer, &cfe_[Cfe::Config]);
		prepareCfe();
	}
	else {
		embeddedQueue_.push(buffer);
	}

	handleState();
}

void PiSPCameraData::ispInputDequeue(FrameBuffer *buffer)
{
	if (state_ == State::Stopped)
		return;

	LOG(PISP, Debug) << "Stream ISP Input buffer complete"
			<< ", buffer id " << cfe_[Cfe::Output0].getBufferId(buffer)
			<< ", timestamp: " << buffer->metadata().timestamp;

	/* The ISP input buffer gets re-queued into CFE. */
	handleStreamBuffer(buffer, &cfe_[Cfe::Output0]);
	handleState();
}

void PiSPCameraData::ispOutputDequeue(FrameBuffer *buffer)
{
	PiSP::Stream *stream = nullptr;
	int index;

	if (state_ == State::Stopped)
		return;

	for (PiSP::Stream &s : isp_) {
		index = s.getBufferId(buffer);
		if (index != -1) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our ISP output streams. */
	ASSERT(stream);

	LOG(PISP, Debug) << "Stream " << stream->name() << " buffer complete"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	handleStreamBuffer(buffer, stream);

	/*
	 * Increment the number of ISP outputs generated.
	 * This is needed to track dropped frames.
	 */
	ispOutputCount_++;

	if (ispOutputCount_ > 2)
		state_ = State::IpaComplete;

	handleState();
}

void PiSPCameraData::clearIncompleteRequests()
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
				buffer->cancel();
				pipe()->completeBuffer(request, buffer);
			}
		}

		pipe()->completeRequest(request);
		requestQueue_.pop();
	}
}

void PiSPCameraData::handleStreamBuffer(FrameBuffer *buffer, PiSP::Stream *stream)
{
	Request *request = requestQueue_.empty() ? nullptr : requestQueue_.front();
	/*
	 * It is possible to be here without a pending request, so check
	 * that we actually have one to action, otherwise we just return
	 * buffer back to the stream.
	 */
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
		 * This buffer was not part of the Request, or there is no
		 * pending request, so we can recycle it.
		 */
		stream->returnBuffer(buffer);
	}
}

void PiSPCameraData::handleExternalBuffer(FrameBuffer *buffer, PiSP::Stream *stream)
{
	unsigned int id = stream->getBufferId(buffer);

	if (!(id & ipa::PiSP::MaskExternalBuffer))
		return;

	/* Stop the Stream object from tracking the buffer. */
	stream->removeExternalBuffer(buffer);
}

void PiSPCameraData::handleState()
{
	switch (state_) {
	case State::Stopped:
	case State::Busy:
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

void PiSPCameraData::checkRequestCompleted()
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
	    ((ispOutputCount_ == 3 && dropFrameCount_) || requestCompleted)) {
		state_ = State::Idle;
		if (dropFrameCount_) {
			dropFrameCount_--;
			LOG(PISP, Info) << "Dropping frame at the request of the IPA ("
				       << dropFrameCount_ << " left)";
		}
	}
}

void PiSPCameraData::applyScalerCrop(const ControlList &controls)
{
	if (controls.contains(controls::ScalerCrop)) {
		Rectangle nativeCrop = controls.get<Rectangle>(controls::ScalerCrop);

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
			//isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &ispCrop);
			ispCrop_ = ispCrop;

			/*
			 * Also update the ScalerCrop in the metadata with what we actually
			 * used. But we must first rescale that from ISP (camera mode) pixels
			 * back into sensor native pixels.
			 */
			scalerCrop_ = ispCrop_.scaledBy(sensorInfo_.analogCrop.size(),
							sensorInfo_.outputSize);
			scalerCrop_.translateBy(sensorInfo_.analogCrop.topLeft());
		}
	}
}

void PiSPCameraData::fillRequestMetadata(const ControlList &bufferControls,
					Request *request)
{
	request->metadata().set(controls::SensorTimestamp,
				bufferControls.get(controls::SensorTimestamp));

	request->metadata().set(controls::ScalerCrop, scalerCrop_);
}

void PiSPCameraData::tryRunPipeline()
{
	FrameBuffer *embeddedBuffer;
	BayerFrame bayerFrame;

	/* If any of our request or buffer queues are empty, we cannot proceed. */
	if (state_ != State::Idle || requestQueue_.empty() ||
	    bayerQueue_.empty() || (embeddedQueue_.empty() && sensorMetadata_))
		return;

	if (!findMatchingBuffers(bayerFrame, embeddedBuffer))
		return;

	/* Take the first request from the queue and action the IPA. */
	Request *request = requestQueue_.front();

	/* See if a new ScalerCrop value needs to be applied. */
	applyScalerCrop(request->controls());

	/*
	 * Clear the request metadata and fill it with some initial non-IPA
	 * related controls. We clear it first because the request metadata
	 * may have been populated if we have dropped the previous frame.
	 */
	request->metadata().clear();
	fillRequestMetadata(bayerFrame.controls, request);

	/*
	 * Process all the user controls by the IPA. Once this is complete, we
	 * queue the ISP output buffer listed in the request to start the HW
	 * pipeline.
	 */
	//ipa_->signalQueueRequest(request->controls());

	/* Set our state to say the pipeline is active. */
	state_ = State::Busy;

	unsigned int bayerId = cfe_[Cfe::Output0].getBufferId(bayerFrame.buffer);

	LOG(PISP, Debug) << "Signalling signalIspPrepare:"
			<< " Bayer buffer id: " << bayerId;

	ipa::PiSP::PrepareConfig prepare;
	prepare.buffers.bayerBufferId = ipa::PiSP::MaskBayerData | bayerId;
	prepare.sensorControls = std::move(bayerFrame.controls);
	prepare.requestControls = request->controls();

	if (embeddedBuffer) {
		unsigned int embeddedId = cfe_[Cfe::Embedded].getBufferId(embeddedBuffer);

		prepare.buffers.embeddedBufferId = ipa::PiSP::MaskEmbeddedData | embeddedId;
		prepare.embeddedBufferPresent = true;

		LOG(PISP, Debug) << "Signalling signalIspPrepare:"
				<< " Bayer buffer id: " << embeddedId;
	}

	//ipa_->signalIspPrepare(ispPrepare);
	runBackend(bayerId);
}

bool PiSPCameraData::findMatchingBuffers(BayerFrame &bayerFrame, FrameBuffer *&embeddedBuffer)
{
	if (bayerQueue_.empty())
		return false;

	/* Start with the front of the bayer queue. */
	bayerFrame = std::move(bayerQueue_.front());
	bayerQueue_.pop();

	/*
	 * Find the embedded data buffer with a matching timestamp to pass to
	 * the IPA. Any embedded buffers with a timestamp lower than the
	 * current bayer buffer will be removed and re-queued to the driver.
	 */
	uint64_t ts = bayerFrame.buffer->metadata().timestamp;
	embeddedBuffer = nullptr;
	while (!embeddedQueue_.empty()) {
		FrameBuffer *b = embeddedQueue_.front();
		if (b->metadata().timestamp < ts) {
			embeddedQueue_.pop();
			cfe_[Cfe::Embedded].returnBuffer(b);
			LOG(PISP, Debug) << "Dropping unmatched input frame in stream "
					 << cfe_[Cfe::Embedded].name();
		} else if (b->metadata().timestamp == ts) {
			/* Found a match! */
			embeddedBuffer = b;
			embeddedQueue_.pop();
			break;
		} else {
			break; /* Only higher timestamps from here. */
		}
	}

	if (!embeddedBuffer && sensorMetadata_) {
		/* Log if there is no matching embedded data buffer found. */
		LOG(PISP, Debug) << "Returning bayer frame without a matching embedded buffer.";
	}

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerPiSP)

} /* namespace libcamera */
