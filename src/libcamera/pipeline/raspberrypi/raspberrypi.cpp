/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * raspberrypi.cpp - Pipeline handler for Raspberry Pi devices
 */
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_set>
#include <utility>

#include <libcamera/base/shared_fd.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/ipa/raspberrypi_ipa_interface.h>
#include <libcamera/ipa/raspberrypi_ipa_proxy.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>

#include <linux/bcm2835-isp.h>
#include <linux/media-bus-format.h>
#include <linux/videodev2.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "delayed_controls.h"
#include "dma_heaps.h"
#include "rpi_stream.h"

using namespace std::chrono_literals;

namespace libcamera {

LOG_DEFINE_CATEGORY(RPI)

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

bool isMonoSensor(std::unique_ptr<CameraSensor> &sensor)
{
	unsigned int mbusCode = sensor->mbusCodes()[0];
	const BayerFormat &bayer = BayerFormat::fromMbusCode(mbusCode);

	return bayer.order == BayerFormat::Order::MONO;
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

V4L2DeviceFormat toV4L2DeviceFormat(const V4L2VideoDevice *dev,
				    const V4L2SubdeviceFormat &format,
				    BayerFormat::Packing packingReq)
{
	const PixelFormat pix = mbusCodeToPixelFormat(format.mbus_code, packingReq);
	V4L2DeviceFormat deviceFormat;

	deviceFormat.fourcc = dev->toV4L2PixelFormat(pix);
	deviceFormat.size = format.size;
	deviceFormat.colorSpace = format.colorSpace;
	return deviceFormat;
}

bool isRaw(const PixelFormat &pixFmt)
{
	/* This test works for both Bayer and raw mono formats. */
	return BayerFormat::fromPixelFormat(pixFmt).isValid();
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

enum class Unicam : unsigned int { Image, Embedded };
enum class Isp : unsigned int { Input, Output0, Output1, Stats };

} /* namespace */

class RPiCameraData : public Camera::Private
{
public:
	RPiCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe), state_(State::Stopped),
		  supportsFlips_(false), flipsAlterBayerOrder_(false),
		  dropFrameCount_(0), buffersAllocated_(false), ispOutputCount_(0)
	{
	}

	~RPiCameraData()
	{
		freeBuffers();
	}

	void freeBuffers();
	void frameStarted(uint32_t sequence);

	int loadIPA(ipa::RPi::IPAInitResult *result);
	int configureIPA(const CameraConfiguration *config, ipa::RPi::IPAConfigResult *result);

	void enumerateVideoDevices(MediaLink *link);

	void statsMetadataComplete(uint32_t bufferId, const ControlList &controls);
	void runIsp(uint32_t bufferId);
	void embeddedComplete(uint32_t bufferId);
	void setIspControls(const ControlList &controls);
	void setDelayedControls(const ControlList &controls, uint32_t delayContext);
	void setSensorControls(ControlList &controls);
	void unicamTimeout();

	/* bufferComplete signal handlers. */
	void unicamBufferDequeue(FrameBuffer *buffer);
	void ispInputDequeue(FrameBuffer *buffer);
	void ispOutputDequeue(FrameBuffer *buffer);

	void clearIncompleteRequests();
	void handleStreamBuffer(FrameBuffer *buffer, RPi::Stream *stream);
	void handleExternalBuffer(FrameBuffer *buffer, RPi::Stream *stream);
	void handleState();
	Rectangle scaleIspCrop(const Rectangle &ispCrop) const;
	void applyScalerCrop(const ControlList &controls);

	std::unique_ptr<ipa::RPi::IPAProxyRPi> ipa_;

	std::unique_ptr<CameraSensor> sensor_;
	SensorFormats sensorFormats_;
	/* Array of Unicam and ISP device streams and associated buffers/streams. */
	RPi::Device<Unicam, 2> unicam_;
	RPi::Device<Isp, 4> isp_;
	/* The vector below is just for convenience when iterating over all streams. */
	std::vector<RPi::Stream *> streams_;
	/* Stores the ids of the buffers mapped in the IPA. */
	std::unordered_set<unsigned int> ipaBuffers_;
	/*
	 * Stores a cascade of Video Mux or Bridge devices between the sensor and
	 * Unicam together with media link across the entities.
	 */
	std::vector<std::pair<std::unique_ptr<V4L2Subdevice>, MediaLink *>> bridgeDevices_;

	/* DMAHEAP allocation helper. */
	RPi::DmaHeap dmaHeap_;
	SharedFD lsTable_;

	std::unique_ptr<RPi::DelayedControls> delayedCtrls_;
	bool sensorMetadata_;

	/*
	 * All the functions in this class are called from a single calling
	 * thread. So, we do not need to have any mutex to protect access to any
	 * of the variables below.
	 */
	enum class State { Stopped, Idle, Busy, IpaComplete, Error };
	State state_;

	bool isRunning()
	{
		return state_ != State::Stopped && state_ != State::Error;
	}

	struct BayerFrame {
		FrameBuffer *buffer;
		ControlList controls;
		unsigned int delayContext;
	};

	std::queue<BayerFrame> bayerQueue_;
	std::queue<FrameBuffer *> embeddedQueue_;
	std::deque<Request *> requestQueue_;

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

	/* Have internal buffers been allocated? */
	bool buffersAllocated_;

private:
	void checkRequestCompleted();
	void fillRequestMetadata(const ControlList &bufferControls,
				 Request *request);
	void tryRunPipeline();
	bool findMatchingBuffers(BayerFrame &bayerFrame, FrameBuffer *&embeddedBuffer);

	unsigned int ispOutputCount_;
};

class RPiCameraConfiguration : public CameraConfiguration
{
public:
	RPiCameraConfiguration(const RPiCameraData *data);

	Status validate() override;

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;

private:
	const RPiCameraData *data_;
};

class PipelineHandlerRPi : public PipelineHandler
{
public:
	PipelineHandlerRPi(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
		const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	void releaseDevice(Camera *camera) override;

private:
	RPiCameraData *cameraData(Camera *camera)
	{
		return static_cast<RPiCameraData *>(camera->_d());
	}

	int registerCamera(MediaDevice *unicam, MediaDevice *isp, MediaEntity *sensorEntity);
	int queueAllBuffers(Camera *camera);
	int prepareBuffers(Camera *camera);
	void mapBuffers(Camera *camera, const RPi::BufferMap &buffers, unsigned int mask);
};

RPiCameraConfiguration::RPiCameraConfiguration(const RPiCameraData *data)
	: CameraConfiguration(), data_(data)
{
}

CameraConfiguration::Status RPiCameraConfiguration::validate()
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
	int32_t rotation = data_->sensor_->properties().get(properties::Rotation).value_or(0);
	bool success;
	Transform rotationTransform = transformFromRotation(rotation, &success);
	if (!success)
		LOG(RPI, Warning) << "Invalid rotation of " << rotation
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
			V4L2VideoDevice *unicam = data_->unicam_[Unicam::Image].dev();
			const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
			unsigned int bitDepth = info.isValid() ? info.bitsPerPixel : defaultRawBitDepth;
			V4L2SubdeviceFormat sensorFormat = findBestFormat(data_->sensorFormats_, cfg.size, bitDepth);
			BayerFormat::Packing packing = BayerFormat::Packing::CSI2;
			if (info.isValid() && !info.packed)
				packing = BayerFormat::Packing::None;
			V4L2DeviceFormat unicamFormat = toV4L2DeviceFormat(unicam, sensorFormat, packing);
			int ret = unicam->tryFormat(&unicamFormat);
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
			V4L2PixelFormat fourcc = unicamFormat.fourcc;
			if (data_->flipsAlterBayerOrder_) {
				BayerFormat bayer = BayerFormat::fromV4L2PixelFormat(fourcc);
				bayer.order = data_->nativeBayerOrder_;
				bayer = bayer.transform(combined);
				fourcc = bayer.toV4L2PixelFormat();
			}

			PixelFormat unicamPixFormat = fourcc.toPixelFormat();
			if (cfg.size != unicamFormat.size ||
			    cfg.pixelFormat != unicamPixFormat) {
				cfg.size = unicamFormat.size;
				cfg.pixelFormat = unicamPixFormat;
				status = Adjusted;
			}

			cfg.stride = unicamFormat.planes[0].bpl;
			cfg.frameSize = unicamFormat.planes[0].size;

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
			LOG(RPI, Error) << "Invalid number of streams requested";
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

		if (fmts.find(dev->toV4L2PixelFormat(cfgPixFmt)) == fmts.end()) {
			/* If we cannot find a native format, use a default one. */
			cfgPixFmt = formats::NV12;
			status = Adjusted;
		}

		V4L2DeviceFormat format;
		format.fourcc = dev->toV4L2PixelFormat(cfg.pixelFormat);
		format.size = cfg.size;
		format.colorSpace = cfg.colorSpace;

		LOG(RPI, Debug)
			<< "Try color space " << ColorSpace::toString(cfg.colorSpace);

		int ret = dev->tryFormat(&format);
		if (ret)
			return Invalid;

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

PipelineHandlerRPi::PipelineHandlerRPi(CameraManager *manager)
	: PipelineHandler(manager)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerRPi::generateConfiguration(Camera *camera, const StreamRoles &roles)
{
	RPiCameraData *data = cameraData(camera);
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
			 * Still image codecs usually expect the sYCC color space.
			 * Even RGB codecs will be fine as the RGB we get with the
			 * sYCC color space is the same as sRGB.
			 */
			colorSpace = ColorSpace::Sycc;
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
			colorSpace = ColorSpace::Sycc;
			size = { 800, 600 };
			bufferCount = 4;
			outCount++;
			break;

		default:
			LOG(RPI, Error) << "Requested stream role not supported: "
					<< role;
			return nullptr;
		}

		if (rawCount > 1 || outCount > 2) {
			LOG(RPI, Error) << "Invalid stream roles requested";
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

int PipelineHandlerRPi::configure(Camera *camera, CameraConfiguration *config)
{
	RPiCameraData *data = cameraData(camera);
	int ret;

	/* Start by freeing all buffers and reset the Unicam and ISP stream states. */
	data->freeBuffers();
	for (auto const stream : data->streams_)
		stream->setExternal(false);

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
		const RPiCameraConfiguration *rpiConfig =
			static_cast<const RPiCameraConfiguration *>(config);
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

	V4L2VideoDevice *unicam = data->unicam_[Unicam::Image].dev();
	V4L2DeviceFormat unicamFormat = toV4L2DeviceFormat(unicam, sensorFormat, packing);
	ret = unicam->setFormat(&unicamFormat);
	if (ret)
		return ret;

	LOG(RPI, Info) << "Sensor: " << camera->id()
		       << " - Selected sensor format: " << sensorFormat
		       << " - Selected unicam format: " << unicamFormat;

	ret = data->isp_[Isp::Input].dev()->setFormat(&unicamFormat);
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
			cfg.setStream(&data->unicam_[Unicam::Image]);
			data->unicam_[Unicam::Image].setExternal(true);
			continue;
		}

		/* The largest resolution gets routed to the ISP Output 0 node. */
		RPi::Stream *stream = i == maxIndex ? &data->isp_[Isp::Output0]
						    : &data->isp_[Isp::Output1];

		V4L2PixelFormat fourcc = stream->dev()->toV4L2PixelFormat(cfg.pixelFormat);
		format.size = cfg.size;
		format.fourcc = fourcc;
		format.colorSpace = cfg.colorSpace;

		LOG(RPI, Debug) << "Setting " << stream->name() << " to "
				<< format;

		ret = stream->dev()->setFormat(&format);
		if (ret)
			return -EINVAL;

		if (format.size != cfg.size || format.fourcc != fourcc) {
			LOG(RPI, Error)
				<< "Failed to set requested format on " << stream->name()
				<< ", returned " << format;
			return -EINVAL;
		}

		LOG(RPI, Debug)
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
		V4L2VideoDevice *dev = data->isp_[Isp::Output0].dev();

		maxSize = Size(320, 240);
		format = {};
		format.size = maxSize;
		format.fourcc = dev->toV4L2PixelFormat(formats::YUV420);
		/* No one asked for output, so the color space doesn't matter. */
		format.colorSpace = ColorSpace::Sycc;
		ret = dev->setFormat(&format);
		if (ret) {
			LOG(RPI, Error)
				<< "Failed to set default format on ISP Output0: "
				<< ret;
			return -EINVAL;
		}

		LOG(RPI, Debug) << "Defaulting ISP Output0 format to "
				<< format;
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
		V4L2VideoDevice *dev = data->isp_[Isp::Output1].dev();

		V4L2DeviceFormat output1Format;
		constexpr Size maxDimensions(1200, 1200);
		const Size limit = maxDimensions.boundedToAspectRatio(format.size);

		output1Format.size = (format.size / 2).boundedTo(limit).alignedDownTo(2, 2);
		output1Format.colorSpace = format.colorSpace;
		output1Format.fourcc = dev->toV4L2PixelFormat(formats::YUV420);

		LOG(RPI, Debug) << "Setting ISP Output1 (internal) to "
				<< output1Format;

		ret = dev->setFormat(&output1Format);
		if (ret) {
			LOG(RPI, Error) << "Failed to set format on ISP Output1: "
					<< ret;
			return -EINVAL;
		}
	}

	/* ISP statistics output format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_BCM2835_ISP_STATS);
	ret = data->isp_[Isp::Stats].dev()->setFormat(&format);
	if (ret) {
		LOG(RPI, Error) << "Failed to set format on ISP stats stream: "
				<< format;
		return ret;
	}

	/* Figure out the smallest selection the ISP will allow. */
	Rectangle testCrop(0, 0, 1, 1);
	data->isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &testCrop);
	data->ispMinCropSize_ = testCrop.size();

	/* Adjust aspect ratio by providing crops on the input image. */
	Size size = unicamFormat.size.boundedToAspectRatio(maxSize);
	Rectangle crop = size.centeredTo(Rectangle(unicamFormat.size).center());
	data->ispCrop_ = crop;

	data->isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &crop);

	ipa::RPi::IPAConfigResult result;
	ret = data->configureIPA(config, &result);
	if (ret)
		LOG(RPI, Error) << "Failed to configure the IPA: " << ret;

	/*
	 * Set the scaler crop to the value we are using (scaled to native sensor
	 * coordinates).
	 */
	data->scalerCrop_ = data->scaleIspCrop(data->ispCrop_);

	/*
	 * Configure the Unicam embedded data output format only if the sensor
	 * supports it.
	 */
	if (data->sensorMetadata_) {
		V4L2SubdeviceFormat embeddedFormat;

		data->sensor_->device()->getFormat(1, &embeddedFormat);
		format.fourcc = V4L2PixelFormat(V4L2_META_FMT_SENSOR_DATA);
		format.planes[0].size = embeddedFormat.size.width * embeddedFormat.size.height;

		LOG(RPI, Debug) << "Setting embedded data format.";
		ret = data->unicam_[Unicam::Embedded].dev()->setFormat(&format);
		if (ret) {
			LOG(RPI, Error) << "Failed to set format on Unicam embedded: "
					<< format;
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

	/* Store the mode sensitivity for the application. */
	data->properties_.set(properties::SensorSensitivity, result.modeSensitivity);

	/* Update the controls that the Raspberry Pi IPA can handle. */
	ControlInfoMap::Map ctrlMap;
	for (auto const &c : result.controlInfo)
		ctrlMap.emplace(c.first, c.second);

	/* Add the ScalerCrop control limits based on the current mode. */
	Rectangle ispMinCrop(data->ispMinCropSize_);
	ispMinCrop.scaleBy(data->sensorInfo_.analogCrop.size(), data->sensorInfo_.outputSize);
	ctrlMap[&controls::ScalerCrop] = ControlInfo(ispMinCrop, Rectangle(data->sensorInfo_.analogCrop.size()));

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
					<< " with format  " << format
					<< ": " << ret;
			return ret;
		}

		LOG(RPI, Debug) << "Configured media link on device " << device->entity()->name()
				<< " on pad " << sinkPad->index();
	}

	return ret;
}

int PipelineHandlerRPi::exportFrameBuffers([[maybe_unused]] Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	RPi::Stream *s = static_cast<RPi::Stream *>(stream);
	unsigned int count = stream->configuration().bufferCount;
	int ret = s->dev()->exportBuffers(count, buffers);

	s->setExportedBuffers(buffers);

	return ret;
}

int PipelineHandlerRPi::start(Camera *camera, const ControlList *controls)
{
	RPiCameraData *data = cameraData(camera);
	int ret;

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

	/* Check if a ScalerCrop control was specified. */
	if (controls)
		data->applyScalerCrop(*controls);

	/* Start the IPA. */
	ipa::RPi::StartConfig startConfig;
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
		LOG(RPI, Error) << "Failed to queue buffers";
		stop(camera);
		return ret;
	}

	/* Enable SOF event generation. */
	data->unicam_[Unicam::Image].dev()->setFrameStartEnabled(true);

	/*
	 * Reset the delayed controls with the gain and exposure values set by
	 * the IPA.
	 */
	data->delayedCtrls_->reset(0);

	data->state_ = RPiCameraData::State::Idle;

	/* Start all streams. */
	for (auto const stream : data->streams_) {
		ret = stream->dev()->streamOn();
		if (ret) {
			stop(camera);
			return ret;
		}
	}

	/*
	 * Set the dequeue timeout to the larger of 2x the maximum possible
	 * frame duration or 1 second.
	 */
	utils::Duration timeout =
		std::max<utils::Duration>(1s, 2 * startConfig.maxSensorFrameLengthMs * 1ms);
	data->unicam_[Unicam::Image].dev()->setDequeueTimeout(timeout);

	return 0;
}

void PipelineHandlerRPi::stopDevice(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);

	data->state_ = RPiCameraData::State::Stopped;

	/* Disable SOF event generation. */
	data->unicam_[Unicam::Image].dev()->setFrameStartEnabled(false);

	for (auto const stream : data->streams_)
		stream->dev()->streamOff();

	data->clearIncompleteRequests();
	data->bayerQueue_ = {};
	data->embeddedQueue_ = {};

	/* Stop the IPA. */
	data->ipa_->stop();
}

int PipelineHandlerRPi::queueRequestDevice(Camera *camera, Request *request)
{
	RPiCameraData *data = cameraData(camera);

	if (!data->isRunning())
		return -EINVAL;

	LOG(RPI, Debug) << "queueRequestDevice: New request.";

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
		 * queuing the request for buffers in the RPiStream object.
		 */
		int ret = stream->queueBuffer(buffer);
		if (ret)
			return ret;
	}

	/* Push the request to the back of the queue. */
	data->requestQueue_.push_back(request);
	data->handleState();

	return 0;
}

bool PipelineHandlerRPi::match(DeviceEnumerator *enumerator)
{
	DeviceMatch unicam("unicam");
	MediaDevice *unicamDevice = acquireMediaDevice(enumerator, unicam);

	if (!unicamDevice) {
		LOG(RPI, Debug) << "Unable to acquire a Unicam instance";
		return false;
	}

	DeviceMatch isp("bcm2835-isp");
	MediaDevice *ispDevice = acquireMediaDevice(enumerator, isp);

	if (!ispDevice) {
		LOG(RPI, Debug) << "Unable to acquire ISP instance";
		return false;
	}

	/*
	 * The loop below is used to register multiple cameras behind one or more
	 * video mux devices that are attached to a particular Unicam instance.
	 * Obviously these cameras cannot be used simultaneously.
	 */
	unsigned int numCameras = 0;
	for (MediaEntity *entity : unicamDevice->entities()) {
		if (entity->function() != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		int ret = registerCamera(unicamDevice, ispDevice, entity);
		if (ret)
			LOG(RPI, Error) << "Failed to register camera "
					<< entity->name() << ": " << ret;
		else
			numCameras++;
	}

	return !!numCameras;
}

void PipelineHandlerRPi::releaseDevice(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);
	data->freeBuffers();
}

int PipelineHandlerRPi::registerCamera(MediaDevice *unicam, MediaDevice *isp, MediaEntity *sensorEntity)
{
	std::unique_ptr<RPiCameraData> data = std::make_unique<RPiCameraData>(this);

	if (!data->dmaHeap_.isValid())
		return -ENOMEM;

	MediaEntity *unicamImage = unicam->getEntityByName("unicam-image");
	MediaEntity *ispOutput0 = isp->getEntityByName("bcm2835-isp0-output0");
	MediaEntity *ispCapture1 = isp->getEntityByName("bcm2835-isp0-capture1");
	MediaEntity *ispCapture2 = isp->getEntityByName("bcm2835-isp0-capture2");
	MediaEntity *ispCapture3 = isp->getEntityByName("bcm2835-isp0-capture3");

	if (!unicamImage || !ispOutput0 || !ispCapture1 || !ispCapture2 || !ispCapture3)
		return -ENOENT;

	/* Locate and open the unicam video streams. */
	data->unicam_[Unicam::Image] = RPi::Stream("Unicam Image", unicamImage);

	/* An embedded data node will not be present if the sensor does not support it. */
	MediaEntity *unicamEmbedded = unicam->getEntityByName("unicam-embedded");
	if (unicamEmbedded) {
		data->unicam_[Unicam::Embedded] = RPi::Stream("Unicam Embedded", unicamEmbedded);
		data->unicam_[Unicam::Embedded].dev()->bufferReady.connect(data.get(),
									   &RPiCameraData::unicamBufferDequeue);
	}

	/* Tag the ISP input stream as an import stream. */
	data->isp_[Isp::Input] = RPi::Stream("ISP Input", ispOutput0, true);
	data->isp_[Isp::Output0] = RPi::Stream("ISP Output0", ispCapture1);
	data->isp_[Isp::Output1] = RPi::Stream("ISP Output1", ispCapture2);
	data->isp_[Isp::Stats] = RPi::Stream("ISP Stats", ispCapture3);

	/* Wire up all the buffer connections. */
	data->unicam_[Unicam::Image].dev()->dequeueTimeout.connect(data.get(), &RPiCameraData::unicamTimeout);
	data->unicam_[Unicam::Image].dev()->frameStart.connect(data.get(), &RPiCameraData::frameStarted);
	data->unicam_[Unicam::Image].dev()->bufferReady.connect(data.get(), &RPiCameraData::unicamBufferDequeue);
	data->isp_[Isp::Input].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispInputDequeue);
	data->isp_[Isp::Output0].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispOutputDequeue);
	data->isp_[Isp::Output1].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispOutputDequeue);
	data->isp_[Isp::Stats].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispOutputDequeue);

	data->sensor_ = std::make_unique<CameraSensor>(sensorEntity);
	if (!data->sensor_)
		return -EINVAL;

	if (data->sensor_->init())
		return -EINVAL;

	/*
	 * Enumerate all the Video Mux/Bridge devices across the sensor -> unicam
	 * chain. There may be a cascade of devices in this chain!
	 */
	MediaLink *link = sensorEntity->getPadByIndex(0)->links()[0];
	data->enumerateVideoDevices(link);

	data->sensorFormats_ = populateSensorFormats(data->sensor_);

	ipa::RPi::IPAInitResult result;
	if (data->loadIPA(&result)) {
		LOG(RPI, Error) << "Failed to load a suitable IPA library";
		return -EINVAL;
	}

	if (result.sensorConfig.sensorMetadata ^ !!unicamEmbedded) {
		LOG(RPI, Warning) << "Mismatch between Unicam and CamHelper for embedded data usage!";
		result.sensorConfig.sensorMetadata = false;
		if (unicamEmbedded)
			data->unicam_[Unicam::Embedded].dev()->bufferReady.disconnect();
	}

	/*
	 * Open all Unicam and ISP streams. The exception is the embedded data
	 * stream, which only gets opened below if the IPA reports that the sensor
	 * supports embedded data.
	 *
	 * The below grouping is just for convenience so that we can easily
	 * iterate over all streams in one go.
	 */
	data->streams_.push_back(&data->unicam_[Unicam::Image]);
	if (result.sensorConfig.sensorMetadata)
		data->streams_.push_back(&data->unicam_[Unicam::Embedded]);

	for (auto &stream : data->isp_)
		data->streams_.push_back(&stream);

	for (auto stream : data->streams_) {
		int ret = stream->dev()->open();
		if (ret)
			return ret;
	}

	if (!data->unicam_[Unicam::Image].dev()->caps().hasMediaController()) {
		LOG(RPI, Error) << "Unicam driver does not use the MediaController, please update your kernel!";
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
		LOG(RPI, Error) << "No Bayer format found";
		return -EINVAL;
	}
	data->nativeBayerOrder_ = bayerFormat.order;

	/*
	 * List the available streams an application may request. At present, we
	 * do not advertise Unicam Embedded and ISP Statistics streams, as there
	 * is no mechanism for the application to request non-image buffer formats.
	 */
	std::set<Stream *> streams;
	streams.insert(&data->unicam_[Unicam::Image]);
	streams.insert(&data->isp_[Isp::Output0]);
	streams.insert(&data->isp_[Isp::Output1]);

	/* Create and register the camera. */
	const std::string &id = data->sensor_->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), id, streams);
	PipelineHandler::registerCamera(std::move(camera));

	LOG(RPI, Info) << "Registered camera " << id
		       << " to Unicam device " << unicam->deviceNode()
		       << " and ISP device " << isp->deviceNode();
	return 0;
}

int PipelineHandlerRPi::queueAllBuffers(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);
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

int PipelineHandlerRPi::prepareBuffers(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);
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
		 * For Unicam, allocate a minimum of 4 buffers as we want
		 * to avoid any frame drops.
		 */
		constexpr unsigned int minBuffers = 4;
		if (stream == &data->unicam_[Unicam::Image]) {
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

		} else if (stream == &data->unicam_[Unicam::Embedded]) {
			/*
			 * Embedded data buffers are (currently) for internal use,
			 * so allocate the minimum required to avoid frame drops.
			 */
			numBuffers = minBuffers;
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

	/*
	 * Pass the stats and embedded data buffers to the IPA. No other
	 * buffers need to be passed.
	 */
	mapBuffers(camera, data->isp_[Isp::Stats].getBuffers(), RPi::MaskStats);
	if (data->sensorMetadata_)
		mapBuffers(camera, data->unicam_[Unicam::Embedded].getBuffers(),
			   RPi::MaskEmbeddedData);

	return 0;
}

void PipelineHandlerRPi::mapBuffers(Camera *camera, const RPi::BufferMap &buffers, unsigned int mask)
{
	RPiCameraData *data = cameraData(camera);
	std::vector<IPABuffer> ipaBuffers;
	/*
	 * Link the FrameBuffers with the id (key value) in the map stored in
	 * the RPi stream object - along with an identifier mask.
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

void RPiCameraData::freeBuffers()
{
	if (ipa_) {
		/*
		 * Copy the buffer ids from the unordered_set to a vector to
		 * pass to the IPA.
		 */
		std::vector<unsigned int> ipaBuffers(ipaBuffers_.begin(),
						     ipaBuffers_.end());
		ipa_->unmapBuffers(ipaBuffers);
		ipaBuffers_.clear();
	}

	for (auto const stream : streams_)
		stream->releaseBuffers();

	buffersAllocated_ = false;
}

void RPiCameraData::frameStarted(uint32_t sequence)
{
	LOG(RPI, Debug) << "frame start " << sequence;

	/* Write any controls for the next frame as soon as we can. */
	delayedCtrls_->applyControls(sequence);
}

int RPiCameraData::loadIPA(ipa::RPi::IPAInitResult *result)
{
	ipa_ = IPAManager::createIPA<ipa::RPi::IPAProxyRPi>(pipe(), 1, 1);

	if (!ipa_)
		return -ENOENT;

	ipa_->statsMetadataComplete.connect(this, &RPiCameraData::statsMetadataComplete);
	ipa_->runIsp.connect(this, &RPiCameraData::runIsp);
	ipa_->embeddedComplete.connect(this, &RPiCameraData::embeddedComplete);
	ipa_->setIspControls.connect(this, &RPiCameraData::setIspControls);
	ipa_->setDelayedControls.connect(this, &RPiCameraData::setDelayedControls);

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

	return ipa_->init(settings, result);
}

int RPiCameraData::configureIPA(const CameraConfiguration *config, ipa::RPi::IPAConfigResult *result)
{
	std::map<unsigned int, IPAStream> streamConfig;
	std::map<unsigned int, ControlInfoMap> entityControls;
	ipa::RPi::IPAConfig ipaConfig;

	/* Inform IPA of stream configuration and sensor controls. */
	unsigned int i = 0;
	for (auto const &stream : isp_) {
		if (stream.isExternal()) {
			streamConfig[i++] = IPAStream(
				stream.configuration().pixelFormat,
				stream.configuration().size);
		}
	}

	entityControls.emplace(0, sensor_->controls());
	entityControls.emplace(1, isp_[Isp::Input].dev()->controls());

	/* Always send the user transform to the IPA. */
	ipaConfig.transform = static_cast<unsigned int>(config->transform);

	/* Allocate the lens shading table via dmaHeap and pass to the IPA. */
	if (!lsTable_.isValid()) {
		lsTable_ = SharedFD(dmaHeap_.alloc("ls_grid", ipa::RPi::MaxLsGridSize));
		if (!lsTable_.isValid())
			return -ENOMEM;

		/* Allow the IPA to mmap the LS table via the file descriptor. */
		/*
		 * \todo Investigate if mapping the lens shading table buffer
		 * could be handled with mapBuffers().
		 */
		ipaConfig.lsTableHandle = lsTable_;
	}

	/* We store the IPACameraSensorInfo for digital zoom calculations. */
	int ret = sensor_->sensorInfo(&sensorInfo_);
	if (ret) {
		LOG(RPI, Error) << "Failed to retrieve camera sensor info";
		return ret;
	}

	/* Ready the IPA - it must know about the sensor resolution. */
	ControlList controls;
	ret = ipa_->configure(sensorInfo_, streamConfig, entityControls, ipaConfig,
			      &controls, result);
	if (ret < 0) {
		LOG(RPI, Error) << "IPA configuration failed!";
		return -EPIPE;
	}

	if (!controls.empty())
		setSensorControls(controls);

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
void RPiCameraData::enumerateVideoDevices(MediaLink *link)
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

	LOG(RPI, Debug) << "Found video mux device " << entity->name()
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
			LOG(RPI, Warning) << "Cannot automatically configure this MC topology!";
			bridgeDevices_.clear();
		}
	}
}

void RPiCameraData::statsMetadataComplete(uint32_t bufferId, const ControlList &controls)
{
	if (!isRunning())
		return;

	FrameBuffer *buffer = isp_[Isp::Stats].getBuffers().at(bufferId & RPi::MaskID);

	handleStreamBuffer(buffer, &isp_[Isp::Stats]);

	/* Add to the Request metadata buffer what the IPA has provided. */
	Request *request = requestQueue_.front();
	request->metadata().merge(controls);

	/*
	 * Inform the sensor of the latest colour gains if it has the
	 * V4L2_CID_NOTIFY_GAINS control (which means notifyGainsUnity_ is set).
	 */
	const auto &colourGains = controls.get(libcamera::controls::ColourGains);
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

	state_ = State::IpaComplete;
	handleState();
}

void RPiCameraData::runIsp(uint32_t bufferId)
{
	if (!isRunning())
		return;

	FrameBuffer *buffer = unicam_[Unicam::Image].getBuffers().at(bufferId & RPi::MaskID);

	LOG(RPI, Debug) << "Input re-queue to ISP, buffer id " << (bufferId & RPi::MaskID)
			<< ", timestamp: " << buffer->metadata().timestamp;

	isp_[Isp::Input].queueBuffer(buffer);
	ispOutputCount_ = 0;
	handleState();
}

void RPiCameraData::embeddedComplete(uint32_t bufferId)
{
	if (!isRunning())
		return;

	FrameBuffer *buffer = unicam_[Unicam::Embedded].getBuffers().at(bufferId & RPi::MaskID);
	handleStreamBuffer(buffer, &unicam_[Unicam::Embedded]);
	handleState();
}

void RPiCameraData::setIspControls(const ControlList &controls)
{
	ControlList ctrls = controls;

	if (ctrls.contains(V4L2_CID_USER_BCM2835_ISP_LENS_SHADING)) {
		ControlValue &value =
			const_cast<ControlValue &>(ctrls.get(V4L2_CID_USER_BCM2835_ISP_LENS_SHADING));
		Span<uint8_t> s = value.data();
		bcm2835_isp_lens_shading *ls =
			reinterpret_cast<bcm2835_isp_lens_shading *>(s.data());
		ls->dmabuf = lsTable_.get();
	}

	isp_[Isp::Input].dev()->setControls(&ctrls);
	handleState();
}

void RPiCameraData::setDelayedControls(const ControlList &controls, uint32_t delayContext)
{
	if (!delayedCtrls_->push(controls, delayContext))
		LOG(RPI, Error) << "V4L2 DelayedControl set failed";
	handleState();
}

void RPiCameraData::setSensorControls(ControlList &controls)
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

void RPiCameraData::unicamTimeout()
{
	LOG(RPI, Error) << "Unicam has timed out!";
	LOG(RPI, Error) << "Please check that your camera sensor connector is attached securely.";
	LOG(RPI, Error) << "Alternatively, try another cable and/or sensor.";

	state_ = RPiCameraData::State::Error;
	/*
	 * To allow the application to attempt a recovery from this timeout,
	 * stop all devices streaming, and return any outstanding requests as
	 * incomplete and cancelled.
	 */
	for (auto const stream : streams_)
		stream->dev()->streamOff();

	clearIncompleteRequests();
}

void RPiCameraData::unicamBufferDequeue(FrameBuffer *buffer)
{
	RPi::Stream *stream = nullptr;
	int index;

	if (!isRunning())
		return;

	for (RPi::Stream &s : unicam_) {
		index = s.getBufferId(buffer);
		if (index != -1) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer dequeue"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	if (stream == &unicam_[Unicam::Image]) {
		/*
		 * Lookup the sensor controls used for this frame sequence from
		 * DelayedControl and queue them along with the frame buffer.
		 */
		auto [ctrl, delayContext] = delayedCtrls_->get(buffer->metadata().sequence);
		/*
		 * Add the frame timestamp to the ControlList for the IPA to use
		 * as it does not receive the FrameBuffer object.
		 */
		ctrl.set(controls::SensorTimestamp, buffer->metadata().timestamp);
		bayerQueue_.push({ buffer, std::move(ctrl), delayContext });
	} else {
		embeddedQueue_.push(buffer);
	}

	handleState();
}

void RPiCameraData::ispInputDequeue(FrameBuffer *buffer)
{
	if (!isRunning())
		return;

	LOG(RPI, Debug) << "Stream ISP Input buffer complete"
			<< ", buffer id " << unicam_[Unicam::Image].getBufferId(buffer)
			<< ", timestamp: " << buffer->metadata().timestamp;

	/* The ISP input buffer gets re-queued into Unicam. */
	handleStreamBuffer(buffer, &unicam_[Unicam::Image]);
	handleState();
}

void RPiCameraData::ispOutputDequeue(FrameBuffer *buffer)
{
	RPi::Stream *stream = nullptr;
	int index;

	if (!isRunning())
		return;

	for (RPi::Stream &s : isp_) {
		index = s.getBufferId(buffer);
		if (index != -1) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our ISP output streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer complete"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	/*
	 * ISP statistics buffer must not be re-queued or sent back to the
	 * application until after the IPA signals so.
	 */
	if (stream == &isp_[Isp::Stats]) {
		ipa_->signalStatReady(RPi::MaskStats | static_cast<unsigned int>(index),
				      requestQueue_.front()->sequence());
	} else {
		/* Any other ISP output can be handed back to the application now. */
		handleStreamBuffer(buffer, stream);
	}

	/*
	 * Increment the number of ISP outputs generated.
	 * This is needed to track dropped frames.
	 */
	ispOutputCount_++;

	handleState();
}

void RPiCameraData::clearIncompleteRequests()
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
		requestQueue_.pop_front();
	}
}

void RPiCameraData::handleStreamBuffer(FrameBuffer *buffer, RPi::Stream *stream)
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

void RPiCameraData::handleExternalBuffer(FrameBuffer *buffer, RPi::Stream *stream)
{
	unsigned int id = stream->getBufferId(buffer);

	if (!(id & RPi::MaskExternalBuffer))
		return;

	/* Stop the Stream object from tracking the buffer. */
	stream->removeExternalBuffer(buffer);
}

void RPiCameraData::handleState()
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

void RPiCameraData::checkRequestCompleted()
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
		requestQueue_.pop_front();
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
			LOG(RPI, Debug) << "Dropping frame at the request of the IPA ("
					<< dropFrameCount_ << " left)";
		}
	}
}

Rectangle RPiCameraData::scaleIspCrop(const Rectangle &ispCrop) const
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

void RPiCameraData::applyScalerCrop(const ControlList &controls)
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
			isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &ispCrop);
			ispCrop_ = ispCrop;

			/*
			 * Also update the ScalerCrop in the metadata with what we actually
			 * used. But we must first rescale that from ISP (camera mode) pixels
			 * back into sensor native pixels.
			 */
			scalerCrop_ = scaleIspCrop(ispCrop_);
		}
	}
}

void RPiCameraData::fillRequestMetadata(const ControlList &bufferControls,
					Request *request)
{
	request->metadata().set(controls::SensorTimestamp,
				bufferControls.get(controls::SensorTimestamp).value_or(0));

	request->metadata().set(controls::ScalerCrop, scalerCrop_);
}

void RPiCameraData::tryRunPipeline()
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
	ipa_->signalQueueRequest(request->controls());

	/* Set our state to say the pipeline is active. */
	state_ = State::Busy;

	unsigned int bayerId = unicam_[Unicam::Image].getBufferId(bayerFrame.buffer);

	LOG(RPI, Debug) << "Signalling signalIspPrepare:"
			<< " Bayer buffer id: " << bayerId;

	ipa::RPi::ISPConfig ispPrepare;
	ispPrepare.bayerBufferId = RPi::MaskBayerData | bayerId;
	ispPrepare.controls = std::move(bayerFrame.controls);
	ispPrepare.ipaContext = request->sequence();
	ispPrepare.delayContext = bayerFrame.delayContext;

	if (embeddedBuffer) {
		unsigned int embeddedId = unicam_[Unicam::Embedded].getBufferId(embeddedBuffer);

		ispPrepare.embeddedBufferId = RPi::MaskEmbeddedData | embeddedId;
		ispPrepare.embeddedBufferPresent = true;

		LOG(RPI, Debug) << "Signalling signalIspPrepare:"
				<< " Embedded buffer id: " << embeddedId;
	}

	ipa_->signalIspPrepare(ispPrepare);
}

bool RPiCameraData::findMatchingBuffers(BayerFrame &bayerFrame, FrameBuffer *&embeddedBuffer)
{
	if (bayerQueue_.empty())
		return false;

	/*
	 * Find the embedded data buffer with a matching timestamp to pass to
	 * the IPA. Any embedded buffers with a timestamp lower than the
	 * current bayer buffer will be removed and re-queued to the driver.
	 */
	uint64_t ts = bayerQueue_.front().buffer->metadata().timestamp;
	embeddedBuffer = nullptr;
	while (!embeddedQueue_.empty()) {
		FrameBuffer *b = embeddedQueue_.front();
		if (b->metadata().timestamp < ts) {
			embeddedQueue_.pop();
			unicam_[Unicam::Embedded].returnBuffer(b);
			LOG(RPI, Debug) << "Dropping unmatched input frame in stream "
					<< unicam_[Unicam::Embedded].name();
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
		if (embeddedQueue_.empty()) {
			/*
			 * If the embedded buffer queue is empty, wait for the next
			 * buffer to arrive - dequeue ordering may send the image
			 * buffer first.
			 */
			LOG(RPI, Debug) << "Waiting for next embedded buffer.";
			return false;
		}

		/* Log if there is no matching embedded data buffer found. */
		LOG(RPI, Debug) << "Returning bayer frame without a matching embedded buffer.";
	}

	bayerFrame = std::move(bayerQueue_.front());
	bayerQueue_.pop();

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerRPi)

} /* namespace libcamera */
