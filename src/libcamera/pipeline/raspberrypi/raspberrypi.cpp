/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2020, Raspberry Pi (Trading) Ltd.
 *
 * raspberrypi.cpp - Pipeline handler for Raspberry Pi devices
 */
#include <algorithm>
#include <assert.h>
#include <fcntl.h>
#include <mutex>
#include <queue>
#include <sys/mman.h>
#include <unordered_set>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/file_descriptor.h>
#include <libcamera/formats.h>
#include <libcamera/ipa/raspberrypi.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>

#include <linux/videodev2.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/utils.h"
#include "libcamera/internal/v4l2_controls.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "dma_heaps.h"
#include "rpi_stream.h"
#include "staggered_ctrl.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(RPI)

namespace {

bool isRaw(PixelFormat &pixFmt)
{
	/*
	 * The isRaw test might be redundant right now the pipeline handler only
	 * supports RAW sensors. Leave it in for now, just as a sanity check.
	 */
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

V4L2DeviceFormat findBestMode(V4L2VideoDevice::Formats &formatsMap,
			      const Size &req)
{
	double bestScore = std::numeric_limits<double>::max(), score;
	V4L2DeviceFormat bestMode = {};

#define PENALTY_AR		1500.0
#define PENALTY_8BIT		2000.0
#define PENALTY_10BIT		1000.0
#define PENALTY_12BIT		   0.0
#define PENALTY_UNPACKED	 500.0

	/* Calculate the closest/best mode from the user requested size. */
	for (const auto &iter : formatsMap) {
		V4L2PixelFormat v4l2Format = iter.first;
		const PixelFormatInfo &info = PixelFormatInfo::info(v4l2Format);

		for (const SizeRange &sz : iter.second) {
			double modeWidth = sz.contains(req) ? req.width : sz.max.width;
			double modeHeight = sz.contains(req) ? req.height : sz.max.height;
			double reqAr = static_cast<double>(req.width) / req.height;
			double modeAr = modeWidth / modeHeight;

			/* Score the dimensions for closeness. */
			score = scoreFormat(req.width, modeWidth);
			score += scoreFormat(req.height, modeHeight);
			score += PENALTY_AR * scoreFormat(reqAr, modeAr);

			/* Add any penalties... this is not an exact science! */
			if (!info.packed)
				score += PENALTY_UNPACKED;

			if (info.bitsPerPixel == 12)
				score += PENALTY_12BIT;
			else if (info.bitsPerPixel == 10)
				score += PENALTY_10BIT;
			else if (info.bitsPerPixel == 8)
				score += PENALTY_8BIT;

			if (score <= bestScore) {
				bestScore = score;
				bestMode.fourcc = v4l2Format;
				bestMode.size = Size(modeWidth, modeHeight);
			}

			LOG(RPI, Info) << "Mode: " << modeWidth << "x" << modeHeight
				       << " fmt " << v4l2Format.toString()
				       << " Score: " << score
				       << " (best " << bestScore << ")";
		}
	}

	return bestMode;
}

enum class Unicam : unsigned int { Image, Embedded };
enum class Isp : unsigned int { Input, Output0, Output1, Stats };

} /* namespace */

class RPiCameraData : public CameraData
{
public:
	RPiCameraData(PipelineHandler *pipe)
		: CameraData(pipe), sensor_(nullptr), state_(State::Stopped),
		  supportsFlips_(false), flipsAlterBayerOrder_(false),
		  updateScalerCrop_(true), dropFrameCount_(0), ispOutputCount_(0)
	{
	}

	void frameStarted(uint32_t sequence);

	int loadIPA();
	int configureIPA(const CameraConfiguration *config);

	void queueFrameAction(unsigned int frame, const IPAOperationData &action);

	/* bufferComplete signal handlers. */
	void unicamBufferDequeue(FrameBuffer *buffer);
	void ispInputDequeue(FrameBuffer *buffer);
	void ispOutputDequeue(FrameBuffer *buffer);

	void clearIncompleteRequests();
	void handleStreamBuffer(FrameBuffer *buffer, RPi::Stream *stream);
	void handleExternalBuffer(FrameBuffer *buffer, RPi::Stream *stream);
	void handleState();

	CameraSensor *sensor_;
	/* Array of Unicam and ISP device streams and associated buffers/streams. */
	RPi::Device<Unicam, 2> unicam_;
	RPi::Device<Isp, 4> isp_;
	/* The vector below is just for convenience when iterating over all streams. */
	std::vector<RPi::Stream *> streams_;
	/* Stores the ids of the buffers mapped in the IPA. */
	std::unordered_set<unsigned int> ipaBuffers_;

	/* DMAHEAP allocation helper. */
	RPi::DmaHeap dmaHeap_;
	FileDescriptor lsTable_;

	RPi::StaggeredCtrl staggeredCtrl_;
	uint32_t expectedSequence_;
	bool sensorMetadata_;

	/*
	 * All the functions in this class are called from a single calling
	 * thread. So, we do not need to have any mutex to protect access to any
	 * of the variables below.
	 */
	enum class State { Stopped, Idle, Busy, IpaComplete };
	State state_;
	std::queue<FrameBuffer *> bayerQueue_;
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
	CameraSensorInfo sensorInfo_;
	Rectangle ispCrop_; /* crop in ISP (camera mode) pixels */
	Rectangle scalerCrop_; /* crop in sensor native pixels */
	bool updateScalerCrop_;
	Size ispMinCropSize_;

	unsigned int dropFrameCount_;

private:
	void checkRequestCompleted();
	void tryRunPipeline();
	void tryFlushQueues();
	FrameBuffer *updateQueue(std::queue<FrameBuffer *> &q, uint64_t timestamp,
				 RPi::Stream *stream);

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

	CameraConfiguration *generateConfiguration(Camera *camera, const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	RPiCameraData *cameraData(const Camera *camera)
	{
		return static_cast<RPiCameraData *>(PipelineHandler::cameraData(camera));
	}

	int queueAllBuffers(Camera *camera);
	int prepareBuffers(Camera *camera);
	void freeBuffers(Camera *camera);
	void mapBuffers(Camera *camera, const RPi::BufferMap &buffers, unsigned int mask);

	MediaDevice *unicam_;
	MediaDevice *isp_;
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
			V4L2VideoDevice::Formats fmts = data_->unicam_[Unicam::Image].dev()->formats();
			V4L2DeviceFormat sensorFormat = findBestMode(fmts, cfg.size);
			int ret = data_->unicam_[Unicam::Image].dev()->tryFormat(&sensorFormat);
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
			V4L2PixelFormat fourcc = sensorFormat.fourcc;
			if (data_->flipsAlterBayerOrder_) {
				BayerFormat bayer(fourcc);
				bayer.order = data_->nativeBayerOrder_;
				bayer = bayer.transform(combined);
				fourcc = bayer.toV4L2PixelFormat();
			}

			PixelFormat sensorPixFormat = fourcc.toPixelFormat();
			if (cfg.size != sensorFormat.size ||
			    cfg.pixelFormat != sensorPixFormat) {
				cfg.size = sensorFormat.size;
				cfg.pixelFormat = sensorPixFormat;
				status = Adjusted;
			}

			cfg.stride = sensorFormat.planes[0].bpl;
			cfg.frameSize = sensorFormat.planes[0].size;

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

		if (fmts.find(V4L2PixelFormat::fromPixelFormat(cfgPixFmt, false)) == fmts.end()) {
			/* If we cannot find a native format, use a default one. */
			cfgPixFmt = formats::NV12;
			status = Adjusted;
		}

		V4L2DeviceFormat format = {};
		format.fourcc = dev->toV4L2PixelFormat(cfg.pixelFormat);
		format.size = cfg.size;

		int ret = dev->tryFormat(&format);
		if (ret)
			return Invalid;

		cfg.stride = format.planes[0].bpl;
		cfg.frameSize = format.planes[0].size;

	}

	return status;
}

PipelineHandlerRPi::PipelineHandlerRPi(CameraManager *manager)
	: PipelineHandler(manager), unicam_(nullptr), isp_(nullptr)
{
}

CameraConfiguration *PipelineHandlerRPi::generateConfiguration(Camera *camera,
							       const StreamRoles &roles)
{
	RPiCameraData *data = cameraData(camera);
	CameraConfiguration *config = new RPiCameraConfiguration(data);
	V4L2DeviceFormat sensorFormat;
	unsigned int bufferCount;
	PixelFormat pixelFormat;
	V4L2VideoDevice::Formats fmts;
	Size size;

	if (roles.empty())
		return config;

	unsigned int rawCount = 0;
	unsigned int outCount = 0;
	for (const StreamRole role : roles) {
		switch (role) {
		case StreamRole::Raw:
			size = data->sensor_->resolution();
			fmts = data->unicam_[Unicam::Image].dev()->formats();
			sensorFormat = findBestMode(fmts, size);
			pixelFormat = sensorFormat.fourcc.toPixelFormat();
			ASSERT(pixelFormat.isValid());
			bufferCount = 2;
			rawCount++;
			break;

		case StreamRole::StillCapture:
			fmts = data->isp_[Isp::Output0].dev()->formats();
			pixelFormat = formats::NV12;
			/* Return the largest sensor resolution. */
			size = data->sensor_->resolution();
			bufferCount = 1;
			outCount++;
			break;

		case StreamRole::VideoRecording:
			fmts = data->isp_[Isp::Output0].dev()->formats();
			pixelFormat = formats::NV12;
			size = { 1920, 1080 };
			bufferCount = 4;
			outCount++;
			break;

		case StreamRole::Viewfinder:
			fmts = data->isp_[Isp::Output0].dev()->formats();
			pixelFormat = formats::ARGB8888;
			size = { 800, 600 };
			bufferCount = 4;
			outCount++;
			break;

		default:
			LOG(RPI, Error) << "Requested stream role not supported: "
					<< role;
			delete config;
			return nullptr;
		}

		if (rawCount > 1 || outCount > 2) {
			LOG(RPI, Error) << "Invalid stream roles requested";
			delete config;
			return nullptr;
		}

		/* Translate the V4L2PixelFormat to PixelFormat. */
		std::map<PixelFormat, std::vector<SizeRange>> deviceFormats;
		for (const auto &format : fmts) {
			PixelFormat pf = format.first.toPixelFormat();
			if (pf.isValid())
				deviceFormats[pf] = format.second;
		}

		/* Add the stream format based on the device node used for the use case. */
		StreamFormats formats(deviceFormats);
		StreamConfiguration cfg(formats);
		cfg.size = size;
		cfg.pixelFormat = pixelFormat;
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

	/* Start by resetting the Unicam and ISP stream states. */
	for (auto const stream : data->streams_)
		stream->reset();

	Size maxSize, sensorSize;
	unsigned int maxIndex = 0;
	bool rawStream = false;

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
		} else {
			if (cfg.size > maxSize) {
				maxSize = config->at(i).size;
				maxIndex = i;
			}
		}
	}

	/* First calculate the best sensor mode we can use based on the user request. */
	V4L2VideoDevice::Formats fmts = data->unicam_[Unicam::Image].dev()->formats();
	V4L2DeviceFormat sensorFormat = findBestMode(fmts, rawStream ? sensorSize : maxSize);

	/*
	 * Unicam image output format. The ISP input format gets set at start,
	 * just in case we have swapped bayer orders due to flips.
	 */
	ret = data->unicam_[Unicam::Image].dev()->setFormat(&sensorFormat);
	if (ret)
		return ret;

	LOG(RPI, Info) << "Sensor: " << camera->id()
		       << " - Selected mode: " << sensorFormat.toString();

	/*
	 * This format may be reset on start() if the bayer order has changed
	 * because of flips in the sensor.
	 */
	ret = data->isp_[Isp::Input].dev()->setFormat(&sensorFormat);

	/*
	 * See which streams are requested, and route the user
	 * StreamConfiguration appropriately.
	 */
	V4L2DeviceFormat format = {};
	for (unsigned i = 0; i < config->size(); i++) {
		StreamConfiguration &cfg = config->at(i);

		if (isRaw(cfg.pixelFormat)) {
			cfg.setStream(&data->unicam_[Unicam::Image]);
			/*
			 * We must set both Unicam streams as external, even
			 * though the application may only request RAW frames.
			 * This is because we match timestamps on both streams
			 * to synchronise buffers.
			 */
			data->unicam_[Unicam::Image].setExternal(true);
			data->unicam_[Unicam::Embedded].setExternal(true);
			continue;
		}

		if (i == maxIndex) {
			/* ISP main output format. */
			V4L2VideoDevice *dev = data->isp_[Isp::Output0].dev();
			V4L2PixelFormat fourcc = dev->toV4L2PixelFormat(cfg.pixelFormat);
			format.size = cfg.size;
			format.fourcc = fourcc;

			ret = dev->setFormat(&format);
			if (ret)
				return -EINVAL;

			if (format.size != cfg.size || format.fourcc != fourcc) {
				LOG(RPI, Error)
					<< "Failed to set format on ISP capture0 device: "
					<< format.toString();
				return -EINVAL;
			}

			cfg.setStream(&data->isp_[Isp::Output0]);
			data->isp_[Isp::Output0].setExternal(true);
		}

		/*
		 * ISP second output format. This fallthrough means that if a
		 * second output stream has not been configured, we simply use
		 * the Output0 configuration.
		 */
		V4L2VideoDevice *dev = data->isp_[Isp::Output1].dev();
		format.fourcc = dev->toV4L2PixelFormat(cfg.pixelFormat);
		format.size = cfg.size;

		ret = dev->setFormat(&format);
		if (ret) {
			LOG(RPI, Error)
				<< "Failed to set format on ISP capture1 device: "
				<< format.toString();
			return ret;
		}
		/*
		 * If we have not yet provided a stream for this config, it
		 * means this is to be routed from Output1.
		 */
		if (!cfg.stream()) {
			cfg.setStream(&data->isp_[Isp::Output1]);
			data->isp_[Isp::Output1].setExternal(true);
		}
	}

	/* ISP statistics output format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_BCM2835_ISP_STATS);
	ret = data->isp_[Isp::Stats].dev()->setFormat(&format);
	if (ret) {
		LOG(RPI, Error) << "Failed to set format on ISP stats stream: "
				<< format.toString();
		return ret;
	}

	/* Unicam embedded data output format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_SENSOR_DATA);
	LOG(RPI, Debug) << "Setting embedded data format.";
	ret = data->unicam_[Unicam::Embedded].dev()->setFormat(&format);
	if (ret) {
		LOG(RPI, Error) << "Failed to set format on Unicam embedded: "
				<< format.toString();
		return ret;
	}

	/* Figure out the smallest selection the ISP will allow. */
	Rectangle testCrop(0, 0, 1, 1);
	data->isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &testCrop);
	data->ispMinCropSize_ = testCrop.size();

	/* Adjust aspect ratio by providing crops on the input image. */
	Size size = sensorFormat.size.boundedToAspectRatio(maxSize);
	Rectangle crop = size.centeredTo(Rectangle(sensorFormat.size).center());
	data->ispCrop_ = crop;

	data->isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &crop);

	ret = data->configureIPA(config);
	if (ret)
		LOG(RPI, Error) << "Failed to configure the IPA: " << ret;

	/*
	 * Update the ScalerCropMaximum to the correct value for this camera mode.
	 * For us, it's the same as the "analogue crop".
	 *
	 * \todo Make this property the ScalerCrop maximum value when dynamic
	 * controls are available and set it at validate() time
	 */
	data->properties_.set(properties::ScalerCropMaximum, data->sensorInfo_.analogCrop);

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

int PipelineHandlerRPi::start(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = prepareBuffers(camera);
	if (ret) {
		LOG(RPI, Error) << "Failed to allocate buffers";
		stop(camera);
		return ret;
	}

	ret = queueAllBuffers(camera);
	if (ret) {
		LOG(RPI, Error) << "Failed to queue buffers";
		stop(camera);
		return ret;
	}

	/* Start the IPA. */
	ret = data->ipa_->start();
	if (ret) {
		LOG(RPI, Error)
			<< "Failed to start IPA for " << camera->id();
		stop(camera);
		return ret;
	}

	/*
	 * IPA configure may have changed the sensor flips - hence the bayer
	 * order. Get the sensor format and set the ISP input now.
	 */
	V4L2DeviceFormat sensorFormat;
	data->unicam_[Unicam::Image].dev()->getFormat(&sensorFormat);
	ret = data->isp_[Isp::Input].dev()->setFormat(&sensorFormat);
	if (ret) {
		stop(camera);
		return ret;
	}

	/* Enable SOF event generation. */
	data->unicam_[Unicam::Image].dev()->setFrameStartEnabled(true);

	/*
	 * Write the last set of gain and exposure values to the camera before
	 * starting. First check that the staggered ctrl has been initialised
	 * by configure().
	 */
	ASSERT(data->staggeredCtrl_);
	data->staggeredCtrl_.reset();
	data->staggeredCtrl_.write();
	data->expectedSequence_ = 0;

	data->state_ = RPiCameraData::State::Idle;

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

void PipelineHandlerRPi::stop(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);

	data->state_ = RPiCameraData::State::Stopped;

	/* Disable SOF event generation. */
	data->unicam_[Unicam::Image].dev()->setFrameStartEnabled(false);

	/* This also stops the streams. */
	data->clearIncompleteRequests();
	data->bayerQueue_ = {};
	data->embeddedQueue_ = {};

	/* Stop the IPA. */
	data->ipa_->stop();

	freeBuffers(camera);
}

int PipelineHandlerRPi::queueRequestDevice(Camera *camera, Request *request)
{
	RPiCameraData *data = cameraData(camera);

	if (data->state_ == RPiCameraData::State::Stopped)
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
	DeviceMatch isp("bcm2835-isp");

	unicam.add("unicam-embedded");
	unicam.add("unicam-image");

	isp.add("bcm2835-isp0-output0"); /* Input */
	isp.add("bcm2835-isp0-capture1"); /* Output 0 */
	isp.add("bcm2835-isp0-capture2"); /* Output 1 */
	isp.add("bcm2835-isp0-capture3"); /* Stats */

	unicam_ = acquireMediaDevice(enumerator, unicam);
	if (!unicam_)
		return false;

	isp_ = acquireMediaDevice(enumerator, isp);
	if (!isp_)
		return false;

	std::unique_ptr<RPiCameraData> data = std::make_unique<RPiCameraData>(this);
	if (!data->dmaHeap_.isValid())
		return false;

	/* Locate and open the unicam video streams. */
	data->unicam_[Unicam::Embedded] = RPi::Stream("Unicam Embedded", unicam_->getEntityByName("unicam-embedded"));
	data->unicam_[Unicam::Image] = RPi::Stream("Unicam Image", unicam_->getEntityByName("unicam-image"));

	/* Tag the ISP input stream as an import stream. */
	data->isp_[Isp::Input] = RPi::Stream("ISP Input", isp_->getEntityByName("bcm2835-isp0-output0"), true);
	data->isp_[Isp::Output0] = RPi::Stream("ISP Output0", isp_->getEntityByName("bcm2835-isp0-capture1"));
	data->isp_[Isp::Output1] = RPi::Stream("ISP Output1", isp_->getEntityByName("bcm2835-isp0-capture2"));
	data->isp_[Isp::Stats] = RPi::Stream("ISP Stats", isp_->getEntityByName("bcm2835-isp0-capture3"));

	/* This is just for convenience so that we can easily iterate over all streams. */
	for (auto &stream : data->unicam_)
		data->streams_.push_back(&stream);
	for (auto &stream : data->isp_)
		data->streams_.push_back(&stream);

	/* Open all Unicam and ISP streams. */
	for (auto const stream : data->streams_) {
		if (stream->dev()->open())
			return false;
	}

	/* Wire up all the buffer connections. */
	data->unicam_[Unicam::Image].dev()->frameStart.connect(data.get(), &RPiCameraData::frameStarted);
	data->unicam_[Unicam::Image].dev()->bufferReady.connect(data.get(), &RPiCameraData::unicamBufferDequeue);
	data->unicam_[Unicam::Embedded].dev()->bufferReady.connect(data.get(), &RPiCameraData::unicamBufferDequeue);
	data->isp_[Isp::Input].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispInputDequeue);
	data->isp_[Isp::Output0].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispOutputDequeue);
	data->isp_[Isp::Output1].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispOutputDequeue);
	data->isp_[Isp::Stats].dev()->bufferReady.connect(data.get(), &RPiCameraData::ispOutputDequeue);

	/* Identify the sensor. */
	for (MediaEntity *entity : unicam_->entities()) {
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR) {
			data->sensor_ = new CameraSensor(entity);
			break;
		}
	}

	if (!data->sensor_)
		return false;

	if (data->sensor_->init())
		return false;

	if (data->loadIPA()) {
		LOG(RPI, Error) << "Failed to load a suitable IPA library";
		return false;
	}

	/* Register the controls that the Raspberry Pi IPA can handle. */
	data->controlInfo_ = RPi::Controls;
	/* Initialize the camera properties. */
	data->properties_ = data->sensor_->properties();

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
	 * As part of answering the final question, we reset the camera to
	 * no transform at all.
	 */

	V4L2VideoDevice *dev = data->unicam_[Unicam::Image].dev();
	const struct v4l2_query_ext_ctrl *hflipCtrl = dev->controlInfo(V4L2_CID_HFLIP);
	if (hflipCtrl) {
		/* We assume it will support vflips too... */
		data->supportsFlips_ = true;
		data->flipsAlterBayerOrder_ = hflipCtrl->flags & V4L2_CTRL_FLAG_MODIFY_LAYOUT;

		ControlList ctrls(dev->controls());
		ctrls.set(V4L2_CID_HFLIP, 0);
		ctrls.set(V4L2_CID_VFLIP, 0);
		dev->setControls(&ctrls);
	}

	/* Look for a valid Bayer format. */
	BayerFormat bayerFormat;
	for (const auto &iter : dev->formats()) {
		V4L2PixelFormat v4l2Format = iter.first;
		bayerFormat = BayerFormat(v4l2Format);
		if (bayerFormat.isValid())
			break;
	}

	if (!bayerFormat.isValid()) {
		LOG(RPI, Error) << "No Bayer format found";
		return false;
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
	std::shared_ptr<Camera> camera =
		Camera::create(this, data->sensor_->id(), streams);
	registerCamera(std::move(camera), std::move(data));

	return true;
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
	int ret;

	/*
	 * Decide how many internal buffers to allocate. For now, simply look
	 * at how many external buffers will be provided. Will need to improve
	 * this logic. However, we really must have all streams allocate the same
	 * number of buffers to simplify error handling in queueRequestDevice().
	 */
	unsigned int maxBuffers = 0;
	for (const Stream *s : camera->streams())
		if (static_cast<const RPi::Stream *>(s)->isExternal())
			maxBuffers = std::max(maxBuffers, s->configuration().bufferCount);

	for (auto const stream : data->streams_) {
		ret = stream->prepareBuffers(maxBuffers);
		if (ret < 0)
			return ret;
	}

	/*
	 * Pass the stats and embedded data buffers to the IPA. No other
	 * buffers need to be passed.
	 */
	mapBuffers(camera, data->isp_[Isp::Stats].getBuffers(), RPi::BufferMask::STATS);
	mapBuffers(camera, data->unicam_[Unicam::Embedded].getBuffers(), RPi::BufferMask::EMBEDDED_DATA);

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
		ipaBuffers.push_back({ .id = mask | it.first,
				       .planes = it.second->planes() });
		data->ipaBuffers_.insert(mask | it.first);
	}

	data->ipa_->mapBuffers(ipaBuffers);
}

void PipelineHandlerRPi::freeBuffers(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);

	/* Copy the buffer ids from the unordered_set to a vector to pass to the IPA. */
	std::vector<unsigned int> ipaBuffers(data->ipaBuffers_.begin(), data->ipaBuffers_.end());
	data->ipa_->unmapBuffers(ipaBuffers);
	data->ipaBuffers_.clear();

	for (auto const stream : data->streams_)
		stream->releaseBuffers();
}

void RPiCameraData::frameStarted(uint32_t sequence)
{
	LOG(RPI, Debug) << "frame start " << sequence;

	/* Write any controls for the next frame as soon as we can. */
	staggeredCtrl_.write();
}

int RPiCameraData::loadIPA()
{
	ipa_ = IPAManager::createIPA(pipe_, 1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->queueFrameAction.connect(this, &RPiCameraData::queueFrameAction);

	IPASettings settings{
		.configurationFile = ipa_->configurationFile(sensor_->model() + ".json")
	};

	return ipa_->init(settings);
}

int RPiCameraData::configureIPA(const CameraConfiguration *config)
{
	/* We know config must be an RPiCameraConfiguration. */
	const RPiCameraConfiguration *rpiConfig =
		static_cast<const RPiCameraConfiguration *>(config);

	std::map<unsigned int, IPAStream> streamConfig;
	std::map<unsigned int, const ControlInfoMap &> entityControls;
	IPAOperationData ipaConfig = {};

	/* Get the device format to pass to the IPA. */
	V4L2DeviceFormat sensorFormat;
	unicam_[Unicam::Image].dev()->getFormat(&sensorFormat);
	/* Inform IPA of stream configuration and sensor controls. */
	unsigned int i = 0;
	for (auto const &stream : isp_) {
		if (stream.isExternal()) {
			streamConfig[i++] = {
				.pixelFormat = stream.configuration().pixelFormat,
				.size = stream.configuration().size
			};
		}
	}

	entityControls.emplace(0, unicam_[Unicam::Image].dev()->controls());
	entityControls.emplace(1, isp_[Isp::Input].dev()->controls());

	/* Always send the user transform to the IPA. */
	ipaConfig.data = { static_cast<unsigned int>(config->transform) };

	/* Allocate the lens shading table via dmaHeap and pass to the IPA. */
	if (!lsTable_.isValid()) {
		lsTable_ = dmaHeap_.alloc("ls_grid", RPi::MaxLsGridSize);
		if (!lsTable_.isValid())
			return -ENOMEM;

		/* Allow the IPA to mmap the LS table via the file descriptor. */
		ipaConfig.operation = RPi::IPA_CONFIG_LS_TABLE;
		ipaConfig.data.push_back(static_cast<unsigned int>(lsTable_.fd()));
	}

	/* We store the CameraSensorInfo for digital zoom calculations. */
	int ret = sensor_->sensorInfo(&sensorInfo_);
	if (ret) {
		LOG(RPI, Error) << "Failed to retrieve camera sensor info";
		return ret;
	}

	/* Ready the IPA - it must know about the sensor resolution. */
	IPAOperationData result;

	ipa_->configure(sensorInfo_, streamConfig, entityControls, ipaConfig,
			&result);

	unsigned int resultIdx = 0;
	if (result.operation & RPi::IPA_CONFIG_STAGGERED_WRITE) {
		/*
		 * Setup our staggered control writer with the sensor default
		 * gain and exposure delays.
		 */
		if (!staggeredCtrl_) {
			staggeredCtrl_.init(unicam_[Unicam::Image].dev(),
					    { { V4L2_CID_ANALOGUE_GAIN, result.data[resultIdx++] },
					      { V4L2_CID_EXPOSURE, result.data[resultIdx++] } });
			sensorMetadata_ = result.data[resultIdx++];
		}
	}

	if (result.operation & RPi::IPA_CONFIG_SENSOR) {
		const ControlList &ctrls = result.controls[0];
		if (!staggeredCtrl_.set(ctrls))
			LOG(RPI, Error) << "V4L2 staggered set failed";
	}

	if (result.operation & RPi::IPA_CONFIG_DROP_FRAMES) {
		/* Configure the number of dropped frames required on startup. */
		dropFrameCount_ = result.data[resultIdx++];
	}

	/*
	 * Configure the H/V flip controls based on the combination of
	 * the sensor and user transform.
	 */
	if (supportsFlips_) {
		ControlList ctrls(unicam_[Unicam::Image].dev()->controls());
		ctrls.set(V4L2_CID_HFLIP,
			  static_cast<int32_t>(!!(rpiConfig->combinedTransform_ & Transform::HFlip)));
		ctrls.set(V4L2_CID_VFLIP,
			  static_cast<int32_t>(!!(rpiConfig->combinedTransform_ & Transform::VFlip)));
		unicam_[Unicam::Image].dev()->setControls(&ctrls);
	}

	return 0;
}

void RPiCameraData::queueFrameAction([[maybe_unused]] unsigned int frame,
				     const IPAOperationData &action)
{
	/*
	 * The following actions can be handled when the pipeline handler is in
	 * a stopped state.
	 */
	switch (action.operation) {
	case RPi::IPA_ACTION_V4L2_SET_STAGGERED: {
		const ControlList &controls = action.controls[0];
		if (!staggeredCtrl_.set(controls))
			LOG(RPI, Error) << "V4L2 staggered set failed";
		goto done;
	}

	case RPi::IPA_ACTION_V4L2_SET_ISP: {
		ControlList controls = action.controls[0];
		isp_[Isp::Input].dev()->setControls(&controls);
		goto done;
	}
	}

	if (state_ == State::Stopped)
		goto done;

	/*
	 * The following actions must not be handled when the pipeline handler
	 * is in a stopped state.
	 */
	switch (action.operation) {
	case RPi::IPA_ACTION_STATS_METADATA_COMPLETE: {
		unsigned int bufferId = action.data[0];
		FrameBuffer *buffer = isp_[Isp::Stats].getBuffers().at(bufferId);

		handleStreamBuffer(buffer, &isp_[Isp::Stats]);

		/* Fill the Request metadata buffer with what the IPA has provided */
		Request *request = requestQueue_.front();
		request->metadata() = std::move(action.controls[0]);

		/*
		 * Also update the ScalerCrop in the metadata with what we actually
		 * used. But we must first rescale that from ISP (camera mode) pixels
		 * back into sensor native pixels.
		 *
		 * Sending this information on every frame may be helpful.
		 */
		if (updateScalerCrop_) {
			updateScalerCrop_ = false;
			scalerCrop_ = ispCrop_.scaledBy(sensorInfo_.analogCrop.size(),
							sensorInfo_.outputSize);
			scalerCrop_.translateBy(sensorInfo_.analogCrop.topLeft());
		}
		request->metadata().set(controls::ScalerCrop, scalerCrop_);

		state_ = State::IpaComplete;
		break;
	}

	case RPi::IPA_ACTION_EMBEDDED_COMPLETE: {
		unsigned int bufferId = action.data[0];
		FrameBuffer *buffer = unicam_[Unicam::Embedded].getBuffers().at(bufferId);
		handleStreamBuffer(buffer, &unicam_[Unicam::Embedded]);
		break;
	}

	case RPi::IPA_ACTION_RUN_ISP: {
		unsigned int bufferId = action.data[0];
		FrameBuffer *buffer = unicam_[Unicam::Image].getBuffers().at(bufferId);

		LOG(RPI, Debug) << "Input re-queue to ISP, buffer id " << bufferId
				<< ", timestamp: " << buffer->metadata().timestamp;

		isp_[Isp::Input].queueBuffer(buffer);
		ispOutputCount_ = 0;
		break;
	}

	default:
		LOG(RPI, Error) << "Unknown action " << action.operation;
		break;
	}

done:
	handleState();
}

void RPiCameraData::unicamBufferDequeue(FrameBuffer *buffer)
{
	RPi::Stream *stream = nullptr;
	int index;

	if (state_ == State::Stopped)
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
		bayerQueue_.push(buffer);
	} else {
		embeddedQueue_.push(buffer);

		std::unordered_map<uint32_t, int32_t> ctrl;
		int offset = buffer->metadata().sequence - expectedSequence_;
		staggeredCtrl_.get(ctrl, offset);

		expectedSequence_ = buffer->metadata().sequence + 1;

		/*
		 * Sensor metadata is unavailable, so put the expected ctrl
		 * values (accounting for the staggered delays) into the empty
		 * metadata buffer.
		 */
		if (!sensorMetadata_) {
			const FrameBuffer &fb = buffer->planes();
			uint32_t *mem = static_cast<uint32_t *>(::mmap(nullptr, fb.planes()[0].length,
								       PROT_READ | PROT_WRITE,
								       MAP_SHARED,
								       fb.planes()[0].fd.fd(), 0));
			mem[0] = ctrl[V4L2_CID_EXPOSURE];
			mem[1] = ctrl[V4L2_CID_ANALOGUE_GAIN];
			munmap(mem, fb.planes()[0].length);
		}
	}

	handleState();
}

void RPiCameraData::ispInputDequeue(FrameBuffer *buffer)
{
	if (state_ == State::Stopped)
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

	if (state_ == State::Stopped)
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
		IPAOperationData op;
		op.operation = RPi::IPA_EVENT_SIGNAL_STAT_READY;
		op.data = { RPi::BufferMask::STATS | static_cast<unsigned int>(index) };
		ipa_->processEvent(op);
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
	 * Queue up any buffers passed in the request.
	 * This is needed because streamOff() will then mark the buffers as
	 * cancelled.
	 */
	for (auto const request : requestQueue_) {
		for (auto const stream : streams_) {
			if (!stream->isExternal())
				continue;

			FrameBuffer *buffer = request->findBuffer(stream);
			if (buffer)
				stream->queueBuffer(buffer);
		}
	}

	/* Stop all streams. */
	for (auto const stream : streams_)
		stream->dev()->streamOff();

	/*
	 * All outstanding requests (and associated buffers) must be returned
	 * back to the pipeline. The buffers would have been marked as
	 * cancelled by the call to streamOff() earlier.
	 */
	while (!requestQueue_.empty()) {
		Request *request = requestQueue_.front();
		/*
		 * A request could be partially complete,
		 * i.e. we have returned some buffers, but still waiting
		 * for others or waiting for metadata.
		 */
		for (auto const stream : streams_) {
			if (!stream->isExternal())
				continue;

			FrameBuffer *buffer = request->findBuffer(stream);
			/*
			 * Has the buffer already been handed back to the
			 * request? If not, do so now.
			 */
			if (buffer && buffer->request())
				pipe_->completeBuffer(camera_, request, buffer);
		}

		pipe_->completeRequest(camera_, request);
		requestQueue_.pop_front();
	}
}

void RPiCameraData::handleStreamBuffer(FrameBuffer *buffer, RPi::Stream *stream)
{
	if (stream->isExternal()) {
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
			pipe_->completeBuffer(camera_, request, buffer);
		} else {
			/*
			 * This buffer was not part of the Request, or there is no
			 * pending request, so we can recycle it.
			 */
			stream->returnBuffer(buffer);
		}
	} else {
		/* Simply re-queue the buffer to the requested stream. */
		stream->queueBuffer(buffer);
	}
}

void RPiCameraData::handleExternalBuffer(FrameBuffer *buffer, RPi::Stream *stream)
{
	unsigned int id = stream->getBufferId(buffer);

	if (!(id & RPi::BufferMask::EXTERNAL_BUFFER))
		return;

	/* Stop the Stream object from tracking the buffer. */
	stream->removeExternalBuffer(buffer);
}

void RPiCameraData::handleState()
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
		tryFlushQueues();
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

		pipe_->completeRequest(camera_, request);
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
			LOG(RPI, Info) << "Dropping frame at the request of the IPA ("
				       << dropFrameCount_ << " left)";
		}
	}
}

void RPiCameraData::tryRunPipeline()
{
	FrameBuffer *bayerBuffer, *embeddedBuffer;
	IPAOperationData op;

	/* If any of our request or buffer queues are empty, we cannot proceed. */
	if (state_ != State::Idle || requestQueue_.empty() ||
	    bayerQueue_.empty() || embeddedQueue_.empty())
		return;

	/* Start with the front of the bayer buffer queue. */
	bayerBuffer = bayerQueue_.front();

	/*
	 * Find the embedded data buffer with a matching timestamp to pass to
	 * the IPA. Any embedded buffers with a timestamp lower than the
	 * current bayer buffer will be removed and re-queued to the driver.
	 */
	embeddedBuffer = updateQueue(embeddedQueue_, bayerBuffer->metadata().timestamp,
				     &unicam_[Unicam::Embedded]);

	if (!embeddedBuffer) {
		LOG(RPI, Debug) << "Could not find matching embedded buffer";

		/*
		 * Look the other way, try to match a bayer buffer with the
		 * first embedded buffer in the queue. This will also do some
		 * housekeeping on the bayer image queue - clear out any
		 * buffers that are older than the first buffer in the embedded
		 * queue.
		 *
		 * But first check if the embedded queue has emptied out.
		 */
		if (embeddedQueue_.empty())
			return;

		embeddedBuffer = embeddedQueue_.front();
		bayerBuffer = updateQueue(bayerQueue_, embeddedBuffer->metadata().timestamp,
					  &unicam_[Unicam::Image]);

		if (!bayerBuffer) {
			LOG(RPI, Debug) << "Could not find matching bayer buffer - ending.";
			return;
		}
	}

	/* Take the first request from the queue and action the IPA. */
	Request *request = requestQueue_.front();

	if (request->controls().contains(controls::ScalerCrop)) {
		Rectangle nativeCrop = request->controls().get<Rectangle>(controls::ScalerCrop);

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
			/* queueFrameAction will have to update its scalerCrop_ */
			updateScalerCrop_ = true;
		}
	}

	/*
	 * Process all the user controls by the IPA. Once this is complete, we
	 * queue the ISP output buffer listed in the request to start the HW
	 * pipeline.
	 */
	op.operation = RPi::IPA_EVENT_QUEUE_REQUEST;
	op.controls = { request->controls() };
	ipa_->processEvent(op);

	/* Ready to use the buffers, pop them off the queue. */
	bayerQueue_.pop();
	embeddedQueue_.pop();

	/* Set our state to say the pipeline is active. */
	state_ = State::Busy;

	unsigned int bayerId = unicam_[Unicam::Image].getBufferId(bayerBuffer);
	unsigned int embeddedId = unicam_[Unicam::Embedded].getBufferId(embeddedBuffer);

	LOG(RPI, Debug) << "Signalling RPi::IPA_EVENT_SIGNAL_ISP_PREPARE:"
			<< " Bayer buffer id: " << bayerId
			<< " Embedded buffer id: " << embeddedId;

	op.operation = RPi::IPA_EVENT_SIGNAL_ISP_PREPARE;
	op.data = { RPi::BufferMask::EMBEDDED_DATA | embeddedId,
		    RPi::BufferMask::BAYER_DATA | bayerId };
	ipa_->processEvent(op);
}

void RPiCameraData::tryFlushQueues()
{
	/*
	 * It is possible for us to end up in a situation where all available
	 * Unicam buffers have been dequeued but do not match. This can happen
	 * when the system is heavily loaded and we get out of lock-step with
	 * the two channels.
	 *
	 * In such cases, the best thing to do is the re-queue all the buffers
	 * and give a chance for the hardware to return to lock-step. We do have
	 * to drop all interim frames.
	 */
	if (unicam_[Unicam::Image].getBuffers().size() == bayerQueue_.size() &&
	    unicam_[Unicam::Embedded].getBuffers().size() == embeddedQueue_.size()) {
		/* This cannot happen when Unicam streams are external. */
		assert(!unicam_[Unicam::Image].isExternal());

		LOG(RPI, Warning) << "Flushing all buffer queues!";

		while (!bayerQueue_.empty()) {
			unicam_[Unicam::Image].queueBuffer(bayerQueue_.front());
			bayerQueue_.pop();
		}

		while (!embeddedQueue_.empty()) {
			unicam_[Unicam::Embedded].queueBuffer(embeddedQueue_.front());
			embeddedQueue_.pop();
		}
	}
}

FrameBuffer *RPiCameraData::updateQueue(std::queue<FrameBuffer *> &q, uint64_t timestamp,
					RPi::Stream *stream)
{
	/*
	 * If the unicam streams are external (both have be to the same), then we
	 * can only return out the top buffer in the queue, and assume they have
	 * been synced by queuing at the same time. We cannot drop these frames,
	 * as they may have been provided externally.
	 */
	while (!q.empty()) {
		FrameBuffer *b = q.front();
		if (!stream->isExternal() && b->metadata().timestamp < timestamp) {
			q.pop();
			stream->queueBuffer(b);
			LOG(RPI, Warning) << "Dropping unmatched input frame in stream "
					  << stream->name();
		} else if (stream->isExternal() || b->metadata().timestamp == timestamp) {
			/* The calling function will pop the item from the queue. */
			return b;
		} else {
			break; /* Only higher timestamps from here. */
		}
	}

	return nullptr;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerRPi)

} /* namespace libcamera */
