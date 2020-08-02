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

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/file_descriptor.h>
#include <libcamera/formats.h>
#include <libcamera/ipa/raspberrypi.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <linux/videodev2.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/utils.h"
#include "libcamera/internal/v4l2_controls.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "dma_heaps.h"
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

} /* namespace */

/*
 * Device stream abstraction for either an internal or external stream.
 * Used for both Unicam and the ISP.
 */
class RPiStream : public Stream
{
public:
	RPiStream()
	{
	}

	RPiStream(const char *name, MediaEntity *dev, bool importOnly = false)
		: external_(false), importOnly_(importOnly), name_(name),
		  dev_(std::make_unique<V4L2VideoDevice>(dev))
	{
	}

	V4L2VideoDevice *dev() const
	{
		return dev_.get();
	}

	void setExternal(bool external)
	{
		external_ = external;
	}

	bool isExternal() const
	{
		/*
		 * Import streams cannot be external.
		 *
		 * RAW capture is a special case where we simply copy the RAW
		 * buffer out of the request. All other buffer handling happens
		 * as if the stream is internal.
		 */
		return external_ && !importOnly_;
	}

	bool isImporter() const
	{
		return importOnly_;
	}

	void reset()
	{
		external_ = false;
		internalBuffers_.clear();
	}

	std::string name() const
	{
		return name_;
	}

	void setExternalBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers)
	{
		externalBuffers_ = buffers;
	}

	const std::vector<std::unique_ptr<FrameBuffer>> *getBuffers() const
	{
		return external_ ? externalBuffers_ : &internalBuffers_;
	}

	void releaseBuffers()
	{
		dev_->releaseBuffers();
		if (!external_ && !importOnly_)
			internalBuffers_.clear();
	}

	int importBuffers(unsigned int count)
	{
		return dev_->importBuffers(count);
	}

	int allocateBuffers(unsigned int count)
	{
		return dev_->allocateBuffers(count, &internalBuffers_);
	}

	int queueBuffers()
	{
		if (external_)
			return 0;

		for (auto &b : internalBuffers_) {
			int ret = dev_->queueBuffer(b.get());
			if (ret) {
				LOG(RPI, Error) << "Failed to queue buffers for "
						<< name_;
				return ret;
			}
		}

		return 0;
	}

	bool findFrameBuffer(FrameBuffer *buffer) const
	{
		auto start = external_ ? externalBuffers_->begin() : internalBuffers_.begin();
		auto end = external_ ? externalBuffers_->end() : internalBuffers_.end();

		if (importOnly_)
			return false;

		if (std::find_if(start, end,
				 [buffer](std::unique_ptr<FrameBuffer> const &ref) { return ref.get() == buffer; }) != end)
			return true;

		return false;
	}

private:
	/*
	 * Indicates that this stream is active externally, i.e. the buffers
	 * are provided by the application.
	 */
	bool external_;
	/* Indicates that this stream only imports buffers, e.g. ISP input. */
	bool importOnly_;
	/* Stream name identifier. */
	std::string name_;
	/* The actual device stream. */
	std::unique_ptr<V4L2VideoDevice> dev_;
	/* Internally allocated framebuffers associated with this device stream. */
	std::vector<std::unique_ptr<FrameBuffer>> internalBuffers_;
	/* Externally allocated framebuffers associated with this device stream. */
	std::vector<std::unique_ptr<FrameBuffer>> *externalBuffers_;
};

/*
 * The following class is just a convenient (and typesafe) array of device
 * streams indexed with an enum class.
 */
enum class Unicam : unsigned int { Image, Embedded };
enum class Isp : unsigned int { Input, Output0, Output1, Stats };

template<typename E, std::size_t N>
class RPiDevice : public std::array<class RPiStream, N>
{
private:
	constexpr auto index(E e) const noexcept
	{
		return static_cast<std::underlying_type_t<E>>(e);
	}
public:
	RPiStream &operator[](E e)
	{
		return std::array<class RPiStream, N>::operator[](index(e));
	}
	const RPiStream &operator[](E e) const
	{
		return std::array<class RPiStream, N>::operator[](index(e));
	}
};

class RPiCameraData : public CameraData
{
public:
	RPiCameraData(PipelineHandler *pipe)
		: CameraData(pipe), sensor_(nullptr), state_(State::Stopped),
		  dropFrame_(false), ispOutputCount_(0)
	{
	}

	void frameStarted(uint32_t sequence);

	int loadIPA();
	int configureIPA();

	void queueFrameAction(unsigned int frame, const IPAOperationData &action);

	/* bufferComplete signal handlers. */
	void unicamBufferDequeue(FrameBuffer *buffer);
	void ispInputDequeue(FrameBuffer *buffer);
	void ispOutputDequeue(FrameBuffer *buffer);

	void clearIncompleteRequests();
	void handleStreamBuffer(FrameBuffer *buffer, const RPiStream *stream);
	void handleState();

	CameraSensor *sensor_;
	/* Array of Unicam and ISP device streams and associated buffers/streams. */
	RPiDevice<Unicam, 2> unicam_;
	RPiDevice<Isp, 4> isp_;
	/* The vector below is just for convenience when iterating over all streams. */
	std::vector<RPiStream *> streams_;
	/* Buffers passed to the IPA. */
	std::vector<IPABuffer> ipaBuffers_;

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

private:
	void checkRequestCompleted();
	void tryRunPipeline();
	void tryFlushQueues();
	FrameBuffer *updateQueue(std::queue<FrameBuffer *> &q, uint64_t timestamp, V4L2VideoDevice *dev);

	bool dropFrame_;
	int ispOutputCount_;
};

class RPiCameraConfiguration : public CameraConfiguration
{
public:
	RPiCameraConfiguration(const RPiCameraData *data);

	Status validate() override;

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

			PixelFormat sensorPixFormat = sensorFormat.fourcc.toPixelFormat();
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
		case StreamRole::StillCaptureRaw:
			size = data->sensor_->resolution();
			fmts = data->unicam_[Unicam::Image].dev()->formats();
			sensorFormat = findBestMode(fmts, size);
			pixelFormat = sensorFormat.fourcc.toPixelFormat();
			ASSERT(pixelFormat.isValid());
			bufferCount = 1;
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
			PixelFormat pixelFormat = format.first.toPixelFormat();
			if (pixelFormat.isValid())
				deviceFormats[pixelFormat] = format.second;
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
			cfg.setStream(&data->isp_[Isp::Input]);
			data->isp_[Isp::Input].setExternal(true);
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

	/* Adjust aspect ratio by providing crops on the input image. */
	Rectangle crop{ 0, 0, sensorFormat.size };

	int ar = maxSize.height * sensorFormat.size.width - maxSize.width * sensorFormat.size.height;
	if (ar > 0)
		crop.width = maxSize.width * sensorFormat.size.height / maxSize.height;
	else if (ar < 0)
		crop.height = maxSize.height * sensorFormat.size.width / maxSize.width;

	crop.width &= ~1;
	crop.height &= ~1;

	crop.x = (sensorFormat.size.width - crop.width) >> 1;
	crop.y = (sensorFormat.size.height - crop.height) >> 1;
	data->isp_[Isp::Input].dev()->setSelection(V4L2_SEL_TGT_CROP, &crop);

	ret = data->configureIPA();
	if (ret)
		LOG(RPI, Error) << "Failed to configure the IPA: " << ret;

	return ret;
}

int PipelineHandlerRPi::exportFrameBuffers(Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	RPiStream *s = static_cast<RPiStream *>(stream);
	unsigned int count = stream->configuration().bufferCount;
	int ret = s->dev()->exportBuffers(count, buffers);

	s->setExternalBuffers(buffers);

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
	/* The default std::queue constructor is explicit with gcc 5 and 6. */
	data->bayerQueue_ = std::queue<FrameBuffer *>{};
	data->embeddedQueue_ = std::queue<FrameBuffer *>{};

	/* Stop the IPA. */
	data->ipa_->stop();

	freeBuffers(camera);
}

int PipelineHandlerRPi::queueRequestDevice(Camera *camera, Request *request)
{
	RPiCameraData *data = cameraData(camera);

	if (data->state_ == RPiCameraData::State::Stopped)
		return -EINVAL;

	/* Ensure all external streams have associated buffers! */
	for (auto &stream : data->isp_) {
		if (!stream.isExternal())
			continue;

		if (!request->findBuffer(&stream)) {
			LOG(RPI, Error) << "Attempt to queue request with invalid stream.";
			return -ENOENT;
		}
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

	/* Locate and open the unicam video streams. */
	data->unicam_[Unicam::Embedded] = RPiStream("Unicam Embedded", unicam_->getEntityByName("unicam-embedded"));
	data->unicam_[Unicam::Image] = RPiStream("Unicam Image", unicam_->getEntityByName("unicam-image"));

	/* Tag the ISP input stream as an import stream. */
	data->isp_[Isp::Input] = RPiStream("ISP Input", isp_->getEntityByName("bcm2835-isp0-output0"), true);
	data->isp_[Isp::Output0] = RPiStream("ISP Output0", isp_->getEntityByName("bcm2835-isp0-capture1"));
	data->isp_[Isp::Output1] = RPiStream("ISP Output1", isp_->getEntityByName("bcm2835-isp0-capture2"));
	data->isp_[Isp::Stats] = RPiStream("ISP Stats", isp_->getEntityByName("bcm2835-isp0-capture3"));

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
	data->controlInfo_ = RPiControls;
	/* Initialize the camera properties. */
	data->properties_ = data->sensor_->properties();

	/*
	 * List the available output streams.
	 * Currently cannot do Unicam streams!
	 */
	std::set<Stream *> streams;
	streams.insert(&data->isp_[Isp::Input]);
	streams.insert(&data->isp_[Isp::Output0]);
	streams.insert(&data->isp_[Isp::Output1]);
	streams.insert(&data->isp_[Isp::Stats]);

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
		ret = stream->queueBuffers();
		if (ret < 0)
			return ret;
	}

	return 0;
}

int PipelineHandlerRPi::prepareBuffers(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);
	int count, ret;

	/*
	 * Decide how many internal buffers to allocate. For now, simply look
	 * at how many external buffers will be provided. Will need to improve
	 * this logic.
	 */
	unsigned int maxBuffers = 0;
	for (const Stream *s : camera->streams())
		if (static_cast<const RPiStream *>(s)->isExternal())
			maxBuffers = std::max(maxBuffers, s->configuration().bufferCount);

	for (auto const stream : data->streams_) {
		if (stream->isExternal() || stream->isImporter()) {
			/*
			 * If a stream is marked as external reserve memory to
			 * prepare to import as many buffers are requested in
			 * the stream configuration.
			 *
			 * If a stream is an internal stream with importer
			 * role, reserve as many buffers as possible.
			 */
			unsigned int count = stream->isExternal()
						     ? stream->configuration().bufferCount
						     : maxBuffers;
			ret = stream->importBuffers(count);
			if (ret < 0)
				return ret;
		} else {
			/*
			 * If the stream is an internal exporter allocate and
			 * export as many buffers as possible to its internal
			 * pool.
			 */
			ret = stream->allocateBuffers(maxBuffers);
			if (ret < 0) {
				freeBuffers(camera);
				return ret;
			}
		}
	}

	/*
	 * Add cookies to the ISP Input buffers so that we can link them with
	 * the IPA and RPI_IPA_EVENT_SIGNAL_ISP_PREPARE event.
	 */
	count = 0;
	for (auto const &b : *data->unicam_[Unicam::Image].getBuffers()) {
		b->setCookie(count++);
	}

	/*
	 * Add cookies to the stats and embedded data buffers and link them with
	 * the IPA.
	 */
	count = 0;
	for (auto const &b : *data->isp_[Isp::Stats].getBuffers()) {
		b->setCookie(count++);
		data->ipaBuffers_.push_back({ .id = RPiIpaMask::STATS | b->cookie(),
					      .planes = b->planes() });
	}

	count = 0;
	for (auto const &b : *data->unicam_[Unicam::Embedded].getBuffers()) {
		b->setCookie(count++);
		data->ipaBuffers_.push_back({ .id = RPiIpaMask::EMBEDDED_DATA | b->cookie(),
					      .planes = b->planes() });
	}

	data->ipa_->mapBuffers(data->ipaBuffers_);

	return 0;
}

void PipelineHandlerRPi::freeBuffers(Camera *camera)
{
	RPiCameraData *data = cameraData(camera);

	std::vector<unsigned int> ids;
	for (IPABuffer &ipabuf : data->ipaBuffers_)
		ids.push_back(ipabuf.id);

	data->ipa_->unmapBuffers(ids);
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

int RPiCameraData::configureIPA()
{
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
			streamConfig[i] = {
				.pixelFormat = stream.configuration().pixelFormat,
				.size = stream.configuration().size
			};
		}
	}
	entityControls.emplace(0, unicam_[Unicam::Image].dev()->controls());
	entityControls.emplace(1, isp_[Isp::Input].dev()->controls());

	/* Allocate the lens shading table via dmaHeap and pass to the IPA. */
	if (!lsTable_.isValid()) {
		lsTable_ = dmaHeap_.alloc("ls_grid", MAX_LS_GRID_SIZE);
		if (!lsTable_.isValid())
			return -ENOMEM;

		/* Allow the IPA to mmap the LS table via the file descriptor. */
		ipaConfig.operation = RPI_IPA_CONFIG_LS_TABLE;
		ipaConfig.data = { static_cast<unsigned int>(lsTable_.fd()) };
	}

	CameraSensorInfo sensorInfo = {};
	int ret = sensor_->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(RPI, Error) << "Failed to retrieve camera sensor info";
		return ret;
	}

	/* Ready the IPA - it must know about the sensor resolution. */
	IPAOperationData result;

	ipa_->configure(sensorInfo, streamConfig, entityControls, ipaConfig,
			&result);

	if (result.operation & RPI_IPA_CONFIG_STAGGERED_WRITE) {
		/*
		 * Setup our staggered control writer with the sensor default
		 * gain and exposure delays.
		 */
		if (!staggeredCtrl_) {
			staggeredCtrl_.init(unicam_[Unicam::Image].dev(),
					    { { V4L2_CID_ANALOGUE_GAIN, result.data[0] },
					      { V4L2_CID_EXPOSURE, result.data[1] } });
			sensorMetadata_ = result.data[2];
		}

		/* Configure the H/V flip controls based on the sensor rotation. */
		ControlList ctrls(unicam_[Unicam::Image].dev()->controls());
		int32_t rotation = sensor_->properties().get(properties::Rotation);
		ctrls.set(V4L2_CID_HFLIP, static_cast<int32_t>(!!rotation));
		ctrls.set(V4L2_CID_VFLIP, static_cast<int32_t>(!!rotation));
		unicam_[Unicam::Image].dev()->setControls(&ctrls);
	}

	if (result.operation & RPI_IPA_CONFIG_SENSOR) {
		const ControlList &ctrls = result.controls[0];
		if (!staggeredCtrl_.set(ctrls))
			LOG(RPI, Error) << "V4L2 staggered set failed";
	}

	return 0;
}

void RPiCameraData::queueFrameAction(unsigned int frame, const IPAOperationData &action)
{
	/*
	 * The following actions can be handled when the pipeline handler is in
	 * a stopped state.
	 */
	switch (action.operation) {
	case RPI_IPA_ACTION_V4L2_SET_STAGGERED: {
		const ControlList &controls = action.controls[0];
		if (!staggeredCtrl_.set(controls))
			LOG(RPI, Error) << "V4L2 staggered set failed";
		goto done;
	}

	case RPI_IPA_ACTION_V4L2_SET_ISP: {
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
	case RPI_IPA_ACTION_STATS_METADATA_COMPLETE: {
		unsigned int bufferId = action.data[0];
		FrameBuffer *buffer = isp_[Isp::Stats].getBuffers()->at(bufferId).get();

		handleStreamBuffer(buffer, &isp_[Isp::Stats]);
		/* Fill the Request metadata buffer with what the IPA has provided */
		requestQueue_.front()->metadata() = std::move(action.controls[0]);
		state_ = State::IpaComplete;
		break;
	}

	case RPI_IPA_ACTION_EMBEDDED_COMPLETE: {
		unsigned int bufferId = action.data[0];
		FrameBuffer *buffer = unicam_[Unicam::Embedded].getBuffers()->at(bufferId).get();
		handleStreamBuffer(buffer, &unicam_[Unicam::Embedded]);
		break;
	}

	case RPI_IPA_ACTION_RUN_ISP_AND_DROP_FRAME:
	case RPI_IPA_ACTION_RUN_ISP: {
		unsigned int bufferId = action.data[0];
		FrameBuffer *buffer = unicam_[Unicam::Image].getBuffers()->at(bufferId).get();

		LOG(RPI, Debug) << "Input re-queue to ISP, buffer id " << buffer->cookie()
				<< ", timestamp: " << buffer->metadata().timestamp;

		isp_[Isp::Input].dev()->queueBuffer(buffer);
		dropFrame_ = (action.operation == RPI_IPA_ACTION_RUN_ISP_AND_DROP_FRAME) ? true : false;
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
	const RPiStream *stream = nullptr;

	if (state_ == State::Stopped)
		return;

	for (RPiStream const &s : unicam_) {
		if (s.findFrameBuffer(buffer)) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer dequeue"
			<< ", buffer id " << buffer->cookie()
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

	handleStreamBuffer(buffer, &unicam_[Unicam::Image]);
	handleState();
}

void RPiCameraData::ispOutputDequeue(FrameBuffer *buffer)
{
	const RPiStream *stream = nullptr;

	if (state_ == State::Stopped)
		return;

	for (RPiStream const &s : isp_) {
		if (s.findFrameBuffer(buffer)) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our ISP output streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer complete"
			<< ", buffer id " << buffer->cookie()
			<< ", timestamp: " << buffer->metadata().timestamp;

	handleStreamBuffer(buffer, stream);

	/*
	 * Increment the number of ISP outputs generated.
	 * This is needed to track dropped frames.
	 */
	ispOutputCount_++;

	/* If this is a stats output, hand it to the IPA now. */
	if (stream == &isp_[Isp::Stats]) {
		IPAOperationData op;
		op.operation = RPI_IPA_EVENT_SIGNAL_STAT_READY;
		op.data = { RPiIpaMask::STATS | buffer->cookie() };
		ipa_->processEvent(op);
	}

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
			if (stream->isExternal())
				stream->dev()->queueBuffer(request->findBuffer(stream));
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
			if (buffer->request())
				pipe_->completeBuffer(camera_, request, buffer);
		}

		pipe_->completeRequest(camera_, request);
		requestQueue_.pop_front();
	}
}

void RPiCameraData::handleStreamBuffer(FrameBuffer *buffer, const RPiStream *stream)
{
	if (stream->isExternal()) {
		if (!dropFrame_) {
			Request *request = buffer->request();
			pipe_->completeBuffer(camera_, request, buffer);
		}
	} else {
		/* Special handling for RAW buffer Requests.
		 *
		 * The ISP input stream is alway an import stream, but if the
		 * current Request has been made for a buffer on the stream,
		 * simply memcpy to the Request buffer and requeue back to the
		 * device.
		 */
		if (stream == &unicam_[Unicam::Image] && !dropFrame_) {
			const Stream *rawStream = static_cast<const Stream *>(&isp_[Isp::Input]);
			Request *request = requestQueue_.front();
			FrameBuffer *raw = request->findBuffer(const_cast<Stream *>(rawStream));
			if (raw) {
				raw->copyFrom(buffer);
				pipe_->completeBuffer(camera_, request, raw);
			}
		}

		/* Simply requeue the buffer. */
		stream->dev()->queueBuffer(buffer);
	}
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
		/* Fall through */

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
	if (!dropFrame_) {
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
	    ((ispOutputCount_ == 3 && dropFrame_) || requestCompleted)) {
		state_ = State::Idle;
		if (dropFrame_)
			LOG(RPI, Info) << "Dropping frame at the request of the IPA";
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
				     unicam_[Unicam::Embedded].dev());

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
					  unicam_[Unicam::Image].dev());

		if (!bayerBuffer) {
			LOG(RPI, Debug) << "Could not find matching bayer buffer - ending.";
			return;
		}
	}

	/*
	 * Take the first request from the queue and action the IPA.
	 * Unicam buffers for the request have already been queued as they come
	 * in.
	 */
	Request *request = requestQueue_.front();

	/*
	 * Process all the user controls by the IPA. Once this is complete, we
	 * queue the ISP output buffer listed in the request to start the HW
	 * pipeline.
	 */
	op.operation = RPI_IPA_EVENT_QUEUE_REQUEST;
	op.controls = { request->controls() };
	ipa_->processEvent(op);

	/* Queue up any ISP buffers passed into the request. */
	for (auto &stream : isp_) {
		if (stream.isExternal())
			stream.dev()->queueBuffer(request->findBuffer(&stream));
	}

	/* Ready to use the buffers, pop them off the queue. */
	bayerQueue_.pop();
	embeddedQueue_.pop();

	/* Set our state to say the pipeline is active. */
	state_ = State::Busy;

	LOG(RPI, Debug) << "Signalling RPI_IPA_EVENT_SIGNAL_ISP_PREPARE:"
			<< " Bayer buffer id: " << bayerBuffer->cookie()
			<< " Embedded buffer id: " << embeddedBuffer->cookie();

	op.operation = RPI_IPA_EVENT_SIGNAL_ISP_PREPARE;
	op.data = { RPiIpaMask::EMBEDDED_DATA | embeddedBuffer->cookie(),
		    RPiIpaMask::BAYER_DATA | bayerBuffer->cookie() };
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
	if (unicam_[Unicam::Image].getBuffers()->size() == bayerQueue_.size() &&
	    unicam_[Unicam::Embedded].getBuffers()->size() == embeddedQueue_.size()) {
		LOG(RPI, Warning) << "Flushing all buffer queues!";

		while (!bayerQueue_.empty()) {
			unicam_[Unicam::Image].dev()->queueBuffer(bayerQueue_.front());
			bayerQueue_.pop();
		}

		while (!embeddedQueue_.empty()) {
			unicam_[Unicam::Embedded].dev()->queueBuffer(embeddedQueue_.front());
			embeddedQueue_.pop();
		}
	}
}

FrameBuffer *RPiCameraData::updateQueue(std::queue<FrameBuffer *> &q, uint64_t timestamp,
					V4L2VideoDevice *dev)
{
	while (!q.empty()) {
		FrameBuffer *b = q.front();
		if (b->metadata().timestamp < timestamp) {
			q.pop();
			dev->queueBuffer(b);
			LOG(RPI, Error) << "Dropping input frame!";
		} else if (b->metadata().timestamp == timestamp) {
			/* The calling function will pop the item from the queue. */
			return b;
		} else {
			break; /* Only higher timestamps from here. */
		}
	}

	return nullptr;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerRPi);

} /* namespace libcamera */
