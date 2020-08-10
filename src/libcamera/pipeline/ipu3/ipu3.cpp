/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipu3.cpp - Pipeline handler for Intel IPU3
 */

#include <algorithm>
#include <iomanip>
#include <memory>
#include <queue>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/formats.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/utils.h"
#include "libcamera/internal/v4l2_controls.h"

#include "cio2.h"
#include "imgu.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPU3)

static constexpr unsigned int IPU3_BUFFER_COUNT = 4;
static constexpr unsigned int IPU3_MAX_STREAMS = 3;
static const Size IMGU_OUTPUT_MIN_SIZE = { 2, 2 };
static const Size IMGU_OUTPUT_MAX_SIZE = { 4480, 34004 };
static constexpr unsigned int IMGU_OUTPUT_WIDTH_ALIGN = 64;
static constexpr unsigned int IMGU_OUTPUT_HEIGHT_ALIGN = 4;
static constexpr unsigned int IMGU_OUTPUT_WIDTH_MARGIN = 64;
static constexpr unsigned int IMGU_OUTPUT_HEIGHT_MARGIN = 32;

class IPU3CameraData : public CameraData
{
public:
	IPU3CameraData(PipelineHandler *pipe)
		: CameraData(pipe)
	{
	}

	void imguOutputBufferReady(FrameBuffer *buffer);
	void cio2BufferReady(FrameBuffer *buffer);

	CIO2Device cio2_;
	ImgUDevice *imgu_;

	Stream outStream_;
	Stream vfStream_;
	Stream rawStream_;
};

class IPU3CameraConfiguration : public CameraConfiguration
{
public:
	IPU3CameraConfiguration(IPU3CameraData *data);

	Status validate() override;

	const StreamConfiguration &cio2Format() const { return cio2Configuration_; };
	const ImgUDevice::PipeConfig imguConfig() const { return pipeConfig_; }

private:
	/*
	 * The IPU3CameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	const IPU3CameraData *data_;

	StreamConfiguration cio2Configuration_;
	ImgUDevice::PipeConfig pipeConfig_;
};

class PipelineHandlerIPU3 : public PipelineHandler
{
public:
	static constexpr unsigned int V4L2_CID_IPU3_PIPE_MODE = 0x009819c1;

	enum IPU3PipeModes {
		IPU3PipeModeVideo = 0,
		IPU3PipeModeStillCapture = 1,
	};

	PipelineHandlerIPU3(CameraManager *manager);

	CameraConfiguration *generateConfiguration(Camera *camera,
		const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	IPU3CameraData *cameraData(const Camera *camera)
	{
		return static_cast<IPU3CameraData *>(
			PipelineHandler::cameraData(camera));
	}

	int registerCameras();

	int allocateBuffers(Camera *camera);
	int freeBuffers(Camera *camera);

	ImgUDevice imgu0_;
	ImgUDevice imgu1_;
	MediaDevice *cio2MediaDev_;
	MediaDevice *imguMediaDev_;
};

IPU3CameraConfiguration::IPU3CameraConfiguration(IPU3CameraData *data)
	: CameraConfiguration()
{
	data_ = data;
}

CameraConfiguration::Status IPU3CameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > IPU3_MAX_STREAMS) {
		config_.resize(IPU3_MAX_STREAMS);
		status = Adjusted;
	}

	/*
	 * Validate the requested stream configuration and select the sensor
	 * format by collecting the maximum RAW stream width and height and
	 * picking the closest larger match, as the IPU3 can downscale only. If
	 * no resolution is requested for the RAW stream, use the one from the
	 * largest YUV stream, plus margins pixels for the IF and BDS to scale.
	 * If no resolution is requested for any stream, pick the largest one.
	 */
	unsigned int rawCount = 0;
	unsigned int yuvCount = 0;
	Size maxYuvSize;
	Size maxRawSize;

	for (const StreamConfiguration &cfg : config_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			rawCount++;
			maxRawSize.expandTo(cfg.size);
		} else {
			yuvCount++;
			maxYuvSize.expandTo(cfg.size);
		}
	}

	if (rawCount > 1 || yuvCount > 2) {
		LOG(IPU3, Debug) << "Camera configuration not supported";
		return Invalid;
	}

	if (maxRawSize.isNull())
		maxRawSize = maxYuvSize.alignedUpTo(IMGU_OUTPUT_WIDTH_MARGIN,
						    IMGU_OUTPUT_HEIGHT_MARGIN)
				       .boundedTo(data_->cio2_.sensor()->resolution());

	/*
	 * Generate raw configuration from CIO2.
	 *
	 * The output YUV streams will be limited in size to the maximum
	 * frame size requested for the RAW stream.
	 */
	cio2Configuration_ = data_->cio2_.generateConfiguration(maxRawSize);
	if (!cio2Configuration_.pixelFormat.isValid())
		return Invalid;

	LOG(IPU3, Debug) << "CIO2 configuration: " << cio2Configuration_.toString();

	ImgUDevice::Pipe pipe{};
	pipe.input = cio2Configuration_.size;

	/*
	 * Adjust the configurations if needed and assign streams while
	 * iterating them.
	 */
	bool mainOutputAvailable = true;
	for (unsigned int i = 0; i < config_.size(); ++i) {
		const PixelFormatInfo &info = PixelFormatInfo::info(config_[i].pixelFormat);
		const StreamConfiguration originalCfg = config_[i];
		StreamConfiguration *cfg = &config_[i];

		LOG(IPU3, Debug) << "Validating stream: " << config_[i].toString();

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			/* Initialize the RAW stream with the CIO2 configuration. */
			cfg->size = cio2Configuration_.size;
			cfg->pixelFormat = cio2Configuration_.pixelFormat;
			cfg->bufferCount = cio2Configuration_.bufferCount;
			cfg->stride = info.stride(cfg->size.width, 0, 64);
			cfg->frameSize = info.frameSize(cfg->size, 64);
			cfg->setStream(const_cast<Stream *>(&data_->rawStream_));

			LOG(IPU3, Debug) << "Assigned " << cfg->toString()
					 << " to the raw stream";
		} else {
			/* Assign and configure the main and viewfinder outputs. */

			/*
			 * Clamp the size to match the ImgU size limits and the
			 * margins from the CIO2 output frame size.
			 *
			 * The ImgU outputs needs to be strictly smaller than
			 * the CIO2 output frame and rounded down to 64 pixels
			 * in width and 32 pixels in height. This assumption
			 * comes from inspecting the pipe configuration script
			 * results and the available suggested configurations in
			 * the ChromeOS BSP .xml camera tuning files and shall
			 * be validated.
			 *
			 * \todo Clarify what are the hardware constraints
			 * that require this alignements, if any. It might
			 * depend on the BDS scaling factor of 1/32, as the main
			 * output has no YUV scaler as the viewfinder output has.
			 */
			unsigned int limit;
			limit = utils::alignDown(cio2Configuration_.size.width - 1,
						 IMGU_OUTPUT_WIDTH_MARGIN);
			cfg->size.width = utils::clamp(cfg->size.width,
						       IMGU_OUTPUT_MIN_SIZE.width,
						       limit);

			limit = utils::alignDown(cio2Configuration_.size.height - 1,
						 IMGU_OUTPUT_HEIGHT_MARGIN);
			cfg->size.height = utils::clamp(cfg->size.height,
							IMGU_OUTPUT_MIN_SIZE.height,
							limit);

			cfg->size.alignDownTo(IMGU_OUTPUT_WIDTH_ALIGN,
					      IMGU_OUTPUT_HEIGHT_ALIGN);

			cfg->pixelFormat = formats::NV12;
			cfg->bufferCount = IPU3_BUFFER_COUNT;
			cfg->stride = info.stride(cfg->size.width, 0, 1);
			cfg->frameSize = info.frameSize(cfg->size, 1);

			/*
			 * Use the main output stream in case only one stream is
			 * requested or if the current configuration is the one
			 * with the maximum YUV output size.
			 */
			if (mainOutputAvailable &&
			    (originalCfg.size == maxYuvSize || yuvCount == 1)) {
				cfg->setStream(const_cast<Stream *>(&data_->outStream_));
				mainOutputAvailable = false;

				pipe.main = cfg->size;
				if (yuvCount == 1)
					pipe.viewfinder = pipe.main;

				LOG(IPU3, Debug) << "Assigned " << cfg->toString()
						 << " to the main output";
			} else {
				cfg->setStream(const_cast<Stream *>(&data_->vfStream_));
				pipe.viewfinder = cfg->size;

				LOG(IPU3, Debug) << "Assigned " << cfg->toString()
						 << " to the viewfinder output";
			}
		}

		if (cfg->pixelFormat != originalCfg.pixelFormat ||
		    cfg->size != originalCfg.size) {
			LOG(IPU3, Debug)
				<< "Stream " << i << " configuration adjusted to "
				<< cfg->toString();
			status = Adjusted;
		}
	}

	/* Only compute the ImgU configuration if a YUV stream has been requested. */
	if (yuvCount) {
		pipeConfig_ = data_->imgu_->calculatePipeConfig(&pipe);
		if (pipeConfig_.isNull()) {
			LOG(IPU3, Error) << "Failed to calculate pipe configuration: "
					 << "unsupported resolutions.";
			return Invalid;
		}
	}

	return status;
}

PipelineHandlerIPU3::PipelineHandlerIPU3(CameraManager *manager)
	: PipelineHandler(manager), cio2MediaDev_(nullptr), imguMediaDev_(nullptr)
{
}

CameraConfiguration *PipelineHandlerIPU3::generateConfiguration(Camera *camera,
								const StreamRoles &roles)
{
	IPU3CameraData *data = cameraData(camera);
	IPU3CameraConfiguration *config = new IPU3CameraConfiguration(data);

	if (roles.empty())
		return config;

	Size sensorResolution = data->cio2_.sensor()->resolution();
	for (const StreamRole role : roles) {
		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		unsigned int bufferCount;
		PixelFormat pixelFormat;
		Size size;

		switch (role) {
		case StreamRole::StillCapture:
			/*
			 * Use as default full-frame configuration a value
			 * strictly smaller than the sensor resolution (limited
			 * to the ImgU  maximum output size) and aligned down to
			 * the required frame margin.
			 *
			 * \todo Clarify the alignment constraints as explained
			 * in validate()
			 */
			size = sensorResolution.boundedTo(IMGU_OUTPUT_MAX_SIZE);
			size.width = utils::alignDown(size.width - 1,
						      IMGU_OUTPUT_WIDTH_MARGIN);
			size.height = utils::alignDown(size.height - 1,
						       IMGU_OUTPUT_HEIGHT_MARGIN);
			pixelFormat = formats::NV12;
			bufferCount = IPU3_BUFFER_COUNT;
			streamFormats[pixelFormat] = { { IMGU_OUTPUT_MIN_SIZE, size } };

			break;

		case StreamRole::StillCaptureRaw: {
			StreamConfiguration cio2Config =
				data->cio2_.generateConfiguration(sensorResolution);
			pixelFormat = cio2Config.pixelFormat;
			size = cio2Config.size;
			bufferCount = cio2Config.bufferCount;

			for (const PixelFormat &format : data->cio2_.formats())
				streamFormats[format] = data->cio2_.sizes();

			break;
		}

		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording: {
			/*
			 * Default viewfinder and videorecording to 1280x720,
			 * capped to the maximum sensor resolution and aligned
			 * to the ImgU output constraints.
			 */
			size = sensorResolution.boundedTo({ 1280, 720 })
					       .alignedDownTo(IMGU_OUTPUT_WIDTH_ALIGN,
							      IMGU_OUTPUT_HEIGHT_ALIGN);
			pixelFormat = formats::NV12;
			bufferCount = IPU3_BUFFER_COUNT;
			streamFormats[pixelFormat] = { { IMGU_OUTPUT_MIN_SIZE, size } };

			break;
		}

		default:
			LOG(IPU3, Error)
				<< "Requested stream role not supported: " << role;
			delete config;
			return nullptr;
		}

		StreamFormats formats(streamFormats);
		StreamConfiguration cfg(formats);
		cfg.size = size;
		cfg.pixelFormat = pixelFormat;
		cfg.bufferCount = bufferCount;
		config->addConfiguration(cfg);
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerIPU3::configure(Camera *camera, CameraConfiguration *c)
{
	IPU3CameraConfiguration *config =
		static_cast<IPU3CameraConfiguration *>(c);
	IPU3CameraData *data = cameraData(camera);
	Stream *outStream = &data->outStream_;
	Stream *vfStream = &data->vfStream_;
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	V4L2DeviceFormat outputFormat;
	int ret;

	/*
	 * FIXME: enabled links in one ImgU pipe interfere with capture
	 * operations on the other one. This can be easily triggered by
	 * capturing from one camera and then trying to capture from the other
	 * one right after, without disabling media links on the first used
	 * pipe.
	 *
	 * The tricky part here is where to disable links on the ImgU instance
	 * which is currently not in use:
	 * 1) Link enable/disable cannot be done at start()/stop() time as video
	 * devices needs to be linked first before format can be configured on
	 * them.
	 * 2) As link enable has to be done at the least in configure(),
	 * before configuring formats, the only place where to disable links
	 * would be 'stop()', but the Camera class state machine allows
	 * start()<->stop() sequences without any configure() in between.
	 *
	 * As of now, disable all links in the ImgU media graph before
	 * configuring the device, to allow alternate the usage of the two
	 * ImgU pipes.
	 *
	 * As a consequence, a Camera using an ImgU shall be configured before
	 * any start()/stop() sequence. An application that wants to
	 * pre-configure all the camera and then start/stop them alternatively
	 * without going through any re-configuration (a sequence that is
	 * allowed by the Camera state machine) would now fail on the IPU3.
	 */
	ret = imguMediaDev_->disableLinks();
	if (ret)
		return ret;

	/*
	 * \todo: Enable links selectively based on the requested streams.
	 * As of now, enable all links unconditionally.
	 * \todo Don't configure the ImgU at all if we only have a single
	 * stream which is for raw capture, in which case no buffers will
	 * ever be queued to the ImgU.
	 */
	ret = data->imgu_->enableLinks(true);
	if (ret)
		return ret;

	/*
	 * Pass the requested stream size to the CIO2 unit and get back the
	 * adjusted format to be propagated to the ImgU output devices.
	 */
	const Size &sensorSize = config->cio2Format().size;
	V4L2DeviceFormat cio2Format = {};
	ret = cio2->configure(sensorSize, &cio2Format);
	if (ret)
		return ret;

	/*
	 * If the ImgU gets configured with proper IF, BDS and GDC sizes, it
	 * is then expected that frames are dequeued from its main output
	 * otherwise the system stalls.
	 *
	 * If no ImgU configuration has been computed, it means only a RAW
	 * stream has been requested: return here to skip the ImgU configuration
	 * part.
	 */
	ImgUDevice::PipeConfig imguConfig = config->imguConfig();
	if (imguConfig.isNull())
		return 0;

	ret = imgu->configure(imguConfig, &cio2Format);
	if (ret)
		return ret;

	/* Apply the format to the configured streams output devices. */
	bool outActive = false;
	bool vfActive = false;

	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == outStream) {
			ret = imgu->configureOutput(cfg, &outputFormat);
			if (ret)
				return ret;

			outActive = true;
		} else if (stream == vfStream) {
			ret = imgu->configureViewfinder(cfg, &outputFormat);
			if (ret)
				return ret;

			vfActive = true;
		}
	}

	/*
	 * As we need to set format also on the non-active streams, use
	 * the configuration of the active one for that purpose (there should
	 * be at least one active stream in the configuration request).
	 */
	if (!outActive) {
		ret = imgu->configureOutput(config->at(0), &outputFormat);
		if (ret)
			return ret;
	}

	if (!vfActive) {
		ret = imgu->configureViewfinder(config->at(0), &outputFormat);
		if (ret)
			return ret;
	}

	/*
	 * Apply the largest available format to the stat node.
	 * \todo Revise this when we'll actually use the stat node.
	 */
	StreamConfiguration statCfg = {};
	statCfg.size = cio2Format.size;

	ret = imgu->configureStat(statCfg, &outputFormat);
	if (ret)
		return ret;

	/* Apply the "pipe_mode" control to the ImgU subdevice. */
	ControlList ctrls(imgu->imgu_->controls());
	ctrls.set(V4L2_CID_IPU3_PIPE_MODE,
		  static_cast<int32_t>(vfActive ? IPU3PipeModeVideo :
				       IPU3PipeModeStillCapture));
	ret = imgu->imgu_->setControls(&ctrls);
	if (ret) {
		LOG(IPU3, Error) << "Unable to set pipe_mode control";
		return ret;
	}

	return 0;
}

int PipelineHandlerIPU3::exportFrameBuffers(Camera *camera, Stream *stream,
					    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	IPU3CameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->outStream_)
		return data->imgu_->output_->exportBuffers(count, buffers);
	else if (stream == &data->vfStream_)
		return data->imgu_->viewfinder_->exportBuffers(count, buffers);
	else if (stream == &data->rawStream_)
		return data->cio2_.exportBuffers(count, buffers);

	return -EINVAL;
}

/**
 * \todo Clarify if 'viewfinder' and 'stat' nodes have to be set up and
 * started even if not in use. As of now, if not properly configured and
 * enabled, the ImgU processing pipeline stalls.
 *
 * In order to be able to start the 'viewfinder' and 'stat' nodes, we need
 * memory to be reserved.
 */
int PipelineHandlerIPU3::allocateBuffers(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	ImgUDevice *imgu = data->imgu_;
	unsigned int bufferCount;
	int ret;

	bufferCount = std::max({
		data->outStream_.configuration().bufferCount,
		data->vfStream_.configuration().bufferCount,
		data->rawStream_.configuration().bufferCount,
	});

	ret = imgu->allocateBuffers(bufferCount);
	if (ret < 0)
		return ret;

	return 0;
}

int PipelineHandlerIPU3::freeBuffers(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);

	data->imgu_->freeBuffers();

	return 0;
}

int PipelineHandlerIPU3::start(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers(camera);
	if (ret)
		return ret;

	/*
	 * Start the ImgU video devices, buffers will be queued to the
	 * ImgU output and viewfinder when requests will be queued.
	 */
	ret = cio2->start();
	if (ret)
		goto error;

	ret = imgu->start();
	if (ret) {
		imgu->stop();
		cio2->stop();
		goto error;
	}

	return 0;

error:
	freeBuffers(camera);
	LOG(IPU3, Error) << "Failed to start camera " << camera->id();

	return ret;
}

void PipelineHandlerIPU3::stop(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	int ret = 0;

	ret |= data->imgu_->stop();
	ret |= data->cio2_.stop();
	if (ret)
		LOG(IPU3, Warning) << "Failed to stop camera " << camera->id();

	freeBuffers(camera);
}

int PipelineHandlerIPU3::queueRequestDevice(Camera *camera, Request *request)
{
	IPU3CameraData *data = cameraData(camera);
	int error = 0;

	/*
	 * Queue a buffer on the CIO2, using the raw stream buffer provided in
	 * the request, if any, or a CIO2 internal buffer otherwise.
	 */
	FrameBuffer *rawBuffer = request->findBuffer(&data->rawStream_);
	error = data->cio2_.queueBuffer(request, rawBuffer);
	if (error)
		return error;

	/* Queue all buffers from the request aimed for the ImgU. */
	for (auto it : request->buffers()) {
		const Stream *stream = it.first;
		FrameBuffer *buffer = it.second;
		int ret;

		if (stream == &data->outStream_)
			ret = data->imgu_->output_->queueBuffer(buffer);
		else if (stream == &data->vfStream_)
			ret = data->imgu_->viewfinder_->queueBuffer(buffer);
		else
			continue;

		if (ret < 0)
			error = ret;
	}

	return error;
}

bool PipelineHandlerIPU3::match(DeviceEnumerator *enumerator)
{
	int ret;

	DeviceMatch cio2_dm("ipu3-cio2");
	cio2_dm.add("ipu3-csi2 0");
	cio2_dm.add("ipu3-cio2 0");
	cio2_dm.add("ipu3-csi2 1");
	cio2_dm.add("ipu3-cio2 1");
	cio2_dm.add("ipu3-csi2 2");
	cio2_dm.add("ipu3-cio2 2");
	cio2_dm.add("ipu3-csi2 3");
	cio2_dm.add("ipu3-cio2 3");

	DeviceMatch imgu_dm("ipu3-imgu");
	imgu_dm.add("ipu3-imgu 0");
	imgu_dm.add("ipu3-imgu 0 input");
	imgu_dm.add("ipu3-imgu 0 parameters");
	imgu_dm.add("ipu3-imgu 0 output");
	imgu_dm.add("ipu3-imgu 0 viewfinder");
	imgu_dm.add("ipu3-imgu 0 3a stat");
	imgu_dm.add("ipu3-imgu 1");
	imgu_dm.add("ipu3-imgu 1 input");
	imgu_dm.add("ipu3-imgu 1 parameters");
	imgu_dm.add("ipu3-imgu 1 output");
	imgu_dm.add("ipu3-imgu 1 viewfinder");
	imgu_dm.add("ipu3-imgu 1 3a stat");

	cio2MediaDev_ = acquireMediaDevice(enumerator, cio2_dm);
	if (!cio2MediaDev_)
		return false;

	imguMediaDev_ = acquireMediaDevice(enumerator, imgu_dm);
	if (!imguMediaDev_)
		return false;

	/*
	 * Disable all links that are enabled by default on CIO2, as camera
	 * creation enables all valid links it finds.
	 */
	if (cio2MediaDev_->disableLinks())
		return false;

	ret = imguMediaDev_->disableLinks();
	if (ret)
		return ret;

	ret = registerCameras();

	return ret == 0;
}

/**
 * \brief Initialise ImgU and CIO2 devices associated with cameras
 *
 * Initialise the two ImgU instances and create cameras with an associated
 * CIO2 device instance.
 *
 * \return 0 on success or a negative error code for error or if no camera
 * has been created
 * \retval -ENODEV no camera has been created
 */
int PipelineHandlerIPU3::registerCameras()
{
	int ret;

	ret = imgu0_.init(imguMediaDev_, 0);
	if (ret)
		return ret;

	ret = imgu1_.init(imguMediaDev_, 1);
	if (ret)
		return ret;

	/*
	 * For each CSI-2 receiver on the IPU3, create a Camera if an
	 * image sensor is connected to it and the sensor can produce images
	 * in a compatible format.
	 */
	unsigned int numCameras = 0;
	for (unsigned int id = 0; id < 4 && numCameras < 2; ++id) {
		std::unique_ptr<IPU3CameraData> data =
			std::make_unique<IPU3CameraData>(this);
		std::set<Stream *> streams = {
			&data->outStream_,
			&data->vfStream_,
			&data->rawStream_,
		};
		CIO2Device *cio2 = &data->cio2_;

		ret = cio2->init(cio2MediaDev_, id);
		if (ret)
			continue;

		/* Initialize the camera properties. */
		data->properties_ = cio2->sensor()->properties();

		/**
		 * \todo Dynamically assign ImgU and output devices to each
		 * stream and camera; as of now, limit support to two cameras
		 * only, and assign imgu0 to the first one and imgu1 to the
		 * second.
		 */
		data->imgu_ = numCameras ? &imgu1_ : &imgu0_;

		/*
		 * Connect video devices' 'bufferReady' signals to their
		 * slot to implement the image processing pipeline.
		 *
		 * Frames produced by the CIO2 unit are passed to the
		 * associated ImgU input where they get processed and
		 * returned through the ImgU main and secondary outputs.
		 */
		data->cio2_.bufferReady().connect(data.get(),
					&IPU3CameraData::cio2BufferReady);
		data->imgu_->input_->bufferReady.connect(&data->cio2_,
					&CIO2Device::tryReturnBuffer);
		data->imgu_->output_->bufferReady.connect(data.get(),
					&IPU3CameraData::imguOutputBufferReady);
		data->imgu_->viewfinder_->bufferReady.connect(data.get(),
					&IPU3CameraData::imguOutputBufferReady);

		/* Create and register the Camera instance. */
		std::string cameraId = cio2->sensor()->id();
		std::shared_ptr<Camera> camera =
			Camera::create(this, cameraId, streams);

		registerCamera(std::move(camera), std::move(data));

		LOG(IPU3, Info)
			<< "Registered Camera[" << numCameras << "] \""
			<< cameraId << "\""
			<< " connected to CSI-2 receiver " << id;

		numCameras++;
	}

	return numCameras ? 0 : -ENODEV;
}

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

/**
 * \brief Handle buffers completion at the ImgU output
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the ImgU output are directed to the application.
 */
void IPU3CameraData::imguOutputBufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	if (!pipe_->completeBuffer(camera_, request, buffer))
		/* Request not completed yet, return here. */
		return;

	/* Mark the request as complete. */
	pipe_->completeRequest(camera_, request);
}

/**
 * \brief Handle buffers completion at the CIO2 output
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the CIO2 are immediately queued to the ImgU unit
 * for further processing.
 */
void IPU3CameraData::cio2BufferReady(FrameBuffer *buffer)
{
	/* \todo Handle buffer failures when state is set to BufferError. */
	if (buffer->metadata().status == FrameMetadata::FrameCancelled)
		return;

	Request *request = buffer->request();

	/*
	 * If the request contains a buffer for the RAW stream only, complete it
	 * now as there's no need for ImgU processing.
	 */
	if (request->findBuffer(&rawStream_)) {
		bool isComplete = pipe_->completeBuffer(camera_, request, buffer);
		if (isComplete) {
			pipe_->completeRequest(camera_, request);
			return;
		}
	}

	imgu_->input_->queueBuffer(buffer);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU3);

} /* namespace libcamera */
