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
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/ipa/ipu3_ipa_interface.h>
#include <libcamera/ipa/ipu3_ipa_proxy.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/utils.h"

#include "cio2.h"
#include "frames.h"
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
static constexpr Size IPU3ViewfinderSize(1280, 720);

static const ControlInfoMap::Map IPU3Controls = {
	{ &controls::draft::PipelineDepth, ControlInfo(2, 3) },
};

class IPU3CameraData : public CameraData
{
public:
	IPU3CameraData(PipelineHandler *pipe)
		: CameraData(pipe), exposureTime_(0), supportsFlips_(false)
	{
	}

	int loadIPA();

	void imguOutputBufferReady(FrameBuffer *buffer);
	void cio2BufferReady(FrameBuffer *buffer);
	void paramBufferReady(FrameBuffer *buffer);
	void statBufferReady(FrameBuffer *buffer);
	void queuePendingRequests();
	void cancelPendingRequests();

	CIO2Device cio2_;
	ImgUDevice *imgu_;

	Stream outStream_;
	Stream vfStream_;
	Stream rawStream_;

	uint32_t exposureTime_;
	Rectangle cropRegion_;
	bool supportsFlips_;
	Transform rotationTransform_;

	std::unique_ptr<DelayedControls> delayedCtrls_;
	IPU3Frames frameInfos_;

	std::unique_ptr<ipa::ipu3::IPAProxyIPU3> ipa_;

	std::queue<Request *> pendingRequests_;

private:
	void queueFrameAction(unsigned int id,
			      const ipa::ipu3::IPU3Action &action);
};

class IPU3CameraConfiguration : public CameraConfiguration
{
public:
	IPU3CameraConfiguration(IPU3CameraData *data);

	Status validate() override;

	const StreamConfiguration &cio2Format() const { return cio2Configuration_; }
	const ImgUDevice::PipeConfig imguConfig() const { return pipeConfig_; }

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;

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

	int start(Camera *camera, const ControlList *controls) override;
	void stop(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	IPU3CameraData *cameraData(const Camera *camera)
	{
		return static_cast<IPU3CameraData *>(
			PipelineHandler::cameraData(camera));
	}

	int initControls(IPU3CameraData *data);
	int registerCameras();

	int allocateBuffers(Camera *camera);
	int freeBuffers(Camera *camera);

	ImgUDevice imgu0_;
	ImgUDevice imgu1_;
	MediaDevice *cio2MediaDev_;
	MediaDevice *imguMediaDev_;

	std::vector<IPABuffer> ipaBuffers_;
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

	Transform combined = transform * data_->rotationTransform_;

	/*
	 * We combine the platform and user transform, but must "adjust away"
	 * any combined result that includes a transposition, as we can't do
	 * those. In this case, flipping only the transpose bit is helpful to
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
		 * rise to this is the inverse of the rotation. (Recall that
		 * combined = transform * rotationTransform.)
		 */
		transform = -data_->rotationTransform_;
		combined = Transform::Identity;
		status = Adjusted;
	}

	/*
	 * Store the final combined transform that configure() will need to
	 * apply to the sensor to save us working it out again.
	 */
	combinedTransform_ = combined;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > IPU3_MAX_STREAMS) {
		config_.resize(IPU3_MAX_STREAMS);
		status = Adjusted;
	}

	/* Validate the requested stream configuration */
	unsigned int rawCount = 0;
	unsigned int yuvCount = 0;
	Size maxYuvSize;
	Size rawSize;

	for (const StreamConfiguration &cfg : config_) {
		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);

		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW) {
			rawCount++;
			rawSize = cfg.size;
		} else {
			yuvCount++;
			maxYuvSize.expandTo(cfg.size);
		}
	}

	if (rawCount > 1 || yuvCount > 2) {
		LOG(IPU3, Debug) << "Camera configuration not supported";
		return Invalid;
	}

	/*
	 * Generate raw configuration from CIO2.
	 *
	 * \todo The image sensor frame size should be selected to optimize
	 * operations based on the sizes of the requested streams. However such
	 * a selection makes the pipeline configuration procedure fail for small
	 * resolutions (for example: 640x480 with OV5670) and causes the capture
	 * operations to stall for some stream size combinations (see the
	 * commit message of the patch that introduced this comment for more
	 * failure examples).
	 *
	 * Until the sensor frame size calculation criteria are clarified, when
	 * capturing from ImgU always use the largest possible size which
	 * guarantees better results at the expense of the frame rate and CSI-2
	 * bus bandwidth. When only a raw stream is requested use the requested
	 * size instead, as the ImgU is not involved.
	 */
	if (!yuvCount)
		cio2Configuration_ = data_->cio2_.generateConfiguration(rawSize);
	else
		cio2Configuration_ = data_->cio2_.generateConfiguration({});
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
			cfg->size.width = std::clamp(cfg->size.width,
						     IMGU_OUTPUT_MIN_SIZE.width,
						     limit);

			limit = utils::alignDown(cio2Configuration_.size.height - 1,
						 IMGU_OUTPUT_HEIGHT_MARGIN);
			cfg->size.height = std::clamp(cfg->size.height,
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

		case StreamRole::Raw: {
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
			size = sensorResolution.boundedTo(IPU3ViewfinderSize)
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
	V4L2DeviceFormat cio2Format;
	ret = cio2->configure(sensorSize, &cio2Format);
	if (ret)
		return ret;

	IPACameraSensorInfo sensorInfo;
	cio2->sensor()->sensorInfo(&sensorInfo);
	data->cropRegion_ = sensorInfo.analogCrop;

	/*
	 * Configure the H/V flip controls based on the combination of
	 * the sensor and user transform.
	 */
	if (data->supportsFlips_) {
		ControlList sensorCtrls(cio2->sensor()->controls());
		sensorCtrls.set(V4L2_CID_HFLIP,
				static_cast<int32_t>(!!(config->combinedTransform_
							& Transform::HFlip)));
		sensorCtrls.set(V4L2_CID_VFLIP,
				static_cast<int32_t>(!!(config->combinedTransform_
						        & Transform::VFlip)));

		ret = cio2->sensor()->setControls(&sensorCtrls);
		if (ret)
			return ret;
	}

	/*
	 * If the ImgU gets configured, its driver seems to expect that
	 * buffers will be queued to its outputs, as otherwise the next
	 * capture session that uses the ImgU fails when queueing
	 * buffers to its input.
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
	StreamConfiguration *mainCfg = nullptr;
	StreamConfiguration *vfCfg = nullptr;

	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = (*config)[i];
		Stream *stream = cfg.stream();

		if (stream == outStream) {
			mainCfg = &cfg;
			ret = imgu->configureOutput(cfg, &outputFormat);
			if (ret)
				return ret;
		} else if (stream == vfStream) {
			vfCfg = &cfg;
			ret = imgu->configureViewfinder(cfg, &outputFormat);
			if (ret)
				return ret;
		}
	}

	/*
	 * As we need to set format also on the non-active streams, use
	 * the configuration of the active one for that purpose (there should
	 * be at least one active stream in the configuration request).
	 */
	if (!vfCfg) {
		ret = imgu->configureViewfinder(*mainCfg, &outputFormat);
		if (ret)
			return ret;
	}

	/* Apply the "pipe_mode" control to the ImgU subdevice. */
	ControlList ctrls(imgu->imgu_->controls());
	/*
	 * Set the ImgU pipe mode to 'Video' unconditionally to have statistics
	 * generated.
	 *
	 * \todo Figure out what the 'Still Capture' mode is meant for, and use
	 * it accordingly.
	 */
	ctrls.set(V4L2_CID_IPU3_PIPE_MODE,
		  static_cast<int32_t>(IPU3PipeModeVideo));
	ret = imgu->imgu_->setControls(&ctrls);
	if (ret) {
		LOG(IPU3, Error) << "Unable to set pipe_mode control";
		return ret;
	}

	ipa::ipu3::IPAConfigInfo configInfo;
	configInfo.entityControls.emplace(0, data->cio2_.sensor()->controls());
	configInfo.sensorInfo = sensorInfo;
	configInfo.bdsOutputSize = config->imguConfig().bds;
	configInfo.iif = config->imguConfig().iif;

	data->ipa_->configure(configInfo);

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

	/* Map buffers to the IPA. */
	unsigned int ipaBufferId = 1;

	for (const std::unique_ptr<FrameBuffer> &buffer : imgu->paramBuffers_) {
		buffer->setCookie(ipaBufferId++);
		ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	for (const std::unique_ptr<FrameBuffer> &buffer : imgu->statBuffers_) {
		buffer->setCookie(ipaBufferId++);
		ipaBuffers_.emplace_back(buffer->cookie(), buffer->planes());
	}

	data->ipa_->mapBuffers(ipaBuffers_);

	data->frameInfos_.init(imgu->paramBuffers_, imgu->statBuffers_);
	data->frameInfos_.bufferAvailable.connect(
		data, &IPU3CameraData::queuePendingRequests);

	return 0;
}

int PipelineHandlerIPU3::freeBuffers(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);

	data->frameInfos_.clear();

	std::vector<unsigned int> ids;
	for (IPABuffer &ipabuf : ipaBuffers_)
		ids.push_back(ipabuf.id);

	data->ipa_->unmapBuffers(ids);
	ipaBuffers_.clear();

	data->imgu_->freeBuffers();

	return 0;
}

int PipelineHandlerIPU3::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	IPU3CameraData *data = cameraData(camera);
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	int ret;

	/* Allocate buffers for internal pipeline usage. */
	ret = allocateBuffers(camera);
	if (ret)
		return ret;

	ret = data->ipa_->start();
	if (ret)
		goto error;

	/*
	 * Start the ImgU video devices, buffers will be queued to the
	 * ImgU output and viewfinder when requests will be queued.
	 */
	ret = cio2->start();
	if (ret)
		goto error;

	ret = imgu->start();
	if (ret)
		goto error;

	return 0;

error:
	imgu->stop();
	cio2->stop();
	data->ipa_->stop();
	freeBuffers(camera);
	LOG(IPU3, Error) << "Failed to start camera " << camera->id();

	return ret;
}

void PipelineHandlerIPU3::stop(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	int ret = 0;

	data->cancelPendingRequests();

	data->ipa_->stop();

	ret |= data->imgu_->stop();
	ret |= data->cio2_.stop();
	if (ret)
		LOG(IPU3, Warning) << "Failed to stop camera " << camera->id();

	freeBuffers(camera);
}

void IPU3CameraData::cancelPendingRequests()
{
	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		for (auto it : request->buffers()) {
			FrameBuffer *buffer = it.second;
			buffer->cancel();
			pipe_->completeBuffer(request, buffer);
		}

		pipe_->completeRequest(request);
		pendingRequests_.pop();
	}
}

void IPU3CameraData::queuePendingRequests()
{
	while (!pendingRequests_.empty()) {
		Request *request = pendingRequests_.front();

		IPU3Frames::Info *info = frameInfos_.create(request);
		if (!info)
			break;

		/*
		 * Queue a buffer on the CIO2, using the raw stream buffer
		 * provided in the request, if any, or a CIO2 internal buffer
		 * otherwise.
		 */
		FrameBuffer *reqRawBuffer = request->findBuffer(&rawStream_);
		FrameBuffer *rawBuffer = cio2_.queueBuffer(request, reqRawBuffer);
		/*
		 * \todo If queueBuffer fails in queuing a buffer to the device,
		 * report the request as error by cancelling the request and
		 * calling PipelineHandler::completeRequest().
		 */
		if (!rawBuffer) {
			frameInfos_.remove(info);
			break;
		}

		info->rawBuffer = rawBuffer;

		ipa::ipu3::IPU3Event ev;
		ev.op = ipa::ipu3::EventProcessControls;
		ev.frame = info->id;
		ev.controls = request->controls();
		ipa_->processEvent(ev);

		pendingRequests_.pop();
	}
}

int PipelineHandlerIPU3::queueRequestDevice(Camera *camera, Request *request)
{
	IPU3CameraData *data = cameraData(camera);

	data->pendingRequests_.push(request);
	data->queuePendingRequests();

	return 0;
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
 * \brief Initialize the camera controls
 * \param[in] data The camera data
 *
 * Initialize the camera controls as the union of the static pipeline handler
 * controls (IPU3Controls) and controls created dynamically from the sensor
 * capabilities.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineHandlerIPU3::initControls(IPU3CameraData *data)
{
	/*
	 * \todo The controls initialized here depend on sensor configuration
	 * and their limits should be updated once the configuration gets
	 * changed.
	 *
	 * Initialize the sensor using its resolution and compute the control
	 * limits.
	 */
	CameraSensor *sensor = data->cio2_.sensor();
	V4L2SubdeviceFormat sensorFormat = {};
	sensorFormat.size = sensor->resolution();
	int ret = sensor->setFormat(&sensorFormat);
	if (ret)
		return ret;

	IPACameraSensorInfo sensorInfo{};
	ret = sensor->sensorInfo(&sensorInfo);
	if (ret)
		return ret;

	ControlInfoMap::Map controls = IPU3Controls;
	const ControlInfoMap &sensorControls = sensor->controls();

	/*
	 * Compute exposure time limits.
	 *
	 * Initialize the control using the line length and pixel rate of the
	 * current configuration converted to microseconds. Use the
	 * V4L2_CID_EXPOSURE control to get exposure min, max and default and
	 * convert it from lines to microseconds.
	 */
	double lineDuration = sensorInfo.lineLength
			    / (sensorInfo.pixelRate / 1e6);
	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>() * lineDuration;
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>() * lineDuration;
	int32_t defExposure = v4l2Exposure.def().get<int32_t>() * lineDuration;

	/*
	 * \todo Report the actual exposure time, use the default for the
	 * moment.
	 */
	data->exposureTime_ = defExposure;

	controls[&controls::ExposureTime] = ControlInfo(minExposure, maxExposure,
							defExposure);

	/*
	 * Compute the frame duration limits.
	 *
	 * The frame length is computed assuming a fixed line length combined
	 * with the vertical frame sizes.
	 */
	const ControlInfo &v4l2HBlank = sensorControls.find(V4L2_CID_HBLANK)->second;
	uint32_t hblank = v4l2HBlank.def().get<int32_t>();
	uint32_t lineLength = sensorInfo.outputSize.width + hblank;

	const ControlInfo &v4l2VBlank = sensorControls.find(V4L2_CID_VBLANK)->second;
	std::array<uint32_t, 3> frameHeights{
		v4l2VBlank.min().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.max().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.def().get<int32_t>() + sensorInfo.outputSize.height,
	};

	std::array<int64_t, 3> frameDurations;
	for (unsigned int i = 0; i < frameHeights.size(); ++i) {
		uint64_t frameSize = lineLength * frameHeights[i];
		frameDurations[i] = frameSize / (sensorInfo.pixelRate / 1000000U);
	}

	controls[&controls::FrameDurations] = ControlInfo(frameDurations[0],
							  frameDurations[1],
							  frameDurations[2]);

	/*
	 * Compute the scaler crop limits.
	 *
	 * Initialize the control use the 'Viewfinder' configuration (1280x720)
	 * as the pipeline output resolution and the full sensor size as input
	 * frame (see the todo note in the validate() function about the usage
	 * of the sensor's full frame as ImgU input).
	 */

	/*
	 * The maximum scaler crop rectangle is the analogue crop used to
	 * produce the maximum frame size.
	 */
	const Rectangle &analogueCrop = sensorInfo.analogCrop;
	Rectangle maxCrop = analogueCrop;

	/*
	 * As the ImgU cannot up-scale, the minimum selection rectangle has to
	 * be as large as the pipeline output size. Use the default viewfinder
	 * configuration as the desired output size and calculate the minimum
	 * rectangle required to satisfy the ImgU processing margins, unless the
	 * sensor resolution is smaller.
	 *
	 * \todo This implementation is based on the same assumptions about the
	 * ImgU pipeline configuration described in then viewfinder and main
	 * output sizes calculation in the validate() function.
	 */

	/* The strictly smaller size than the sensor resolution, aligned to margins. */
	Size minSize = Size(sensor->resolution().width - 1,
			    sensor->resolution().height - 1)
		       .alignedDownTo(IMGU_OUTPUT_WIDTH_MARGIN,
				      IMGU_OUTPUT_HEIGHT_MARGIN);

	/*
	 * Either the smallest margin-aligned size larger than the viewfinder
	 * size or the adjusted sensor resolution.
	 */
	minSize = Size(IPU3ViewfinderSize.width + 1,
		       IPU3ViewfinderSize.height + 1)
		  .alignedUpTo(IMGU_OUTPUT_WIDTH_MARGIN,
			       IMGU_OUTPUT_HEIGHT_MARGIN)
		  .boundedTo(minSize);

	/*
	 * Re-scale in the sensor's native coordinates. Report (0,0) as
	 * top-left corner as we allow application to freely pan the crop area.
	 */
	Rectangle minCrop = Rectangle(minSize).scaledBy(analogueCrop.size(),
					       sensorInfo.outputSize);

	controls[&controls::ScalerCrop] = ControlInfo(minCrop, maxCrop, maxCrop);

	data->controlInfo_ = std::move(controls);

	return 0;
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

		ret = data->loadIPA();
		if (ret)
			continue;

		/* Initialize the camera properties. */
		data->properties_ = cio2->sensor()->properties();

		ret = initControls(data.get());
		if (ret)
			continue;

		/*
		 * \todo Read delay values from the sensor itself or from a
		 * a sensor database. For now use generic values taken from
		 * the Raspberry Pi and listed as 'generic values'.
		 */
		std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
			{ V4L2_CID_ANALOGUE_GAIN, { 1, false } },
			{ V4L2_CID_EXPOSURE, { 2, false } },
		};

		data->delayedCtrls_ =
			std::make_unique<DelayedControls>(cio2->sensor()->device(),
							  params);
		data->cio2_.frameStart().connect(data->delayedCtrls_.get(),
						 &DelayedControls::applyControls);

		/* Convert the sensor rotation to a transformation */
		int32_t rotation = 0;
		if (data->properties_.contains(properties::Rotation))
			rotation = data->properties_.get(properties::Rotation);
		else
			LOG(IPU3, Warning) << "Rotation control not exposed by "
					   << cio2->sensor()->id()
					   << ". Assume rotation 0";

		bool success;
		data->rotationTransform_ = transformFromRotation(rotation, &success);
		if (!success)
			LOG(IPU3, Warning) << "Invalid rotation of " << rotation
					   << " degrees: ignoring";

		ControlList ctrls = cio2->sensor()->getControls({ V4L2_CID_HFLIP });
		if (!ctrls.empty())
			/* We assume the sensor supports VFLIP too. */
			data->supportsFlips_ = true;

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
		data->cio2_.bufferAvailable.connect(
			data.get(), &IPU3CameraData::queuePendingRequests);
		data->imgu_->input_->bufferReady.connect(&data->cio2_,
					&CIO2Device::tryReturnBuffer);
		data->imgu_->output_->bufferReady.connect(data.get(),
					&IPU3CameraData::imguOutputBufferReady);
		data->imgu_->viewfinder_->bufferReady.connect(data.get(),
					&IPU3CameraData::imguOutputBufferReady);
		data->imgu_->param_->bufferReady.connect(data.get(),
					&IPU3CameraData::paramBufferReady);
		data->imgu_->stat_->bufferReady.connect(data.get(),
					&IPU3CameraData::statBufferReady);

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

int IPU3CameraData::loadIPA()
{
	ipa_ = IPAManager::createIPA<ipa::ipu3::IPAProxyIPU3>(pipe_, 1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->queueFrameAction.connect(this, &IPU3CameraData::queueFrameAction);

	CameraSensor *sensor = cio2_.sensor();
	int ret = ipa_->init(IPASettings{ "", sensor->model() });
	if (ret) {
		LOG(IPU3, Error) << "Failed to initialise the IPU3 IPA";
		return ret;
	}

	return 0;
}

void IPU3CameraData::queueFrameAction(unsigned int id,
				      const ipa::ipu3::IPU3Action &action)
{
	switch (action.op) {
	case ipa::ipu3::ActionSetSensorControls: {
		const ControlList &controls = action.controls;
		delayedCtrls_->push(controls);
		break;
	}
	case ipa::ipu3::ActionParamFilled: {
		IPU3Frames::Info *info = frameInfos_.find(id);
		if (!info)
			break;

		/* Queue all buffers from the request aimed for the ImgU. */
		for (auto it : info->request->buffers()) {
			const Stream *stream = it.first;
			FrameBuffer *outbuffer = it.second;

			if (stream == &outStream_)
				imgu_->output_->queueBuffer(outbuffer);
			else if (stream == &vfStream_)
				imgu_->viewfinder_->queueBuffer(outbuffer);
		}

		imgu_->param_->queueBuffer(info->paramBuffer);
		imgu_->stat_->queueBuffer(info->statBuffer);
		imgu_->input_->queueBuffer(info->rawBuffer);

		break;
	}
	case ipa::ipu3::ActionMetadataReady: {
		IPU3Frames::Info *info = frameInfos_.find(id);
		if (!info)
			break;

		Request *request = info->request;
		request->metadata().merge(action.controls);

		info->metadataProcessed = true;
		if (frameInfos_.tryComplete(info))
			pipe_->completeRequest(request);

		break;
	}
	default:
		LOG(IPU3, Error) << "Unknown action " << action.op;
		break;
	}
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
	IPU3Frames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	Request *request = info->request;

	pipe_->completeBuffer(request, buffer);

	request->metadata().set(controls::draft::PipelineDepth, 3);
	/* \todo Move the ExposureTime control to the IPA. */
	request->metadata().set(controls::ExposureTime, exposureTime_);
	/* \todo Actually apply the scaler crop region to the ImgU. */
	if (request->controls().contains(controls::ScalerCrop))
		cropRegion_ = request->controls().get(controls::ScalerCrop);
	request->metadata().set(controls::ScalerCrop, cropRegion_);

	if (frameInfos_.tryComplete(info))
		pipe_->completeRequest(request);
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
	IPU3Frames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	Request *request = info->request;

	/*
	 * Record the sensor's timestamp in the request metadata.
	 *
	 * \todo The sensor timestamp should be better estimated by connecting
	 * to the V4L2Device::frameStart signal.
	 */
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	/* If the buffer is cancelled force a complete of the whole request. */
	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		for (auto it : request->buffers()) {
			FrameBuffer *b = it.second;
			b->cancel();
			pipe_->completeBuffer(request, b);
		}

		frameInfos_.remove(info);
		pipe_->completeRequest(request);
		return;
	}

	if (request->findBuffer(&rawStream_))
		pipe_->completeBuffer(request, buffer);

	ipa::ipu3::IPU3Event ev;
	ev.op = ipa::ipu3::EventFillParams;
	ev.frame = info->id;
	ev.bufferId = info->paramBuffer->cookie();
	ipa_->processEvent(ev);
}

void IPU3CameraData::paramBufferReady(FrameBuffer *buffer)
{
	IPU3Frames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	info->paramDequeued = true;

	/*
	 * tryComplete() will delete info if it completes the IPU3Frame.
	 * In that event, we must have obtained the Request before hand.
	 *
	 * \todo Improve the FrameInfo API to avoid this type of issue
	 */
	Request *request = info->request;

	if (frameInfos_.tryComplete(info))
		pipe_->completeRequest(request);
}

void IPU3CameraData::statBufferReady(FrameBuffer *buffer)
{
	IPU3Frames::Info *info = frameInfos_.find(buffer);
	if (!info)
		return;

	Request *request = info->request;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		info->metadataProcessed = true;

		/*
		 * tryComplete() will delete info if it completes the IPU3Frame.
		 * In that event, we must have obtained the Request before hand.
		 */
		if (frameInfos_.tryComplete(info))
			pipe_->completeRequest(request);

		return;
	}

	ipa::ipu3::IPU3Event ev;
	ev.op = ipa::ipu3::EventStatReady;
	ev.frame = info->id;
	ev.bufferId = info->statBuffer->cookie();
	ev.frameTimestamp = request->metadata().get(controls::SensorTimestamp);
	ipa_->processEvent(ev);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU3)

} /* namespace libcamera */
