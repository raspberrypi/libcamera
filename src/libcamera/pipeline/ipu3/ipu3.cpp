/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipu3.cpp - Pipeline handler for Intel IPU3
 */

#include <algorithm>
#include <iomanip>
#include <memory>
#include <vector>

#include <linux/media-bus-format.h>

#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "camera_sensor.h"
#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"
#include "pipeline_handler.h"
#include "utils.h"
#include "v4l2_device.h"
#include "v4l2_subdevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPU3)

class ImgUDevice
{
public:
	static constexpr unsigned int PAD_INPUT = 0;
	static constexpr unsigned int PAD_OUTPUT = 2;
	static constexpr unsigned int PAD_VF = 3;
	static constexpr unsigned int PAD_STAT = 4;

	/* ImgU output descriptor: group data specific to an ImgU output. */
	struct ImgUOutput {
		V4L2Device *dev;
		unsigned int pad;
		std::string name;
		BufferPool *pool;
	};

	ImgUDevice()
		: imgu_(nullptr), input_(nullptr)
	{
		output_.dev = nullptr;
		viewfinder_.dev = nullptr;
		stat_.dev = nullptr;
	}

	~ImgUDevice()
	{
		delete imgu_;
		delete input_;
		delete output_.dev;
		delete viewfinder_.dev;
		delete stat_.dev;
	}

	int init(MediaDevice *media, unsigned int index);
	int configureInput(const Size &size,
			   V4L2DeviceFormat *inputFormat);
	int configureOutput(ImgUOutput *output,
			    const StreamConfiguration &cfg);

	int importBuffers(BufferPool *pool);
	int exportBuffers(ImgUOutput *output, BufferPool *pool);
	void freeBuffers();

	int start();
	int stop();

	int linkSetup(const std::string &source, unsigned int sourcePad,
		      const std::string &sink, unsigned int sinkPad,
		      bool enable);
	int enableLinks(bool enable);

	unsigned int index_;
	std::string name_;
	MediaDevice *media_;

	V4L2Subdevice *imgu_;
	V4L2Device *input_;
	ImgUOutput output_;
	ImgUOutput viewfinder_;
	ImgUOutput stat_;
	/* \todo Add param video device for 3A tuning */

	BufferPool vfPool_;
	BufferPool statPool_;
	BufferPool outPool_;
};

class CIO2Device
{
public:
	static constexpr unsigned int CIO2_BUFFER_COUNT = 4;

	CIO2Device()
		: output_(nullptr), csi2_(nullptr), sensor_(nullptr)
	{
	}

	~CIO2Device()
	{
		delete output_;
		delete csi2_;
		delete sensor_;
	}

	int init(const MediaDevice *media, unsigned int index);
	int configure(const Size &size,
		      V4L2DeviceFormat *outputFormat);

	BufferPool *exportBuffers();
	void freeBuffers();

	int start();
	int stop();

	static int mediaBusToFormat(unsigned int code);

	V4L2Device *output_;
	V4L2Subdevice *csi2_;
	CameraSensor *sensor_;

	BufferPool pool_;
};

class IPU3Stream : public Stream
{
public:
	IPU3Stream()
		: active_(false), device_(nullptr)
	{
	}

	bool active_;
	std::string name_;
	ImgUDevice::ImgUOutput *device_;
};

class IPU3CameraData : public CameraData
{
public:
	IPU3CameraData(PipelineHandler *pipe)
		: CameraData(pipe)
	{
	}

	void imguOutputBufferReady(Buffer *buffer);
	void imguInputBufferReady(Buffer *buffer);
	void cio2BufferReady(Buffer *buffer);

	CIO2Device cio2_;
	ImgUDevice *imgu_;

	IPU3Stream outStream_;
	IPU3Stream vfStream_;
};

class IPU3CameraConfiguration : public CameraConfiguration
{
public:
	IPU3CameraConfiguration(Camera *camera, IPU3CameraData *data);

	Status validate() override;

	const V4L2SubdeviceFormat &sensorFormat() { return sensorFormat_; }
	const std::vector<const IPU3Stream *> &streams() { return streams_; }

private:
	static constexpr unsigned int IPU3_BUFFER_COUNT = 4;

	void adjustStream(StreamConfiguration &cfg, bool scale);

	/*
	 * The IPU3CameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	const IPU3CameraData *data_;

	V4L2SubdeviceFormat sensorFormat_;
	std::vector<const IPU3Stream *> streams_;
};

class PipelineHandlerIPU3 : public PipelineHandler
{
public:
	PipelineHandlerIPU3(CameraManager *manager);

	CameraConfiguration *generateConfiguration(Camera *camera,
		const StreamRoles &roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int allocateBuffers(Camera *camera,
			    const std::set<Stream *> &streams) override;
	int freeBuffers(Camera *camera,
			const std::set<Stream *> &streams) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequest(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	IPU3CameraData *cameraData(const Camera *camera)
	{
		return static_cast<IPU3CameraData *>(
			PipelineHandler::cameraData(camera));
	}

	int registerCameras();

	ImgUDevice imgu0_;
	ImgUDevice imgu1_;
	MediaDevice *cio2MediaDev_;
	MediaDevice *imguMediaDev_;
};

IPU3CameraConfiguration::IPU3CameraConfiguration(Camera *camera,
						 IPU3CameraData *data)
	: CameraConfiguration()
{
	camera_ = camera->shared_from_this();
	data_ = data;
}

void IPU3CameraConfiguration::adjustStream(StreamConfiguration &cfg, bool scale)
{
	/* The only pixel format the driver supports is NV12. */
	cfg.pixelFormat = V4L2_PIX_FMT_NV12;

	if (scale) {
		/*
		 * Provide a suitable default that matches the sensor aspect
		 * ratio.
		 */
		if (!cfg.size.width || !cfg.size.height) {
			cfg.size.width = 1280;
			cfg.size.height = 1280 * sensorFormat_.size.height
					/ sensorFormat_.size.width;
		}

		/*
		 * \todo: Clamp the size to the hardware bounds when we will
		 * figure them out.
		 *
		 * \todo: Handle the scaler (BDS) restrictions. The BDS can
		 * only scale with the same factor in both directions, and the
		 * scaling factor is limited to a multiple of 1/32. At the
		 * moment the ImgU driver hides these constraints by applying
		 * additional cropping, this should be fixed on the driver
		 * side, and cropping should be exposed to us.
		 */
	} else {
		/*
		 * \todo: Properly support cropping when the ImgU driver
		 * interface will be cleaned up.
		 */
		cfg.size = sensorFormat_.size;
	}

	/*
	 * Clamp the size to match the ImgU alignment constraints. The width
	 * shall be a multiple of 8 pixels and the height a multiple of 4
	 * pixels.
	 */
	if (cfg.size.width % 8 || cfg.size.height % 4) {
		cfg.size.width &= ~7;
		cfg.size.height &= ~3;
	}

	cfg.bufferCount = IPU3_BUFFER_COUNT;
}

CameraConfiguration::Status IPU3CameraConfiguration::validate()
{
	const CameraSensor *sensor = data_->cio2_.sensor_;
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/* Cap the number of entries to the available streams. */
	if (config_.size() > 2) {
		config_.resize(2);
		status = Adjusted;
	}

	/*
	 * Select the sensor format by collecting the maximum width and height
	 * and picking the closest larger match, as the IPU3 can downscale
	 * only. If no resolution is requested for any stream, or if no sensor
	 * resolution is large enough, pick the largest one.
	 */
	Size size = {};

	for (const StreamConfiguration &cfg : config_) {
		if (cfg.size.width > size.width)
			size.width = cfg.size.width;
		if (cfg.size.height > size.height)
			size.height = cfg.size.height;
	}

	if (!size.width || !size.height)
		size = sensor->resolution();

	sensorFormat_ = sensor->getFormat({ MEDIA_BUS_FMT_SBGGR10_1X10,
					    MEDIA_BUS_FMT_SGBRG10_1X10,
					    MEDIA_BUS_FMT_SGRBG10_1X10,
					    MEDIA_BUS_FMT_SRGGB10_1X10 },
					  size);
	if (!sensorFormat_.size.width || !sensorFormat_.size.height)
		sensorFormat_.size = sensor->resolution();

	/*
	 * Verify and update all configuration entries, and assign a stream to
	 * each of them. The viewfinder stream can scale, while the output
	 * stream can crop only, so select the output stream when the requested
	 * resolution is equal to the sensor resolution, and the viewfinder
	 * stream otherwise.
	 */
	std::set<const IPU3Stream *> availableStreams = {
		&data_->outStream_,
		&data_->vfStream_,
	};

	streams_.clear();
	streams_.reserve(config_.size());

	for (unsigned int i = 0; i < config_.size(); ++i) {
		StreamConfiguration &cfg = config_[i];
		const unsigned int pixelFormat = cfg.pixelFormat;
		const Size size = cfg.size;
		const IPU3Stream *stream;

		if (cfg.size == sensorFormat_.size)
			stream = &data_->outStream_;
		else
			stream = &data_->vfStream_;

		if (availableStreams.find(stream) == availableStreams.end())
			stream = *availableStreams.begin();

		LOG(IPU3, Debug)
			<< "Assigned '" << stream->name_ << "' to stream " << i;

		bool scale = stream == &data_->vfStream_;
		adjustStream(config_[i], scale);

		if (cfg.pixelFormat != pixelFormat || cfg.size != size) {
			LOG(IPU3, Debug)
				<< "Stream " << i << " configuration adjusted to "
				<< cfg.toString();
			status = Adjusted;
		}

		streams_.push_back(stream);
		availableStreams.erase(stream);
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
	IPU3CameraConfiguration *config;
	std::set<IPU3Stream *> streams = {
		&data->outStream_,
		&data->vfStream_,
	};

	config = new IPU3CameraConfiguration(camera, data);

	for (const StreamRole role : roles) {
		StreamConfiguration cfg = {};
		IPU3Stream *stream = nullptr;

		switch (role) {
		case StreamRole::StillCapture:
			/*
			 * Pick the output stream by default as the Viewfinder
			 * and VideoRecording roles are not allowed on
			 * the output stream.
			 */
			if (streams.find(&data->outStream_) != streams.end()) {
				stream = &data->outStream_;
			} else if (streams.find(&data->vfStream_) != streams.end()) {
				stream = &data->vfStream_;
			} else {
				LOG(IPU3, Error)
					<< "No stream available for requested role "
					<< role;
				break;
			}

			/*
			 * FIXME: Soraka: the maximum resolution reported by
			 * both sensors (2592x1944 for ov5670 and 4224x3136 for
			 * ov13858) are returned as default configurations but
			 * they're not correctly processed by the ImgU.
			 * Resolutions up tp 2560x1920 have been validated.
			 *
			 * \todo Clarify ImgU alignment requirements.
			 */
			cfg.size = { 2560, 1920 };

			break;

		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording: {
			/*
			 * We can't use the 'output' stream for viewfinder or
			 * video capture roles.
			 *
			 * \todo This is an artificial limitation until we
			 * figure out the exact capabilities of the hardware.
			 */
			if (streams.find(&data->vfStream_) == streams.end()) {
				LOG(IPU3, Error)
					<< "No stream available for requested role "
					<< role;
				break;
			}

			stream = &data->vfStream_;

			/*
			 * Align the default viewfinder size to the maximum
			 * available sensor resolution and to the IPU3
			 * alignment constraints.
			 */
			const Size &res = data->cio2_.sensor_->resolution();
			unsigned int width = std::min(1280U, res.width);
			unsigned int height = std::min(720U, res.height);
			cfg.size = { width & ~7, height & ~3 };

			break;
		}

		default:
			LOG(IPU3, Error)
				<< "Requested stream role not supported: " << role;
			break;
		}

		if (!stream) {
			delete config;
			return nullptr;
		}

		streams.erase(stream);

		config->addConfiguration(cfg);
	}

	config->validate();

	return config;
}

int PipelineHandlerIPU3::configure(Camera *camera, CameraConfiguration *c)
{
	IPU3CameraConfiguration *config =
		static_cast<IPU3CameraConfiguration *>(c);
	IPU3CameraData *data = cameraData(camera);
	IPU3Stream *outStream = &data->outStream_;
	IPU3Stream *vfStream = &data->vfStream_;
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	int ret;

	/*
	 * \todo: Enable links selectively based on the requested streams.
	 * As of now, enable all links unconditionally.
	 */
	ret = data->imgu_->enableLinks(true);
	if (ret)
		return ret;

	/*
	 * Pass the requested stream size to the CIO2 unit and get back the
	 * adjusted format to be propagated to the ImgU output devices.
	 */
	const Size &sensorSize = config->sensorFormat().size;
	V4L2DeviceFormat cio2Format = {};
	ret = cio2->configure(sensorSize, &cio2Format);
	if (ret)
		return ret;

	ret = imgu->configureInput(sensorSize, &cio2Format);
	if (ret)
		return ret;

	/* Apply the format to the configured streams output devices. */
	outStream->active_ = false;
	vfStream->active_ = false;

	for (unsigned int i = 0; i < config->size(); ++i) {
		/*
		 * Use a const_cast<> here instead of storing a mutable stream
		 * pointer in the configuration to let the compiler catch
		 * unwanted modifications of camera data in the configuration
		 * validate() implementation.
		 */
		IPU3Stream *stream = const_cast<IPU3Stream *>(config->streams()[i]);
		StreamConfiguration &cfg = (*config)[i];

		stream->active_ = true;
		cfg.setStream(stream);

		ret = imgu->configureOutput(stream->device_, cfg);
		if (ret)
			return ret;
	}

	/*
	 * As we need to set format also on the non-active streams, use
	 * the configuration of the active one for that purpose (there should
	 * be at least one active stream in the configuration request).
	 */
	if (!outStream->active_) {
		ret = imgu->configureOutput(outStream->device_, config->at(0));
		if (ret)
			return ret;
	}

	if (!vfStream->active_) {
		ret = imgu->configureOutput(vfStream->device_, config->at(0));
		if (ret)
			return ret;
	}

	/*
	 * Apply the largest available format to the stat node.
	 * \todo Revise this when we'll actually use the stat node.
	 */
	StreamConfiguration statCfg = {};
	statCfg.size = cio2Format.size;

	ret = imgu->configureOutput(&imgu->stat_, statCfg);
	if (ret)
		return ret;

	return 0;
}

/**
 * \todo Clarify if 'viewfinder' and 'stat' nodes have to be set up and
 * started even if not in use. As of now, if not properly configured and
 * enabled, the ImgU processing pipeline stalls.
 *
 * In order to be able to start the 'viewfinder' and 'stat' nodes, we need
 * memory to be reserved.
 */
int PipelineHandlerIPU3::allocateBuffers(Camera *camera,
					 const std::set<Stream *> &streams)
{
	IPU3CameraData *data = cameraData(camera);
	IPU3Stream *outStream = &data->outStream_;
	IPU3Stream *vfStream = &data->vfStream_;
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	unsigned int bufferCount;
	int ret;

	/* Share buffers between CIO2 output and ImgU input. */
	BufferPool *pool = cio2->exportBuffers();
	if (!pool)
		return -ENOMEM;

	ret = imgu->importBuffers(pool);
	if (ret)
		goto error;

	/*
	 * Use for the stat's internal pool the same number of buffer as
	 * for the input pool.
	 * \todo To be revised when we'll actually use the stat node.
	 */
	bufferCount = pool->count();
	imgu->stat_.pool->createBuffers(bufferCount);
	ret = imgu->exportBuffers(&imgu->stat_, imgu->stat_.pool);
	if (ret)
		goto error;

	/* Allocate buffers for each active stream. */
	for (Stream *s : streams) {
		IPU3Stream *stream = static_cast<IPU3Stream *>(s);
		ImgUDevice::ImgUOutput *dev = stream->device_;

		ret = imgu->exportBuffers(dev, &stream->bufferPool());
		if (ret)
			goto error;
	}

	/*
	 * Allocate buffers also on non-active outputs; use the same number
	 * of buffers as the active ones.
	 */
	if (!outStream->active_) {
		bufferCount = vfStream->bufferPool().count();
		outStream->device_->pool->createBuffers(bufferCount);
		ret = imgu->exportBuffers(outStream->device_,
					  outStream->device_->pool);
		if (ret)
			goto error;
	}

	if (!vfStream->active_) {
		bufferCount = outStream->bufferPool().count();
		vfStream->device_->pool->createBuffers(bufferCount);
		ret = imgu->exportBuffers(vfStream->device_,
					  vfStream->device_->pool);
		if (ret)
			goto error;
	}

	return 0;

error:
	freeBuffers(camera, streams);

	return ret;
}

int PipelineHandlerIPU3::freeBuffers(Camera *camera,
				     const std::set<Stream *> &streams)
{
	IPU3CameraData *data = cameraData(camera);

	data->cio2_.freeBuffers();
	data->imgu_->freeBuffers();

	return 0;
}

int PipelineHandlerIPU3::start(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	int ret;

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
	LOG(IPU3, Error) << "Failed to start camera " << camera->name();

	return ret;
}

void PipelineHandlerIPU3::stop(Camera *camera)
{
	IPU3CameraData *data = cameraData(camera);
	int ret;

	ret = data->cio2_.stop();
	ret |= data->imgu_->stop();
	if (ret)
		LOG(IPU3, Warning) << "Failed to stop camera "
				   << camera->name();

	PipelineHandler::stop(camera);
}

int PipelineHandlerIPU3::queueRequest(Camera *camera, Request *request)
{
	int error = 0;

	for (auto it : request->buffers()) {
		IPU3Stream *stream = static_cast<IPU3Stream *>(it.first);
		Buffer *buffer = it.second;

		int ret = stream->device_->dev->queueBuffer(buffer);
		if (ret < 0)
			error = ret;
	}

	PipelineHandler::queueRequest(camera, request);

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

	/*
	 * FIXME: enabled links in one ImgU instance interfere with capture
	 * operations on the other one. This can be easily triggered by
	 * capturing from one camera and then trying to capture from the other
	 * one right after, without disabling media links in the media graph
	 * first.
	 *
	 * The tricky part here is where to disable links on the ImgU instance
	 * which is currently not in use:
	 * 1) Link enable/disable cannot be done at start/stop time as video
	 * devices needs to be linked first before format can be configured on
	 * them.
	 * 2) As link enable has to be done at the least in configure(),
	 * before configuring formats, the only place where to disable links
	 * would be 'stop()', but the Camera class state machine allows
	 * start()<->stop() sequences without any configure() in between.
	 *
	 * As of now, disable all links in the media graph at 'match()' time,
	 * to allow testing different cameras in different test applications
	 * runs. A test application that would use two distinct cameras without
	 * going through a library teardown->match() sequence would fail
	 * at the moment.
	 */
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
			utils::make_unique<IPU3CameraData>(this);
		std::set<Stream *> streams = {
			&data->outStream_,
			&data->vfStream_,
		};
		CIO2Device *cio2 = &data->cio2_;

		ret = cio2->init(cio2MediaDev_, id);
		if (ret)
			continue;

		/**
		 * \todo Dynamically assign ImgU and output devices to each
		 * stream and camera; as of now, limit support to two cameras
		 * only, and assign imgu0 to the first one and imgu1 to the
		 * second.
		 */
		data->imgu_ = numCameras ? &imgu1_ : &imgu0_;
		data->outStream_.device_ = &data->imgu_->output_;
		data->outStream_.name_ = "output";
		data->vfStream_.device_ = &data->imgu_->viewfinder_;
		data->vfStream_.name_ = "viewfinder";

		/*
		 * Connect video devices' 'bufferReady' signals to their
		 * slot to implement the image processing pipeline.
		 *
		 * Frames produced by the CIO2 unit are passed to the
		 * associated ImgU input where they get processed and
		 * returned through the ImgU main and secondary outputs.
		 */
		data->cio2_.output_->bufferReady.connect(data.get(),
					&IPU3CameraData::cio2BufferReady);
		data->imgu_->input_->bufferReady.connect(data.get(),
					&IPU3CameraData::imguInputBufferReady);
		data->imgu_->output_.dev->bufferReady.connect(data.get(),
					&IPU3CameraData::imguOutputBufferReady);
		data->imgu_->viewfinder_.dev->bufferReady.connect(data.get(),
					&IPU3CameraData::imguOutputBufferReady);

		/* Create and register the Camera instance. */
		std::string cameraName = cio2->sensor_->entity()->name() + " "
				       + std::to_string(id);
		std::shared_ptr<Camera> camera = Camera::create(this,
								cameraName,
								streams);

		registerCamera(std::move(camera), std::move(data));

		LOG(IPU3, Info)
			<< "Registered Camera[" << numCameras << "] \""
			<< cameraName << "\""
			<< " connected to CSI-2 receiver " << id;

		numCameras++;
	}

	return numCameras ? 0 : -ENODEV;
}

/* -----------------------------------------------------------------------------
 * Buffer Ready slots
 */

/**
 * \brief Handle buffers completion at the ImgU input
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the ImgU input are immediately queued back to the
 * CIO2 unit to continue frame capture.
 */
void IPU3CameraData::imguInputBufferReady(Buffer *buffer)
{
	cio2_.output_->queueBuffer(buffer);
}

/**
 * \brief Handle buffers completion at the ImgU output
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the ImgU output are directed to the application.
 */
void IPU3CameraData::imguOutputBufferReady(Buffer *buffer)
{
	Request *request = buffer->request();

	if (!pipe_->completeBuffer(camera_, request, buffer))
		/* Request not completed yet, return here. */
		return;

	/* Complete the pending requests in queuing order. */
	while (1) {
		request = queuedRequests_.front();
		if (request->hasPendingBuffers())
			break;

		pipe_->completeRequest(camera_, request);
	}
}

/**
 * \brief Handle buffers completion at the CIO2 output
 * \param[in] buffer The completed buffer
 *
 * Buffers completed from the CIO2 are immediately queued to the ImgU unit
 * for further processing.
 */
void IPU3CameraData::cio2BufferReady(Buffer *buffer)
{
	imgu_->input_->queueBuffer(buffer);
}

/* -----------------------------------------------------------------------------
 * ImgU Device
 */

/**
 * \brief Initialize components of the ImgU instance
 * \param[in] mediaDevice The ImgU instance media device
 * \param[in] index The ImgU instance index
 *
 * Create and open the V4L2 devices and subdevices of the ImgU instance
 * with \a index.
 *
 * In case of errors the created V4L2Device and V4L2Subdevice instances
 * are destroyed at pipeline handler delete time.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::init(MediaDevice *media, unsigned int index)
{
	int ret;

	index_ = index;
	name_ = "ipu3-imgu " + std::to_string(index_);
	media_ = media;

	/*
	 * The media entities presence in the media device has been verified
	 * by the match() function: no need to check for newly created
	 * video devices and subdevice validity here.
	 */
	imgu_ = V4L2Subdevice::fromEntityName(media, name_);
	ret = imgu_->open();
	if (ret)
		return ret;

	input_ = V4L2Device::fromEntityName(media, name_ + " input");
	ret = input_->open();
	if (ret)
		return ret;

	output_.dev = V4L2Device::fromEntityName(media, name_ + " output");
	ret = output_.dev->open();
	if (ret)
		return ret;

	output_.pad = PAD_OUTPUT;
	output_.name = "output";
	output_.pool = &outPool_;

	viewfinder_.dev = V4L2Device::fromEntityName(media,
						     name_ + " viewfinder");
	ret = viewfinder_.dev->open();
	if (ret)
		return ret;

	viewfinder_.pad = PAD_VF;
	viewfinder_.name = "viewfinder";
	viewfinder_.pool = &vfPool_;

	stat_.dev = V4L2Device::fromEntityName(media, name_ + " 3a stat");
	ret = stat_.dev->open();
	if (ret)
		return ret;

	stat_.pad = PAD_STAT;
	stat_.name = "stat";
	stat_.pool = &statPool_;

	return 0;
}

/**
 * \brief Configure the ImgU unit input
 * \param[in] size The ImgU input frame size
 * \param[in] inputFormat The format to be applied to ImgU input
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configureInput(const Size &size,
			       V4L2DeviceFormat *inputFormat)
{
	/* Configure the ImgU input video device with the requested sizes. */
	int ret = input_->setFormat(inputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU input format = " << inputFormat->toString();

	/*
	 * \todo The IPU3 driver implementation shall be changed to use the
	 * input sizes as 'ImgU Input' subdevice sizes, and use the desired
	 * GDC output sizes to configure the crop/compose rectangles.
	 *
	 * The current IPU3 driver implementation uses GDC sizes as the
	 * 'ImgU Input' subdevice sizes, and the input video device sizes
	 * to configure the crop/compose rectangles, contradicting the
	 * V4L2 specification.
	 */
	Rectangle rect = {
		.x = 0,
		.y = 0,
		.w = inputFormat->size.width,
		.h = inputFormat->size.height,
	};
	ret = imgu_->setCrop(PAD_INPUT, &rect);
	if (ret)
		return ret;

	ret = imgu_->setCompose(PAD_INPUT, &rect);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU input feeder and BDS rectangle = "
			 << rect.toString();

	V4L2SubdeviceFormat imguFormat = {};
	imguFormat.mbus_code = MEDIA_BUS_FMT_FIXED;
	imguFormat.size = size;

	ret = imgu_->setFormat(PAD_INPUT, &imguFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU GDC format = " << imguFormat.toString();

	return 0;
}

/**
 * \brief Configure the ImgU unit \a id video output
 * \param[in] output The ImgU output device to configure
 * \param[in] cfg The requested configuration
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configureOutput(ImgUOutput *output,
				const StreamConfiguration &cfg)
{
	V4L2Device *dev = output->dev;
	unsigned int pad = output->pad;

	V4L2SubdeviceFormat imguFormat = {};
	imguFormat.mbus_code = MEDIA_BUS_FMT_FIXED;
	imguFormat.size = cfg.size;

	int ret = imgu_->setFormat(pad, &imguFormat);
	if (ret)
		return ret;

	/* No need to apply format to the stat node. */
	if (output == &stat_)
		return 0;

	V4L2DeviceFormat outputFormat = {};
	outputFormat.fourcc = V4L2_PIX_FMT_NV12;
	outputFormat.size = cfg.size;
	outputFormat.planesCount = 2;

	ret = dev->setFormat(&outputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU " << output->name << " format = "
			 << outputFormat.toString();

	return 0;
}

/**
 * \brief Import buffers from \a pool into the ImgU input
 * \param[in] pool The buffer pool to import
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::importBuffers(BufferPool *pool)
{
	int ret = input_->importBuffers(pool);
	if (ret) {
		LOG(IPU3, Error) << "Failed to import ImgU input buffers";
		return ret;
	}

	return 0;
}

/**
 * \brief Export buffers from \a output to the provided \a pool
 * \param[in] output The ImgU output device
 * \param[in] pool The buffer pool where to export buffers
 *
 * Export memory buffers reserved in the video device memory associated with
 * \a output id to the buffer pool provided as argument.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::exportBuffers(ImgUOutput *output, BufferPool *pool)
{
	int ret = output->dev->exportBuffers(pool);
	if (ret) {
		LOG(IPU3, Error) << "Failed to export ImgU "
				 << output->name << " buffers";
		return ret;
	}

	return 0;
}

/**
 * \brief Release buffers for all the ImgU video devices
 */
void ImgUDevice::freeBuffers()
{
	int ret;

	ret = output_.dev->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU output buffers";

	ret = stat_.dev->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU stat buffers";

	ret = viewfinder_.dev->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU viewfinder buffers";

	ret = input_->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU input buffers";
}

int ImgUDevice::start()
{
	int ret;

	/* Start the ImgU video devices. */
	ret = output_.dev->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU output";
		return ret;
	}

	ret = viewfinder_.dev->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU viewfinder";
		return ret;
	}

	ret = stat_.dev->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU stat";
		return ret;
	}

	ret = input_->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU input";
		return ret;
	}

	return 0;
}

int ImgUDevice::stop()
{
	int ret;

	ret = output_.dev->streamOff();
	ret |= viewfinder_.dev->streamOff();
	ret |= stat_.dev->streamOff();
	ret |= input_->streamOff();

	return ret;
}

/**
 * \brief Enable or disable a single link on the ImgU instance
 *
 * This method assumes the media device associated with the ImgU instance
 * is open.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::linkSetup(const std::string &source, unsigned int sourcePad,
			  const std::string &sink, unsigned int sinkPad,
			  bool enable)
{
	MediaLink *link = media_->link(source, sourcePad, sink, sinkPad);
	if (!link) {
		LOG(IPU3, Error)
			<< "Failed to get link: '" << source << "':"
			<< sourcePad << " -> '" << sink << "':" << sinkPad;
		return -ENODEV;
	}

	return link->setEnabled(enable);
}

/**
 * \brief Enable or disable all media links in the ImgU instance to prepare
 * for capture operations
 *
 * \todo This method will probably be removed or changed once links will be
 * enabled or disabled selectively.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::enableLinks(bool enable)
{
	std::string viewfinderName = name_ + " viewfinder";
	std::string outputName = name_ + " output";
	std::string statName = name_ + " 3a stat";
	std::string inputName = name_ + " input";
	int ret;

	ret = linkSetup(inputName, 0, name_, PAD_INPUT, enable);
	if (ret)
		return ret;

	ret = linkSetup(name_, PAD_OUTPUT, outputName, 0, enable);
	if (ret)
		return ret;

	ret = linkSetup(name_, PAD_VF, viewfinderName, 0, enable);
	if (ret)
		return ret;

	return linkSetup(name_, PAD_STAT, statName, 0, enable);
}

/*------------------------------------------------------------------------------
 * CIO2 Device
 */

/**
 * \brief Initialize components of the CIO2 device with \a index
 * \param[in] media The CIO2 media device
 * \param[in] index The CIO2 device index
 *
 * Create and open the video device and subdevices in the CIO2 instance at \a
 * index, if a supported image sensor is connected to the CSI-2 receiver of
 * this CIO2 instance.  Enable the media links connecting the CIO2 components
 * to prepare for capture operations and cached the sensor maximum size.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV No supported image sensor is connected to this CIO2 instance
 */
int CIO2Device::init(const MediaDevice *media, unsigned int index)
{
	int ret;

	/*
	 * Verify that a sensor subdevice is connected to this CIO2 instance
	 * and enable the media link between the two.
	 */
	std::string csi2Name = "ipu3-csi2 " + std::to_string(index);
	MediaEntity *csi2Entity = media->getEntityByName(csi2Name);
	const std::vector<MediaPad *> &pads = csi2Entity->pads();
	if (pads.empty())
		return -ENODEV;

	/* IPU3 CSI-2 receivers have a single sink pad at index 0. */
	MediaPad *sink = pads[0];
	const std::vector<MediaLink *> &links = sink->links();
	if (links.empty())
		return -ENODEV;

	MediaLink *link = links[0];
	MediaEntity *sensorEntity = link->source()->entity();
	sensor_ = new CameraSensor(sensorEntity);
	ret = sensor_->init();
	if (ret)
		return ret;

	ret = link->setEnabled(true);
	if (ret)
		return ret;

	/*
	 * Make sure the sensor produces at least one format compatible with
	 * the CIO2 requirements.
	 *
	 * utils::set_overlap requires the ranges to be sorted, keep the
	 * cio2Codes vector sorted in ascending order.
	 */
	const std::vector<unsigned int> cio2Codes{ MEDIA_BUS_FMT_SBGGR10_1X10,
						   MEDIA_BUS_FMT_SGRBG10_1X10,
						   MEDIA_BUS_FMT_SGBRG10_1X10,
						   MEDIA_BUS_FMT_SRGGB10_1X10 };
	const std::vector<unsigned int> &sensorCodes = sensor_->mbusCodes();
	if (!utils::set_overlap(sensorCodes.begin(), sensorCodes.end(),
				cio2Codes.begin(), cio2Codes.end())) {
		LOG(IPU3, Error)
			<< "Sensor " << sensor_->entity()->name()
			<< " has not format compatible with the IPU3";
		return -EINVAL;
	}

	/*
	 * \todo Define when to open and close video device nodes, as they
	 * might impact on power consumption.
	 */

	csi2_ = new V4L2Subdevice(csi2Entity);
	ret = csi2_->open();
	if (ret)
		return ret;

	std::string cio2Name = "ipu3-cio2 " + std::to_string(index);
	output_ = V4L2Device::fromEntityName(media, cio2Name);
	ret = output_->open();
	if (ret)
		return ret;

	return 0;
}

/**
 * \brief Configure the CIO2 unit
 * \param[in] size The requested CIO2 output frame size
 * \param[out] outputFormat The CIO2 unit output image format
 * \return 0 on success or a negative error code otherwise
 */
int CIO2Device::configure(const Size &size,
			  V4L2DeviceFormat *outputFormat)
{
	V4L2SubdeviceFormat sensorFormat;
	int ret;

	/*
	 * Apply the selected format to the sensor, the CSI-2 receiver and
	 * the CIO2 output device.
	 */
	sensorFormat = sensor_->getFormat({ MEDIA_BUS_FMT_SBGGR10_1X10,
					    MEDIA_BUS_FMT_SGBRG10_1X10,
					    MEDIA_BUS_FMT_SGRBG10_1X10,
					    MEDIA_BUS_FMT_SRGGB10_1X10 },
					  size);
	ret = sensor_->setFormat(&sensorFormat);
	if (ret)
		return ret;

	ret = csi2_->setFormat(0, &sensorFormat);
	if (ret)
		return ret;

	outputFormat->fourcc = mediaBusToFormat(sensorFormat.mbus_code);
	outputFormat->size = sensorFormat.size;
	outputFormat->planesCount = 1;

	ret = output_->setFormat(outputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "CIO2 output format " << outputFormat->toString();

	return 0;
}

/**
 * \brief Allocate CIO2 memory buffers and export them in a BufferPool
 *
 * Allocate memory buffers in the CIO2 video device and export them to
 * a buffer pool that can be imported by another device.
 *
 * \return The buffer pool with export buffers on success or nullptr otherwise
 */
BufferPool *CIO2Device::exportBuffers()
{
	pool_.createBuffers(CIO2_BUFFER_COUNT);

	int ret = output_->exportBuffers(&pool_);
	if (ret) {
		LOG(IPU3, Error) << "Failed to export CIO2 buffers";
		return nullptr;
	}

	return &pool_;
}

void CIO2Device::freeBuffers()
{
	if (output_->releaseBuffers())
		LOG(IPU3, Error) << "Failed to release CIO2 buffers";
}

int CIO2Device::start()
{
	int ret;

	for (Buffer &buffer : pool_.buffers()) {
		ret = output_->queueBuffer(&buffer);
		if (ret)
			return ret;
	}

	ret = output_->streamOn();
	if (ret)
		return ret;

	return 0;
}

int CIO2Device::stop()
{
	return output_->streamOff();
}

int CIO2Device::mediaBusToFormat(unsigned int code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return V4L2_PIX_FMT_IPU3_SBGGR10;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return V4L2_PIX_FMT_IPU3_SGBRG10;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return V4L2_PIX_FMT_IPU3_SGRBG10;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return V4L2_PIX_FMT_IPU3_SRGGB10;
	default:
		return -EINVAL;
	}
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU3);

} /* namespace libcamera */
