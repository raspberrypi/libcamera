/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipu3.cpp - Pipeline handler for Intel IPU3
 */

#include <iomanip>
#include <memory>
#include <vector>

#include <linux/media-bus-format.h>

#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

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
	int configureInput(const StreamConfiguration &config,
			   V4L2DeviceFormat *inputFormat);
	int configureOutput(ImgUOutput *output,
			    const StreamConfiguration &config);

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
	int configure(const StreamConfiguration &config,
		      V4L2DeviceFormat *outputFormat);

	BufferPool *exportBuffers();
	void freeBuffers();

	int start();
	int stop();

	static int mediaBusToFormat(unsigned int code);

	V4L2Device *output_;
	V4L2Subdevice *csi2_;
	V4L2Subdevice *sensor_;

	/* Maximum sizes and the mbus code used to produce them. */
	unsigned int mbusCode_;
	Size maxSize_;

	BufferPool pool_;
};

class PipelineHandlerIPU3 : public PipelineHandler
{
public:
	PipelineHandlerIPU3(CameraManager *manager);
	~PipelineHandlerIPU3();

	std::map<Stream *, StreamConfiguration>
	streamConfiguration(Camera *camera,
			    std::set<Stream *> &streams) override;
	int configureStreams(Camera *camera,
			     std::map<Stream *, StreamConfiguration> &config) override;

	int allocateBuffers(Camera *camera, Stream *stream) override;
	int freeBuffers(Camera *camera, Stream *stream) override;

	int start(Camera *camera) override;
	void stop(Camera *camera) override;

	int queueRequest(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator);

private:
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

		Stream stream_;
	};

	static constexpr unsigned int IPU3_BUFFER_COUNT = 4;

	IPU3CameraData *cameraData(const Camera *camera)
	{
		return static_cast<IPU3CameraData *>(
			PipelineHandler::cameraData(camera));
	}

	int registerCameras();

	ImgUDevice imgu0_;
	ImgUDevice imgu1_;
	std::shared_ptr<MediaDevice> cio2MediaDev_;
	std::shared_ptr<MediaDevice> imguMediaDev_;
};

PipelineHandlerIPU3::PipelineHandlerIPU3(CameraManager *manager)
	: PipelineHandler(manager), cio2MediaDev_(nullptr), imguMediaDev_(nullptr)
{
}

PipelineHandlerIPU3::~PipelineHandlerIPU3()
{
	if (cio2MediaDev_)
		cio2MediaDev_->release();

	if (imguMediaDev_)
		imguMediaDev_->release();
}

std::map<Stream *, StreamConfiguration>
PipelineHandlerIPU3::streamConfiguration(Camera *camera,
					 std::set<Stream *> &streams)
{
	std::map<Stream *, StreamConfiguration> configs;
	IPU3CameraData *data = cameraData(camera);
	StreamConfiguration *config = &configs[&data->stream_];

	/*
	 * FIXME: Soraka: the maximum resolution reported by both sensors
	 * (2592x1944 for ov5670 and 4224x3136 for ov13858) are returned as
	 * default configurations but they're not correctly processed by the
	 * ImgU. Resolutions up tp 2560x1920 have been validated.
	 *
	 * \todo Clarify ImgU alignement requirements.
	 */
	config->width = 2560;
	config->height = 1920;
	config->pixelFormat = V4L2_PIX_FMT_NV12;
	config->bufferCount = IPU3_BUFFER_COUNT;

	LOG(IPU3, Debug)
		<< "Stream format set to " << config->width << "x"
		<< config->height << "-0x" << std::hex << std::setfill('0')
		<< std::setw(8) << config->pixelFormat;

	return configs;
}

int PipelineHandlerIPU3::configureStreams(Camera *camera,
					  std::map<Stream *, StreamConfiguration> &config)
{
	IPU3CameraData *data = cameraData(camera);
	const StreamConfiguration &cfg = config[&data->stream_];
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	int ret;

	LOG(IPU3, Info)
		<< "Requested image format " << cfg.width << "x"
		<< cfg.height << "-0x" << std::hex << std::setfill('0')
		<< std::setw(8) << cfg.pixelFormat << " on camera '"
		<< camera->name() << "'";

	/*
	 * Verify that the requested size respects the IPU3 alignement
	 * requirements (the image width shall be a multiple of 8 pixels and
	 * its height a multiple of 4 pixels) and the camera maximum sizes.
	 *
	 * \todo: consider the BDS scaling factor requirements:
	 * "the downscaling factor must be an integer value multiple of 1/32"
	 */
	if (cfg.width % 8 || cfg.height % 4) {
		LOG(IPU3, Error) << "Invalid stream size: bad alignment";
		return -EINVAL;
	}

	if (cfg.width > cio2->maxSize_.width ||
	    cfg.height > cio2->maxSize_.height) {
		LOG(IPU3, Error)
			<< "Invalid stream size: larger than sensor resolution";
		return -EINVAL;
	}

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
	V4L2DeviceFormat cio2Format = {};
	ret = cio2->configure(cfg, &cio2Format);
	if (ret)
		return ret;

	ret = imgu->configureInput(cfg, &cio2Format);
	if (ret)
		return ret;

	/* Apply the format to the ImgU output, viewfinder and stat. */
	ret = imgu->configureOutput(&imgu->output_, cfg);
	if (ret)
		return ret;

	ret = imgu->configureOutput(&imgu->viewfinder_, cfg);
	if (ret)
		return ret;

	ret = imgu->configureOutput(&imgu->stat_, cfg);
	if (ret)
		return ret;

	return 0;
}

int PipelineHandlerIPU3::allocateBuffers(Camera *camera, Stream *stream)
{
	IPU3CameraData *data = cameraData(camera);
	CIO2Device *cio2 = &data->cio2_;
	ImgUDevice *imgu = data->imgu_;
	int ret;

	/* Share buffers between CIO2 output and ImgU input. */
	BufferPool *pool = cio2->exportBuffers();
	if (!pool)
		return -ENOMEM;

	ret = imgu->importBuffers(pool);
	if (ret)
		return ret;

	/* Export ImgU output buffers to the stream's pool. */
	ret = imgu->exportBuffers(&imgu->output_, &stream->bufferPool());
	if (ret)
		return ret;

	/*
	 * Reserve memory in viewfinder and stat output devices. Use the
	 * same number of buffers as the ones requested for the output
	 * stream.
	 */
	unsigned int bufferCount = stream->bufferPool().count();

	imgu->viewfinder_.pool->createBuffers(bufferCount);
	ret = imgu->exportBuffers(&imgu->viewfinder_, imgu->viewfinder_.pool);
	if (ret)
		return ret;

	imgu->stat_.pool->createBuffers(bufferCount);
	ret = imgu->exportBuffers(&imgu->stat_, imgu->stat_.pool);
	if (ret)
		return ret;

	return 0;
}

int PipelineHandlerIPU3::freeBuffers(Camera *camera, Stream *stream)
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
	IPU3CameraData *data = cameraData(camera);
	V4L2Device *output = data->imgu_->output_.dev;
	Stream *stream = &data->stream_;

	/* Queue a buffer to the ImgU output for capture. */
	Buffer *buffer = request->findBuffer(stream);
	if (!buffer) {
		LOG(IPU3, Error)
			<< "Attempt to queue request with invalid stream";
		return -ENOENT;
	}

	int ret = output->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	PipelineHandler::queueRequest(camera, request);

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

	/*
	 * It is safe to acquire both media devices at this point as
	 * DeviceEnumerator::search() skips the busy ones for us.
	 */
	cio2MediaDev_ = enumerator->search(cio2_dm);
	if (!cio2MediaDev_)
		return false;

	cio2MediaDev_->acquire();

	imguMediaDev_ = enumerator->search(imgu_dm);
	if (!imguMediaDev_)
		return false;

	imguMediaDev_->acquire();

	/*
	 * Disable all links that are enabled by default on CIO2, as camera
	 * creation enables all valid links it finds.
	 *
	 * Close the CIO2 media device after, as links are enabled and should
	 * not need to be changed after.
	 */
	if (cio2MediaDev_->open())
		return false;

	if (cio2MediaDev_->disableLinks()) {
		cio2MediaDev_->close();
		return false;
	}

	if (imguMediaDev_->open()) {
		cio2MediaDev_->close();
		return false;
	}

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
	 * 2) As link enable has to be done at the least in configureStreams,
	 * before configuring formats, the only place where to disable links
	 * would be 'stop()', but the Camera class state machine allows
	 * start()<->stop() sequences without any streamConfiguration() in
	 * between.
	 *
	 * As of now, disable all links in the media graph at 'match()' time,
	 * to allow testing different cameras in different test applications
	 * runs. A test application that would use two distinct cameras without
	 * going through a library teardown->match() sequence would fail
	 * at the moment.
	 */
	ret = imguMediaDev_->disableLinks();
	if (ret)
		goto error;

	ret = registerCameras();

error:
	cio2MediaDev_->close();
	imguMediaDev_->close();

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

	ret = imgu0_.init(imguMediaDev_.get(), 0);
	if (ret)
		return ret;

	ret = imgu1_.init(imguMediaDev_.get(), 1);
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
		std::set<Stream *> streams{ &data->stream_ };
		CIO2Device *cio2 = &data->cio2_;

		ret = cio2->init(cio2MediaDev_.get(), id);
		if (ret)
			continue;

		/**
		 * \todo Dynamically assign ImgU devices; as of now, limit
		 * support to two cameras only, and assign imgu0 to the first
		 * one and imgu1 to the second.
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
		data->cio2_.output_->bufferReady.connect(data.get(),
					&IPU3CameraData::cio2BufferReady);
		data->imgu_->input_->bufferReady.connect(data.get(),
					&IPU3CameraData::imguInputBufferReady);
		data->imgu_->output_.dev->bufferReady.connect(data.get(),
					&IPU3CameraData::imguOutputBufferReady);

		/* Create and register the Camera instance. */
		std::string cameraName = cio2->sensor_->entityName() + " "
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
 * \param buffer The completed buffer
 *
 * Buffers completed from the ImgU input are immediately queued back to the
 * CIO2 unit to continue frame capture.
 */
void PipelineHandlerIPU3::IPU3CameraData::imguInputBufferReady(Buffer *buffer)
{
	cio2_.output_->queueBuffer(buffer);
}

/**
 * \brief Handle buffers completion at the ImgU output
 * \param buffer The completed buffer
 *
 * Buffers completed from the ImgU output are directed to the application.
 */
void PipelineHandlerIPU3::IPU3CameraData::imguOutputBufferReady(Buffer *buffer)
{
	Request *request = queuedRequests_.front();

	pipe_->completeBuffer(camera_, request, buffer);
	pipe_->completeRequest(camera_, request);
}

/**
 * \brief Handle buffers completion at the CIO2 output
 * \param buffer The completed buffer
 *
 * Buffers completed from the CIO2 are immediately queued to the ImgU unit
 * for further processing.
 */
void PipelineHandlerIPU3::IPU3CameraData::cio2BufferReady(Buffer *buffer)
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
 * \param[in] config The requested stream configuration
 * \param[in] inputFormat The format to be applied to ImgU input
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configureInput(const StreamConfiguration &config,
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
		.w = inputFormat->width,
		.h = inputFormat->height,
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
	imguFormat.width = config.width;
	imguFormat.height = config.height;
	imguFormat.mbus_code = MEDIA_BUS_FMT_FIXED;

	ret = imgu_->setFormat(PAD_INPUT, &imguFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU GDC format = " << imguFormat.toString();

	return 0;
}

/**
 * \brief Configure the ImgU unit \a id video output
 * \param[in] output The ImgU output device to configure
 * \param[in] config The requested configuration
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configureOutput(ImgUOutput *output,
				const StreamConfiguration &config)
{
	V4L2Device *dev = output->dev;
	unsigned int pad = output->pad;

	V4L2SubdeviceFormat imguFormat = {};
	imguFormat.width = config.width;
	imguFormat.height = config.height;
	imguFormat.mbus_code = MEDIA_BUS_FMT_FIXED;

	int ret = imgu_->setFormat(pad, &imguFormat);
	if (ret)
		return ret;

	/* No need to apply format to the stat node. */
	if (output == &stat_)
		return 0;

	V4L2DeviceFormat outputFormat = {};
	outputFormat.width = config.width;
	outputFormat.height = config.height;
	outputFormat.fourcc = V4L2_PIX_FMT_NV12;
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
 *
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

	/* \todo Establish rules to handle media devices open/close. */
	ret = media_->open();
	if (ret)
		return ret;

	ret = linkSetup(inputName, 0, name_, PAD_INPUT, enable);
	if (ret)
		goto done;

	ret = linkSetup(name_, PAD_OUTPUT, outputName, 0, enable);
	if (ret)
		goto done;

	ret = linkSetup(name_, PAD_VF, viewfinderName, 0, enable);
	if (ret)
		goto done;

	ret = linkSetup(name_, PAD_STAT, statName, 0, enable);

done:
	media_->close();

	return ret;
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
	if (sensorEntity->function() != MEDIA_ENT_F_CAM_SENSOR)
		return -ENODEV;

	ret = link->setEnabled(true);
	if (ret)
		return ret;

	/*
	 * Now that we're sure a sensor subdevice is connected, make sure it
	 * produces at least one image format compatible with CIO2 requirements
	 * and cache the camera maximum size.
	 *
	 * \todo Define when to open and close video device nodes, as they
	 * might impact on power consumption.
	 */
	sensor_ = new V4L2Subdevice(sensorEntity);
	ret = sensor_->open();
	if (ret)
		return ret;

	for (auto it : sensor_->formats(0)) {
		int mbusCode = mediaBusToFormat(it.first);
		if (mbusCode < 0)
			continue;

		for (const SizeRange &size : it.second) {
			if (maxSize_.width < size.maxWidth &&
			    maxSize_.height < size.maxHeight) {
				maxSize_.width = size.maxWidth;
				maxSize_.height = size.maxHeight;
				mbusCode_ = mbusCode;
			}
		}
	}
	if (maxSize_.width == 0) {
		LOG(IPU3, Info) << "Sensor '" << sensor_->entityName()
				<< "' detected, but no supported image format "
				<< " found: skip camera creation";
		return -ENODEV;
	}

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
 * \param[in] config The requested configuration
 * \param[out] outputFormat The CIO2 unit output image format
 *
 * \return 0 on success or a negative error code otherwise
 */
int CIO2Device::configure(const StreamConfiguration &config,
			  V4L2DeviceFormat *outputFormat)
{
	unsigned int imageSize = config.width * config.height;
	V4L2SubdeviceFormat sensorFormat = {};
	unsigned int best = ~0;
	int ret;

	for (auto it : sensor_->formats(0)) {
		/* Only consider formats consumable by the CIO2 unit. */
		if (mediaBusToFormat(it.first) < 0)
			continue;

		for (const SizeRange &size : it.second) {
			/*
			 * Only select formats bigger than the requested sizes
			 * as the IPU3 cannot up-scale.
			 *
			 * \todo: Unconditionally scale on the sensor as much
			 * as possible. This will need to be revisited when
			 * implementing the scaling policy.
			 */
			if (size.maxWidth < config.width ||
			    size.maxHeight < config.height)
				continue;

			unsigned int diff = size.maxWidth * size.maxHeight
					  - imageSize;
			if (diff >= best)
				continue;

			best = diff;

			sensorFormat.width = size.maxWidth;
			sensorFormat.height = size.maxHeight;
			sensorFormat.mbus_code = it.first;
		}
	}

	/*
	 * Apply the selected format to the sensor, the CSI-2 receiver and
	 * the CIO2 output device.
	 */
	ret = sensor_->setFormat(0, &sensorFormat);
	if (ret)
		return ret;

	ret = csi2_->setFormat(0, &sensorFormat);
	if (ret)
		return ret;

	outputFormat->width = sensorFormat.width;
	outputFormat->height = sensorFormat.height;
	outputFormat->fourcc = mediaBusToFormat(sensorFormat.mbus_code);
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
