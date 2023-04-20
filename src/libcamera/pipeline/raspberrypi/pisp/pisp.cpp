/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * pisp.cpp - Pipeline handler for PiSP based Raspberry Pi devices
 */

#include <unordered_map>

#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <libcamera/formats.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "libpisp/backend/backend.hpp"
#include "libpisp/common/pisp_logging.hpp"
#include "libpisp/common/pisp_utils.hpp"
#include "libpisp/frontend/frontend.hpp"
#include "libpisp/variants/pisp_variant.hpp"

#include "pipeline_base.h"
#include "rpi_stream.h"
#include "shared_mem_object.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

using StreamFlags = RPi::Stream::Flags;

namespace {

enum class Cfe : unsigned int { Output0, Embedded, Stats, Config };
enum class Isp : unsigned int { Input, Output0, Output1, Tdn, Stitch, Config };

uint32_t mbusCodeUnpacked16(unsigned int mbus_code)
{
	BayerFormat bayer = BayerFormat::fromMbusCode(mbus_code);
	BayerFormat bayer16(bayer.order, 16, BayerFormat::Packing::None);

	bool valid;
	return bayer16.toMbusCode(valid);
}

uint8_t toPiSPBayerOrder(V4L2PixelFormat format)
{
	BayerFormat bayer = BayerFormat::fromV4L2PixelFormat(format);

	switch (bayer.order) {
	case BayerFormat::Order::BGGR:
		return PISP_BAYER_ORDER_BGGR;
	case BayerFormat::Order::GBRG:
		return PISP_BAYER_ORDER_GBRG;
	case BayerFormat::Order::GRBG:
		return PISP_BAYER_ORDER_GRBG;
	case BayerFormat::Order::RGGB:
		return PISP_BAYER_ORDER_RGGB;
	default:
		ASSERT(0);
		return -1;
	}
}

pisp_image_format_config toPiSPImageFormat(V4L2DeviceFormat &format)
{
	pisp_image_format_config image = {};

	image.width = format.size.width;
	image.height = format.size.height;
	image.stride = format.planes[0].bpl;

	PixelFormat pix = format.fourcc.toPixelFormat();

	if (RPi::PipelineHandlerBase::isRaw(pix)) {
		BayerFormat bayer = BayerFormat::fromPixelFormat(pix);
		switch (bayer.packing) {
		case BayerFormat::Packing::None:
			image.format = PISP_IMAGE_FORMAT_BPS_16 + PISP_IMAGE_FORMAT_UNCOMPRESSED;
			break;
		case BayerFormat::Packing::PISP1:
			image.format = PISP_IMAGE_FORMAT_COMPRESSION_MODE_1;
			break;
		case BayerFormat::Packing::PISP2:
			image.format = PISP_IMAGE_FORMAT_COMPRESSION_MODE_2;
			break;
		default:
			ASSERT(0);
		}
	} else if (pix == formats::YUV420) {
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL + PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_420 + PISP_IMAGE_FORMAT_PLANARITY_PLANAR;
		image.stride2 = image.stride / 2;
	} else if (pix == formats::RGB888) {
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL;
	} else if (pix == formats::BGR888) {
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL;
	} else {
		LOG(RPI, Error) << "Pixel format " << pix << " unsupported";
		ASSERT(0);
	}

	return image;
}

void computeOptimalStride(V4L2DeviceFormat &format)
{
	pisp_image_format_config fmt = toPiSPImageFormat(format);

	libpisp::compute_optimal_stride(fmt);

	uint32_t fourcc = format.fourcc.fourcc();

	/*
	 * For YUV420/422 non-multiplanar formats, double the U/V stride for the
	 * Y-plane to ensure we get the optimal alignment on all three planes.
	 */
	if (fourcc == V4L2_PIX_FMT_YUV420 || fourcc == V4L2_PIX_FMT_YUV422P ||
	    fourcc == V4L2_PIX_FMT_YVU420)
		fmt.stride = fmt.stride2 * 2;

	format.planes[0].bpl = fmt.stride;
	format.planes[1].bpl = fmt.stride2;
	format.planes[2].bpl = fmt.stride2;
}

bool calculateCscConfiguration(const V4L2DeviceFormat &v4l2Format, pisp_be_ccm_config &csc)
{
	const PixelFormat &pixFormat = v4l2Format.fourcc.toPixelFormat();
	const PixelFormatInfo &info = PixelFormatInfo::info(pixFormat);
	memset(&csc, 0, sizeof(csc));

	if (info.colourEncoding == PixelFormatInfo::ColourEncodingYUV) {
		/* Look up the correct YCbCr conversion matrix for this colour space. */
		if (v4l2Format.colorSpace == ColorSpace::Sycc)
			libpisp::initialise_ycbcr(csc, "jpeg");
		else if (v4l2Format.colorSpace == ColorSpace::Smpte170m)
			libpisp::initialise_ycbcr(csc, "smpte170m");
		else if (v4l2Format.colorSpace == ColorSpace::Rec709)
			libpisp::initialise_ycbcr(csc, "rec709");
		else {
			LOG(RPI, Warning)
				<< "Unrecognised colour space " << ColorSpace::toString(v4l2Format.colorSpace)
				<< ", defaulting to sYCC";
			libpisp::initialise_ycbcr(csc, "jpeg");
		}
		return true;
	}
	/* There will be more formats to check for in due course. */
	else if (pixFormat == formats::RGB888) {
		/* Identity matrix but with RB colour swap. */
		csc.coeffs[2] = csc.coeffs[4] = csc.coeffs[6] = 1 << 10;
		return true;
	}

	return false;
}

void setupOutputClipping(const V4L2DeviceFormat &v4l2Format, pisp_be_output_format_config &outputFormat)
{
	const PixelFormat &pixFormat = v4l2Format.fourcc.toPixelFormat();
	const PixelFormatInfo &info = PixelFormatInfo::info(pixFormat);

	if (info.colourEncoding == PixelFormatInfo::ColourEncodingYUV) {
		if (v4l2Format.colorSpace == ColorSpace::Sycc) {
			outputFormat.lo = 0;
			outputFormat.hi = 65535;
			outputFormat.lo2 = 0;
			outputFormat.hi2 = 65535;
		}
		else if (v4l2Format.colorSpace == ColorSpace::Smpte170m ||
			 v4l2Format.colorSpace == ColorSpace::Rec709) {
			outputFormat.lo = 16 << 8;
			outputFormat.hi = 235 << 8;
			outputFormat.lo2 = 16 << 8;
			outputFormat.hi2 = 240 << 8;
		}
		else {
			LOG(RPI, Warning)
				<< "Unrecognised colour space " << ColorSpace::toString(v4l2Format.colorSpace)
				<< ", using full range";
			outputFormat.lo = 0;
			outputFormat.hi = 65535;
			outputFormat.lo2 = 0;
			outputFormat.hi2 = 65535;
		}
	}
}

void do32BitConversion(void *mem, unsigned int width, unsigned int height, unsigned int stride)
{
	/*
	 * The arm64 version is actually not that much quicker because the
	 * vast bulk of the time is spent waiting for memory.
	 */
#if __aarch64__
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;
		unsigned int count = (width + 15) / 16;
		uint8_t *dest = ptr + count * 64;
		uint8_t *src = ptr + count * 48;

		// Pre-decrement would have been nice.
		asm volatile("movi v3.16b, #255 \n"
				"1: \n"
				"sub %[src], %[src], #48 \n"
				"sub %[dest], %[dest], #64 \n"
				"subs %[count], %[count], #1 \n"
				"ld3 {v0.16b, v1.16b, v2.16b}, [%[src]] \n"
				"st4 {v0.16b, v1.16b, v2.16b, v3.16b}, [%[dest]] \n"
				"b.gt 1b \n"
				: [count]"+r" (count)
				: [src]"r" (src), [dest]"r" (dest)
				: "cc", "v1", "v2", "v3", "v4", "memory"
				);
	}
#else
	std::vector<uint8_t> cache(4 * width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;
		memcpy(cache.data(), ptr, 3 * width);

		uint8_t *ptr3 = cache.data() + width * 3;
		uint8_t *ptr4 = cache.data() + width * 4;
		for (unsigned int i = 0; i < width; i++) {
			*(--ptr4) = 255;
			*(--ptr4) = *(--ptr3);
			*(--ptr4) = *(--ptr3);
			*(--ptr4) = *(--ptr3);
		}

		memcpy(ptr, cache.data(), 4 * width);
	}
#endif
}

} /* namespace */

using ::libpisp::BackEnd;
using ::libpisp::BCM2712_HW;
using ::libpisp::FrontEnd;

class PiSPCameraData final : public RPi::CameraData
{
public:
	PiSPCameraData(PipelineHandler *pipe)
		: RPi::CameraData(pipe),
		  fe_("pisp_frontend", true, BCM2712_HW),
		  be_("pisp_backend", BackEnd::Config({}), BCM2712_HW)
	{
		ASSERT(fe_ && be_);
		/* Initialise internal libpisp logging. */
		::libpisp::logging_init();
	}

	~PiSPCameraData()
	{
		freeBuffers();
	}

	V4L2VideoDevice::Formats ispFormats() const override
	{
		return isp_[Isp::Output0].dev()->formats();
	}

	V4L2VideoDevice::Formats rawFormats() const override
	{
		return cfe_[Cfe::Output0].dev()->formats();
	}

	V4L2VideoDevice *frontendDevice() override
	{
		return cfe_[Cfe::Output0].dev();
	}

	CameraConfiguration::Status platformValidate(std::vector<StreamParams> &rawStreams,
						     std::vector<StreamParams> &outStreams) const override;

	int platformPipelineConfigure(const std::unique_ptr<YamlObject> &root) override;

	void platformStart() override;
	void platformStop() override;
	void platformFreeBuffers() override;

	void cfeBufferDequeue(FrameBuffer *buffer);
	void beInputDequeue(FrameBuffer *buffer);
	void beOutputDequeue(FrameBuffer *buffer);

	void processStatsComplete(const ipa::RPi::BufferIds &buffers);
	void prepareIspComplete(const ipa::RPi::BufferIds &buffers);

	/* Array of CFE and ISP device streams and associated buffers/streams. */
	RPi::Device<Cfe, 4> cfe_;
	RPi::Device<Isp, 6> isp_;

	/* Frontend/Backend objects shared with the IPA. */
	RPi::SharedMemObject<FrontEnd> fe_;
	RPi::SharedMemObject<BackEnd> be_;

	std::unique_ptr<V4L2Subdevice> csi2Subdev_;
	std::unique_ptr<V4L2Subdevice> feSubdev_;

	std::vector<std::unique_ptr<FrameBuffer>> tdnBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> stitchBuffers_;

	struct Config {
		/*
		 * The minimum number of internal buffers to be allocated for
		 * the CFE Image stream.
		 */
		unsigned int minCfeBuffers;
		/*
		 * The minimum total (internal + external) buffer count used for
		 * the CFE Image stream.
		 *
		 * Note that:
		 * minTotalCFEBuffers must be >= 1, and
		 * minTotalCFEBuffers >= minCFEBuffers
		 */
		unsigned int minTotalCfeBuffers;
	};

	Config config_;

	bool adjustDeviceFormat(V4L2DeviceFormat &format) const override;

private:
	int platformConfigure(const V4L2SubdeviceFormat &sensorFormat,
			      std::optional<BayerFormat::Packing> packing,
			      const std::optional<ColorSpace> &yuvColorSpace,
			      std::vector<StreamParams> &rawStreams,
			      std::vector<StreamParams> &outStreams) override;

	int platformConfigureIpa([[maybe_unused]] ipa::RPi::ConfigParams &params) override
	{
		return 0;
	}

	int platformInitIpa(ipa::RPi::InitParams &params) override;

	int configureEntities(V4L2SubdeviceFormat &sensorFormat);
	int configureCfe();
	int configureBe(const std::optional<ColorSpace> &yuvColorSpace);

	void platformIspCrop() override;

	void prepareCfe();
	void runBackend(uint32_t bufferId);

	void tryRunPipeline() override;

	struct CfeJob {
		ControlList sensorControls;
		unsigned int delayContext;
		std::unordered_map<const RPi::Stream *, FrameBuffer *> buffers;
	};

	std::queue<CfeJob> cfeJobQueue_;

	bool cfeJobComplete() const
	{
		if (cfeJobQueue_.empty())
			return false;

		const CfeJob &job = cfeJobQueue_.back();
		return job.buffers.count(&cfe_[Cfe::Output0]) && job.buffers.count(&cfe_[Cfe::Stats]) &&
		       (!sensorMetadata_ || job.buffers.count(&cfe_[Cfe::Embedded]));
	}

	/* Offset for all compressed buffers; mode for TDN and Stitch. */
	static constexpr unsigned int DefaultCompressionOffset = 2048;
	static constexpr unsigned int DefaultCompressionMode = 1;
};

class PipelineHandlerPiSP : public RPi::PipelineHandlerBase
{
public:
	PipelineHandlerPiSP(CameraManager *manager)
		: RPi::PipelineHandlerBase(manager)
	{
	}

	~PipelineHandlerPiSP()
	{
	}

	bool match(DeviceEnumerator *enumerator) override;

private:
	PiSPCameraData *cameraData(Camera *camera)
	{
		return static_cast<PiSPCameraData *>(camera->_d());
	}

	std::unique_ptr<RPi::CameraData> allocateCameraData()
	{
		return std::make_unique<PiSPCameraData>(this);
	}

	int prepareBuffers(Camera *camera) override;
	int platformRegister(std::unique_ptr<RPi::CameraData> &cameraData, MediaDevice *cfe, MediaDevice *isp);
};

bool PipelineHandlerPiSP::match(DeviceEnumerator *enumerator)
{
	constexpr unsigned int numCfeDevices = 2;

	/*
	 * Loop over all CFE instances, but return out once a match is found.
	 * This is to ensure we correctly enumrate the camera when an instance
	 * of Unicam has registered with media controller, but has not registered
	 * device nodes due to a sensor subdevice failure.
	 */
	for (unsigned int i = 0; i < numCfeDevices; i++) {
		DeviceMatch cfe("rp1-cfe");
		MediaDevice *cfeDevice = acquireMediaDevice(enumerator, cfe);

		if (!cfeDevice) {
			LOG(RPI, Debug) << "Unable to acquire a CFE instance";
			break;
		}

		DeviceMatch isp("pispbe");
		MediaDevice *ispDevice = acquireMediaDevice(enumerator, isp);

		if (!ispDevice) {
			LOG(RPI, Debug) << "Unable to acquire ISP instance";
			break;
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

			int ret = RPi::PipelineHandlerBase::registerCamera(cfeDevice, "csi2", ispDevice, entity);
			if (ret)
				LOG(RPI, Error) << "Failed to register camera "
						<< entity->name() << ": " << ret;
			else
				numCameras++;
		}

		if (numCameras)
			return true;
	}

	return false;
}

int PipelineHandlerPiSP::prepareBuffers(Camera *camera)
{
	PiSPCameraData *data = cameraData(camera);
	unsigned int numRawBuffers = 0;
	int ret;

	for (Stream *s : camera->streams()) {
		if (PipelineHandlerBase::isRaw(s->configuration().pixelFormat)) {
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
			 * ISP input buffers are imported from the CFE, so follow
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
		} else if (stream == &data->cfe_[Cfe::Config] || stream == &data->cfe_[Cfe::Stats]) {
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

	data->isp_[Isp::Tdn].dev()->allocateBuffers(2, &data->tdnBuffers_);
	data->isp_[Isp::Stitch].dev()->allocateBuffers(2, &data->stitchBuffers_);

	/*
	 * Pass the stats and embedded data buffers to the IPA. No other
	 * buffers need to be passed.
	 */
	mapBuffers(camera, data->cfe_[Cfe::Stats].getBuffers(), RPi::MaskStats);
	if (data->sensorMetadata_)
		mapBuffers(camera, data->cfe_[Cfe::Embedded].getBuffers(),
			   RPi::MaskEmbeddedData);

	return 0;
}

int PipelineHandlerPiSP::platformRegister(std::unique_ptr<RPi::CameraData> &cameraData, MediaDevice *cfe, MediaDevice *isp)
{
	PiSPCameraData *data = static_cast<PiSPCameraData *>(cameraData.get());
	int ret;

	MediaEntity *cfeImage = cfe->getEntityByName("rp1-cfe-fe_image0");
	MediaEntity *cfeStats = cfe->getEntityByName("rp1-cfe-fe_stats");
	MediaEntity *cfeConfig = cfe->getEntityByName("rp1-cfe-fe_config");
	MediaEntity *ispInput = isp->getEntityByName("pispbe-input");
	MediaEntity *IpaPrepare = isp->getEntityByName("pispbe-config");
	MediaEntity *ispOutput0 = isp->getEntityByName("pispbe-output0");
	MediaEntity *ispOutput1 = isp->getEntityByName("pispbe-output1");
	MediaEntity *ispTdn = isp->getEntityByName("pispbe-tdn_output");
	MediaEntity *ispStitch = isp->getEntityByName("pispbe-stitch_output");

	if (!cfeImage || !cfeStats || !cfeConfig || !IpaPrepare || !ispOutput0 ||
	    !ispOutput1 || !ispTdn || !ispStitch)
		return -ENOENT;

	/* Locate and open the cfe video streams. */
	data->cfe_[Cfe::Output0] = RPi::Stream("CFE Image", cfeImage);
	data->cfe_[Cfe::Stats] = RPi::Stream("CFE Stats", cfeStats);
	data->cfe_[Cfe::Config] = RPi::Stream("CFE Config", cfeConfig, StreamFlags::Config | StreamFlags::RequiresMmap);

	/* An embedded data node will not be present if the sensor does not support it. */
	MediaEntity *cfeEmbedded = cfe->getEntityByName("rp1-cfe-embedded");
	if (cfeEmbedded) {
		data->cfe_[Cfe::Embedded] = RPi::Stream("CFE Embedded", cfeEmbedded);
		data->cfe_[Cfe::Embedded].dev()->bufferReady.connect(data,
								     &PiSPCameraData::cfeBufferDequeue);
	}

	/* Tag the ISP input stream as an import stream. */
	data->isp_[Isp::Input] = RPi::Stream("ISP Input", ispInput, StreamFlags::ImportOnly);
	data->isp_[Isp::Config] = RPi::Stream("ISP Config", IpaPrepare, StreamFlags::Config | StreamFlags::RequiresMmap);
	data->isp_[Isp::Output0] = RPi::Stream("ISP Output0", ispOutput0);
	data->isp_[Isp::Output1] = RPi::Stream("ISP Output1", ispOutput1);
	data->isp_[Isp::Tdn] = RPi::Stream("ISP TDN", ispTdn);
	data->isp_[Isp::Stitch] = RPi::Stream("ISP Stitch", ispStitch);

	if (data->sensorMetadata_ ^ !!data->cfe_[Cfe::Embedded].dev()) {
		LOG(RPI, Warning) << "Mismatch between CFE and CamHelper for embedded data usage!";
		data->sensorMetadata_ = false;
		if (data->cfe_[Cfe::Embedded].dev())
			data->cfe_[Cfe::Embedded].dev()->bufferReady.disconnect();
	}

	data->csi2Subdev_ = std::make_unique<V4L2Subdevice>(cfe->getEntityByName("csi2"));
	data->feSubdev_ = std::make_unique<V4L2Subdevice>(cfe->getEntityByName("pisp-fe"));
	data->csi2Subdev_->open();
	data->feSubdev_->open();

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
	if (data->sensorMetadata_)
		data->streams_.push_back(&data->cfe_[Cfe::Embedded]);

	data->streams_.push_back(&data->isp_[Isp::Input]);
	data->streams_.push_back(&data->isp_[Isp::Output0]);
	data->streams_.push_back(&data->isp_[Isp::Output1]);
	data->streams_.push_back(&data->isp_[Isp::Config]);

	for (auto stream : data->streams_) {
		ret = stream->dev()->open();
		if (ret)
			return ret;
	}

	/* TDN and Stitch nodes are special and not part of streams_. */
	data->isp_[Isp::Tdn].dev()->open();
	data->isp_[Isp::Stitch].dev()->open();

	/* Wire up the default IPA connections. The others get connected on start() */
	data->ipa_->setDelayedControls.connect((RPi::CameraData *)data, &RPi::CameraData::setDelayedControls);
	data->ipa_->setLensControls.connect((RPi::CameraData *)data, &RPi::CameraData::setLensControls);

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
		Camera::create(std::move(cameraData), id, streams);
	PipelineHandler::registerCamera(std::move(camera));

	LOG(RPI, Info) << "Registered camera " << id
		       << " to CFE device " << cfe->deviceNode()
		       << " and ISP device " << isp->deviceNode();

	return 0;
}

CameraConfiguration::Status PiSPCameraData::platformValidate(std::vector<StreamParams> &rawStreams,
							     std::vector<StreamParams> &outStreams) const
{
	CameraConfiguration::Status status = CameraConfiguration::Status::Valid;

	/* Can only output 1 RAW stream, or 2 YUV/RGB streams for now. */
	if (rawStreams.size() > 1 || outStreams.size() > 2) {
		LOG(RPI, Error) << "Invalid number of streams requested";
		return CameraConfiguration::Status::Invalid;
	}

	if (!rawStreams.empty()) {
		rawStreams[0].dev = cfe_[Cfe::Output0].dev();
		/*
		 * We cannot output CSI2 packed or non 16-bit output from the frontend,
		 * so signal the output as unpacked 16-bits in these cases.
		 */
		BayerFormat bayer = BayerFormat::fromPixelFormat(rawStreams[0].cfg->pixelFormat);
		if (bayer.packing == BayerFormat::Packing::CSI2 || bayer.bitDepth != 16) {
			bayer.packing = BayerFormat::Packing::None;
			bayer.bitDepth = 16;
			rawStreams[0].cfg->pixelFormat = bayer.toPixelFormat();
			status = CameraConfiguration::Status::Adjusted;
		}
	}

	/*
	 * For the two ISP outputs, the lower resolution must be routed from
	 * Output 1
	 *
	 * Index 0 contains the largest requested resolution.
	 */
	for (unsigned int i = 0; i < outStreams.size(); i++) {
		Size size;

		size.width = std::min(outStreams[i].cfg->size.width,
				      outStreams[0].cfg->size.width);
		size.height = std::min(outStreams[i].cfg->size.height,
				       outStreams[0].cfg->size.height);

		if (outStreams[i].cfg->size != size) {
			outStreams[i].cfg->size = size;
			status = CameraConfiguration::Status::Adjusted;
		}

		/*
		 * Output 1 must be for the smallest resolution. We will
		 * have that fixed up in the code above.
		 */
		outStreams[i].dev = isp_[i == 1 || outStreams.size() == 1 ? Isp::Output1 : Isp::Output0].dev();
	}

	return status;
}

int PiSPCameraData::platformPipelineConfigure(const std::unique_ptr<YamlObject> &root)
{
	config_ = {
		.minCfeBuffers = 2,
		.minTotalCfeBuffers = 4,
	};

	if (!root)
		return 0;

	std::optional<double> ver = (*root)["version"].get<double>();
	if (!ver || *ver != 1.0) {
		LOG(RPI, Error) << "Unexpected configuration file version reported";
		return -EINVAL;
	}

	std::optional<std::string> target = (*root)["target"].get<std::string>();
	if (!target || *target != "pisp") {
		LOG(RPI, Error) << "Unexpected target reported: expected \"pisp\", got "
				<< *target;
		return -EINVAL;
	}

	const YamlObject &phConfig = (*root)["pipeline_handler"];
	config_.minCfeBuffers =
		phConfig["min_cfe_buffers"].get<unsigned int>(config_.minCfeBuffers);
	config_.minTotalCfeBuffers =
		phConfig["min_total_cfe_buffers"].get<unsigned int>(config_.minTotalCfeBuffers);

	if (config_.minTotalCfeBuffers < config_.minCfeBuffers) {
		LOG(RPI, Error) << "Invalid configuration: min_total_cfe_buffers must be >= min_cfe_buffers";
		return -EINVAL;
	}

	if (config_.minTotalCfeBuffers < 1) {
		LOG(RPI, Error) << "Invalid configuration: min_total_cfe_buffers must be >= 1";
		return -EINVAL;
	}

	return 0;
}

std::unordered_map<uint32_t, uint32_t> deviceAdjustTable = {
	{ V4L2_PIX_FMT_RGBX32, V4L2_PIX_FMT_RGB24 },
	{ V4L2_PIX_FMT_XBGR32, V4L2_PIX_FMT_BGR24 },
	{ V4L2_PIX_FMT_RGBA32, V4L2_PIX_FMT_RGB24 },
	{ V4L2_PIX_FMT_ABGR32, V4L2_PIX_FMT_BGR24 }
};

bool PiSPCameraData::adjustDeviceFormat(V4L2DeviceFormat &format) const
{
	auto it = deviceAdjustTable.find(format.fourcc.fourcc());

	if (it != deviceAdjustTable.end()) {
		LOG(RPI, Debug) << "Swapping 32-bit for 24-bit format";
		format.fourcc = V4L2PixelFormat(it->second);
		format.planes[0].bpl = 4 * format.size.width;
		format.planes[0].size = format.planes[0].bpl * format.size.height;
		format.planesCount = 1;
		return true;
	}

	return false;
}

int PiSPCameraData::platformConfigure(const V4L2SubdeviceFormat &sensorFormat,
				      std::optional<BayerFormat::Packing> packing,
				      const std::optional<ColorSpace> &yuvColorSpace,
				      std::vector<StreamParams> &rawStreams,
				      std::vector<StreamParams> &outStreams)
{
	int ret;

	if (!packing)
		packing = BayerFormat::Packing::PISP1;

	/*
	 * The CFE Frontend output will always be 16-bits unpacked, so adjust the
	 * mbus code right at the start.
	 */
	V4L2SubdeviceFormat sensorFormatMod = sensorFormat;
	sensorFormatMod.mbus_code = mbusCodeUnpacked16(sensorFormatMod.mbus_code);

	V4L2VideoDevice *cfe = cfe_[Cfe::Output0].dev();
	V4L2DeviceFormat cfeFormat = RPi::PipelineHandlerBase::toV4L2DeviceFormat(cfe, sensorFormatMod, *packing);

	/* Compute the optimal stride for the FE output / BE input buffers. */
	computeOptimalStride(cfeFormat);

	ret = cfe->setFormat(&cfeFormat);
	if (ret)
		return ret;

	/* Set the TDN and Stitch node formats in case they are turned on. */
	isp_[Isp::Tdn].dev()->setFormat(&cfeFormat);
	isp_[Isp::Stitch].dev()->setFormat(&cfeFormat);

	/*
	 * See which streams are requested, and route the user
	 * StreamConfiguration appropriately.
	 */
	if (!rawStreams.empty()) {
		rawStreams[0].cfg->setStream(&cfe_[Cfe::Output0]);
		cfe_[Cfe::Output0].setFlags(StreamFlags::External);
	}

	ret = isp_[Isp::Input].dev()->setFormat(&cfeFormat);
	if (ret)
		return ret;

	LOG(RPI, Info) << "Sensor: " << sensor_->id()
		       << " - Selected sensor format: " << sensorFormat
		       << " - Selected CFE format: " << cfeFormat;

	unsigned int beEnables = 0;
	V4L2DeviceFormat format;

	/*
	 * First thing is to remove Isp::Output0 and Isp::Output1 from streams_
	 * as they may be unused depending on the configuration. Add them back
	 * only if needed.
	 */
	streams_.erase(std::remove_if(streams_.begin(), streams_.end(),
		       [this] (const RPi::Stream *s) { return s == &isp_[Isp::Output0] ||
							      s == &isp_[Isp::Output1]; }),
		       streams_.end());

	for (unsigned int i = 0; i < outStreams.size(); i++) {
		StreamConfiguration *cfg = outStreams[i].cfg;

		/*
		 * Output 1 must be for the smallest resolution. We will
		 * have that fixed up in the code above.
		 */
		RPi::Stream *stream;
		if (i == 1 || outStreams.size() == 1) {
			stream = &isp_[Isp::Output1];
			beEnables |= PISP_BE_RGB_ENABLE_OUTPUT1;
		} else {
			stream = &isp_[Isp::Output0];
			beEnables |= PISP_BE_RGB_ENABLE_OUTPUT0;
		}

		V4L2PixelFormat fourcc = stream->dev()->toV4L2PixelFormat(cfg->pixelFormat);
		bool needs32BitConversion = false;

		format = {};
		format.size = cfg->size;
		format.fourcc = fourcc;
		format.colorSpace = cfg->colorSpace;
		format.planesCount = 0;
		if (!fourcc.isValid()) {
			const std::vector<V4L2PixelFormat> &v4l2PixelFormats =
				V4L2PixelFormat::fromPixelFormat(cfg->pixelFormat);
			for (V4L2PixelFormat f : v4l2PixelFormats) {
				format.fourcc = f;
				if (adjustDeviceFormat(format)) {
					/* Assume that the adjusted format is supported! */
					needs32BitConversion = true;
					break;
				}
			}
		}

		/* Compute the optimal stride for the BE output buffers. */
		computeOptimalStride(format);

		LOG(RPI, Debug) << "Setting " << stream->name() << " to "
				<< format;

		fourcc = format.fourcc;
		ret = stream->dev()->setFormat(&format);
		if (ret)
			return -EINVAL;

		if (format.size != cfg->size || format.fourcc != fourcc) {
			LOG(RPI, Error)
				<< "Failed to set requested format on " << stream->name()
				<< ", returned " << format;
			return -EINVAL;
		}

		LOG(RPI, Debug)
			<< "Stream " << stream->name() << " has color space "
			<< ColorSpace::toString(cfg->colorSpace);

		unsigned int flags = StreamFlags::External;

		stream->clearFlags(StreamFlags::Needs32bitConv);
		if (needs32BitConversion)
			flags |= StreamFlags::Needs32bitConv | StreamFlags::RequiresMmap;

		cfg->setStream(stream);
		stream->setFlags(flags);
		streams_.push_back(stream);
	}

	{
		std::scoped_lock<BackEnd> l(*be_);
		pisp_be_global_config global;

		be_->GetGlobal(global);
		global.rgb_enables &= ~(PISP_BE_RGB_ENABLE_OUTPUT0 + PISP_BE_RGB_ENABLE_OUTPUT1);
		global.rgb_enables |= beEnables;
		be_->SetGlobal(global);
	}

	/* CFE statistics output format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_RPI_FE_STATS);
	ret = cfe_[Cfe::Stats].dev()->setFormat(&format);
	if (ret) {
		LOG(RPI, Error) << "Failed to set format on CFE stats stream: "
				<< format.toString();
		return ret;
	}

	/* CFE config format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_RPI_FE_CFG);
	ret = cfe_[Cfe::Config].dev()->setFormat(&format);
	if (ret) {
		LOG(RPI, Error) << "Failed to set format on CFE config stream: "
				<< format.toString();
		return ret;
	}

	/*
	 * Configure the Unicam embedded data output format only if the sensor
	 * supports it.
	 */
	if (sensorMetadata_) {
		V4L2SubdeviceFormat embeddedFormat;

		sensor_->device()->getFormat(1, &embeddedFormat);
		format = {};
		format.fourcc = V4L2PixelFormat(V4L2_META_FMT_SENSOR_DATA);
		format.planes[0].size = embeddedFormat.size.width * embeddedFormat.size.height;

		LOG(RPI, Debug) << "Setting embedded data format " << format.toString();
		ret = cfe_[Cfe::Embedded].dev()->setFormat(&format);
		if (ret) {
			LOG(RPI, Error) << "Failed to set format on Unicam embedded: "
					<< format;
			return ret;
		}
	}

	/* Set smallest selection the ISP will allow. */
	ispMinCropSize_ = Size(32, 32);

	if (!outStreams.empty()) {
		/* Adjust aspect ratio by providing crops on the input image. */
		Size size = cfeFormat.size.boundedToAspectRatio(outStreams[0].cfg->size);
		ispCrop_ = size.centeredTo(Rectangle(cfeFormat.size).center());
	}

	/* Reset back to the original sensor mbus code for entity configuration. */
	sensorFormatMod.mbus_code = sensorFormat.mbus_code;
	configureEntities(sensorFormatMod);
	configureCfe();
	configureBe(yuvColorSpace);

	platformIspCrop();

	return 0;
}

void PiSPCameraData::platformStart()
{
	cfe_[Cfe::Output0].dev()->bufferReady.connect(this, &PiSPCameraData::cfeBufferDequeue);
	cfe_[Cfe::Stats].dev()->bufferReady.connect(this, &PiSPCameraData::cfeBufferDequeue);
	cfe_[Cfe::Config].dev()->bufferReady.connect(this, &PiSPCameraData::cfeBufferDequeue);

	isp_[Isp::Input].dev()->bufferReady.connect(this, &PiSPCameraData::beInputDequeue);
	isp_[Isp::Config].dev()->bufferReady.connect(this, &PiSPCameraData::beOutputDequeue);
	isp_[Isp::Output0].dev()->bufferReady.connect(this, &PiSPCameraData::beOutputDequeue);
	isp_[Isp::Output1].dev()->bufferReady.connect(this, &PiSPCameraData::beOutputDequeue);
	ipa_->prepareIspComplete.connect(this, &PiSPCameraData::prepareIspComplete);
	ipa_->processStatsComplete.connect(this, &PiSPCameraData::processStatsComplete);

	cfeJobQueue_ = {};
	prepareCfe();
}

void PiSPCameraData::platformStop()
{
	cfe_[Cfe::Output0].dev()->bufferReady.disconnect();
	cfe_[Cfe::Stats].dev()->bufferReady.disconnect();
	cfe_[Cfe::Config].dev()->bufferReady.disconnect();

	isp_[Isp::Input].dev()->bufferReady.disconnect();
	isp_[Isp::Config].dev()->bufferReady.disconnect();
	isp_[Isp::Output0].dev()->bufferReady.disconnect();
	isp_[Isp::Output1].dev()->bufferReady.disconnect();

	ipa_->prepareIspComplete.disconnect();
	ipa_->processStatsComplete.disconnect();
}

void PiSPCameraData::platformFreeBuffers()
{
	tdnBuffers_.clear();
	stitchBuffers_.clear();
	cfeJobQueue_ = {};

	if (buffersAllocated_) {
		isp_[Isp::Tdn].releaseBuffers();
		isp_[Isp::Stitch].releaseBuffers();
	}
}

void PiSPCameraData::cfeBufferDequeue(FrameBuffer *buffer)
{
	RPi::Stream *stream = nullptr;
	int index;

	for (RPi::Stream &s : cfe_) {
		index = s.getBufferId(buffer);
		if (index) {
			stream = &s;
			break;
		}
	}

	/* If the last CFE job has completed, we need a new job entry in the queue. */
	if (cfeJobQueue_.empty() || cfeJobComplete())
		cfeJobQueue_.push({});

	CfeJob &job = cfeJobQueue_.back();

	/* The buffer must belong to one of our streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer dequeue"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	job.buffers[stream] = buffer;

	if (stream == &cfe_[Cfe::Output0]) {
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
		job.sensorControls = std::move(ctrl);
		job.delayContext = delayContext;
	} else if (stream == &cfe_[Cfe::Config]) {
		/* The config buffer can be re-queued back straight away. */
		handleStreamBuffer(buffer, &cfe_[Cfe::Config]);
		prepareCfe();
	}

	handleState();
}

void PiSPCameraData::beInputDequeue(FrameBuffer *buffer)
{
	LOG(RPI, Debug) << "Stream ISP Input buffer complete"
			<< ", buffer id " << cfe_[Cfe::Output0].getBufferId(buffer)
			<< ", timestamp: " << buffer->metadata().timestamp;

	/* The ISP input buffer gets re-queued into CFE. */
	handleStreamBuffer(buffer, &cfe_[Cfe::Output0]);
	handleState();
}

void PiSPCameraData::beOutputDequeue(FrameBuffer *buffer)
{
	RPi::Stream *stream = nullptr;
	int index;

	for (RPi::Stream &s : isp_) {
		index = s.getBufferId(buffer);
		if (index) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our ISP output streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer complete"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	/* Convert 24bpp outputs to 32bpp outputs where necessary. */
	if (stream->getFlags() & StreamFlags::Needs32bitConv) {
		unsigned int stride = stream->configuration().stride;
		unsigned int width = stream->configuration().size.width;
		unsigned int height = stream->configuration().size.height;

		const RPi::BufferObject &b = stream->getBuffer(index);

		ASSERT(b.mapped);
		void *mem = b.mapped->planes()[0].data();
		do32BitConversion(mem, width, height, stride);
	}

	handleStreamBuffer(buffer, stream);

	/*
	 * Increment the number of ISP outputs generated.
	 * This is needed to track dropped frames.
	 */
	ispOutputCount_++;
	handleState();
}

void PiSPCameraData::processStatsComplete(const ipa::RPi::BufferIds &buffers)
{
	handleStreamBuffer(cfe_[Cfe::Stats].getBuffers().at(buffers.stats & RPi::MaskID).buffer,
			   &cfe_[Cfe::Stats]);
}

void PiSPCameraData::prepareIspComplete(const ipa::RPi::BufferIds &buffers)
{
	unsigned int embeddedId = buffers.embedded & RPi::MaskID;
	unsigned int bayer = buffers.bayer & RPi::MaskID;
	FrameBuffer *buffer;

	if (sensorMetadata_ && embeddedId) {
		buffer = cfe_[Cfe::Embedded].getBuffers().at(buffers.embedded & RPi::MaskID).buffer;
		handleStreamBuffer(buffer, &cfe_[Cfe::Embedded]);
	}

	runBackend(bayer & RPi::MaskID);

	/* Add to the Request metadata buffer what the IPA has provided. */
	Request *request = requestQueue_.front();
	ControlList metadata;

	ipa_->reportMetadata(request->sequence(), &metadata);
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

	state_ = State::IpaComplete;

	handleState();
}

int PiSPCameraData::configureCfe()
{
	V4L2DeviceFormat cfeFormat;
	cfe_[Cfe::Output0].dev()->getFormat(&cfeFormat);

	std::scoped_lock<FrontEnd> l(*fe_);

	pisp_fe_global_config global;
	fe_->GetGlobal(global);
	global.enables &= ~PISP_FE_ENABLE_COMPRESS0;

	global.enables |= PISP_FE_ENABLE_OUTPUT0;
	global.bayer_order = toPiSPBayerOrder(cfeFormat.fourcc);

	pisp_image_format_config image = toPiSPImageFormat(cfeFormat);
	pisp_fe_input_config input = {};

	input.streaming = 1;
	input.format = image;
	input.format.format = PISP_IMAGE_FORMAT_BPS_16;

	if (PISP_IMAGE_FORMAT_compressed(image.format)) {
		pisp_compress_config compress;
		compress.offset = DefaultCompressionOffset;
		compress.mode =
			(image.format & PISP_IMAGE_FORMAT_COMPRESSION_MASK) / PISP_IMAGE_FORMAT_COMPRESSION_MODE_1;
		global.enables |= PISP_FE_ENABLE_COMPRESS0;
		fe_->SetCompress(0, compress);
	}

	fe_->SetGlobal(global);
	fe_->SetInput(input);
	fe_->SetOutputFormat(0, image);

	return 0;
}

int PiSPCameraData::configureBe(const std::optional<ColorSpace> &yuvColorSpace)
{
	std::scoped_lock<BackEnd> l(*be_);
	pisp_image_format_config inputFormat;
	V4L2DeviceFormat cfeFormat;

	isp_[Isp::Input].dev()->getFormat(&cfeFormat);
	inputFormat = toPiSPImageFormat(cfeFormat);

	pisp_be_global_config global;
	be_->GetGlobal(global);
	global.bayer_enables &= ~(PISP_BE_BAYER_ENABLE_DECOMPRESS +
				  PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS + PISP_BE_BAYER_ENABLE_TDN_COMPRESS +
				  PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS + PISP_BE_BAYER_ENABLE_STITCH_COMPRESS);
	global.rgb_enables &= ~(PISP_BE_RGB_ENABLE_RESAMPLE0 + PISP_BE_RGB_ENABLE_RESAMPLE1 +
				PISP_BE_RGB_ENABLE_DOWNSCALE0 + PISP_BE_RGB_ENABLE_DOWNSCALE1 +
				PISP_BE_RGB_ENABLE_CSC0 + PISP_BE_RGB_ENABLE_CSC1);

	global.bayer_enables |= PISP_BE_BAYER_ENABLE_INPUT;
	global.bayer_order = toPiSPBayerOrder(cfeFormat.fourcc);

	V4L2DeviceFormat ispFormat0 = {}, ispFormat1 = {};
	pisp_be_output_format_config outputFormat0 = {}, outputFormat1 = {};

	isp_[Isp::Output0].dev()->getFormat(&ispFormat0);
	outputFormat0.image = toPiSPImageFormat(ispFormat0);

	ispOutputTotal_ = 1; // Config buffer
	if (PISP_IMAGE_FORMAT_compressed(inputFormat.format)) {
		pisp_decompress_config decompress;
		decompress.offset = DefaultCompressionOffset;
		decompress.mode =
			(inputFormat.format & PISP_IMAGE_FORMAT_COMPRESSION_MASK) / PISP_IMAGE_FORMAT_COMPRESSION_MODE_1;
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_DECOMPRESS;
		be_->SetDecompress(decompress);
	}

	pisp_be_ccm_config csc;
	if (calculateCscConfiguration(ispFormat0, csc)) {
		global.rgb_enables |= PISP_BE_RGB_ENABLE_CSC0;
		be_->SetCsc(0, csc);
	}

	if (global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT0) {
		BackEnd::SmartResize resize = {};
		resize.width = ispFormat0.size.width;
		resize.height = ispFormat0.size.height;
		be_->SetSmartResize(0, resize);

		setupOutputClipping(ispFormat0, outputFormat0);

		ispOutputTotal_++;
	}

	if (global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1) {
		isp_[Isp::Output1].dev()->getFormat(&ispFormat1);
		outputFormat1.image = toPiSPImageFormat(ispFormat1);

		if (calculateCscConfiguration(ispFormat1, csc)) {
			global.rgb_enables |= PISP_BE_RGB_ENABLE_CSC1;
			be_->SetCsc(1, csc);
		}

		BackEnd::SmartResize resize = {};
		resize.width = ispFormat1.size.width;
		resize.height = ispFormat1.size.height;
		be_->SetSmartResize(1, resize);

		setupOutputClipping(ispFormat1, outputFormat1);

		ispOutputTotal_++;
	}

	/* Setup the TDN I/O blocks in case TDN gets turned on later. */
	V4L2DeviceFormat tdnV4L2Format;
	isp_[Isp::Tdn].dev()->getFormat(&tdnV4L2Format);
	pisp_image_format_config tdnFormat = toPiSPImageFormat(tdnV4L2Format);
	be_->SetTdnOutputFormat(tdnFormat);
	be_->SetTdnInputFormat(tdnFormat);

	if (PISP_IMAGE_FORMAT_compressed(tdnFormat.format)) {
		pisp_decompress_config tdnDecompress;
		pisp_compress_config tdnCompress;

		tdnDecompress.offset = tdnCompress.offset = DefaultCompressionOffset;
		tdnDecompress.mode = tdnCompress.mode = DefaultCompressionMode;
		be_->SetTdnDecompress(tdnDecompress);
		be_->SetTdnCompress(tdnCompress);
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS + PISP_BE_BAYER_ENABLE_TDN_COMPRESS;
	}

	/* Likewise for the Stitch block. */
	V4L2DeviceFormat stitchV4L2Format;
	isp_[Isp::Stitch].dev()->getFormat(&stitchV4L2Format);
	pisp_image_format_config stitchFormat = toPiSPImageFormat(stitchV4L2Format);
	be_->SetStitchOutputFormat(stitchFormat);
	be_->SetStitchInputFormat(stitchFormat);

	if (PISP_IMAGE_FORMAT_compressed(stitchFormat.format)) {
		pisp_decompress_config stitchDecompress;
		pisp_compress_config stitchCompress;

		stitchDecompress.offset = stitchCompress.offset = DefaultCompressionOffset;
		stitchDecompress.mode = stitchCompress.mode = DefaultCompressionMode;
		be_->SetStitchDecompress(stitchDecompress);
		be_->SetStitchCompress(stitchCompress);
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS + PISP_BE_BAYER_ENABLE_STITCH_COMPRESS;
	}

	/*
	 * For the bit of the pipeline where we go temporarily into YCbCr, we'll use the
	 * same flavour of YCbCr as dictated by the headline colour space. But there's
	 * no benefit from compressing and shifting the range, so we'll stick with the
	 * full range version of whatever that colour space is.
	 */
	if (yuvColorSpace) {
		pisp_be_ccm_config ccm;
		if (yuvColorSpace == ColorSpace::Sycc) {
			libpisp::initialise_ycbcr(ccm, "jpeg");
			be_->SetYcbcr(ccm);
			libpisp::initialise_ycbcr_inverse(ccm, "jpeg");
			be_->SetYcbcrInverse(ccm);
		} else if (yuvColorSpace == ColorSpace::Smpte170m) {
			/* We want the full range version of smpte170m, aka. jpeg */
			libpisp::initialise_ycbcr(ccm, "jpeg");
			be_->SetYcbcr(ccm);
			libpisp::initialise_ycbcr_inverse(ccm, "jpeg");
			be_->SetYcbcrInverse(ccm);
		} else if (yuvColorSpace == ColorSpace::Rec709) {
			libpisp::initialise_ycbcr(ccm, "rec709_full");
			be_->SetYcbcr(ccm);
			libpisp::initialise_ycbcr_inverse(ccm, "rec709_full");
			be_->SetYcbcrInverse(ccm);
		} else {
			/* Validation should have ensured this can't happen. */
			LOG(RPI, Error)
				<< "Invalid colour space " << ColorSpace::toString(yuvColorSpace);
			ASSERT(0);
		}
	} else {
		/* Again, validation should have prevented this. */
		LOG(RPI, Error) << "No YUV colour space";
		ASSERT(0);
	}

	be_->SetGlobal(global);
	be_->SetInputFormat(inputFormat);
	be_->SetOutputFormat(0, outputFormat0);
	be_->SetOutputFormat(1, outputFormat1);

	return 0;
}

void PiSPCameraData::platformIspCrop()
{
	pisp_be_crop_config beCrop = {
		static_cast<uint16_t>(ispCrop_.x),
		static_cast<uint16_t>(ispCrop_.y),
		static_cast<uint16_t>(ispCrop_.width),
		static_cast<uint16_t>(ispCrop_.height)
	};

	std::scoped_lock<BackEnd> l(*be_);
	be_->SetCrop(beCrop);
}

int PiSPCameraData::platformInitIpa(ipa::RPi::InitParams &params)
{
	params.fe = fe_.getFD();
	params.be = be_.getFD();
	return 0;
}

int PiSPCameraData::configureEntities(V4L2SubdeviceFormat &sensorFormat)
{
	int ret = 0;

	constexpr unsigned int csiFeLinkPad = 4;

	const MediaEntity *csi2 = csi2Subdev_->entity();
	const MediaEntity *fe = feSubdev_->entity();

	for (MediaLink *link : csi2->pads()[csiFeLinkPad]->links()) {
		if (link->sink()->entity()->name() == "rp1-cfe-csi2_ch0")
			link->setEnabled(false);
		else if (link->sink()->entity()->name() == "pisp-fe")
			link->setEnabled(true);
	}

	fe->pads()[1]->links()[0]->setEnabled(true);
	fe->pads()[2]->links()[0]->setEnabled(true);
	fe->pads()[3]->links()[0]->setEnabled(false);
	fe->pads()[4]->links()[0]->setEnabled(true);

	ret = csi2Subdev_->setFormat(0, &sensorFormat);
	if (ret)
		return ret;

	V4L2SubdeviceFormat feFormat = sensorFormat;
	feFormat.mbus_code = mbusCodeUnpacked16(sensorFormat.mbus_code);
	ret = feSubdev_->setFormat(0, &feFormat);
	if (ret)
		return ret;

	ret = csi2Subdev_->setFormat(csiFeLinkPad, &feFormat);
	if (ret)
		return ret;

	V4L2DeviceFormat feOutputFormat;
	cfe_[Cfe::Output0].dev()->getFormat(&feOutputFormat);
	BayerFormat feOutputBayer = BayerFormat::fromV4L2PixelFormat(feOutputFormat.fourcc);

	bool valid;
	feFormat.mbus_code = feOutputBayer.toMbusCode(valid);
	ret = feSubdev_->setFormat(2, &feFormat);

	return ret;
}

void PiSPCameraData::prepareCfe()
{
	/* Fetch an unused config buffer from the stream .*/
	const RPi::BufferObject &config = cfe_[Cfe::Config].getBuffer(0);

	ASSERT(config.mapped);

	Span<uint8_t> configBuffer = config.mapped->planes()[0];
	fe_->Prepare(reinterpret_cast<pisp_fe_config *>(configBuffer.data()));

	config.buffer->_d()->metadata().planes()[0].bytesused = sizeof(pisp_fe_config);
	cfe_[Cfe::Config].queueBuffer(config.buffer);
}

void PiSPCameraData::runBackend(uint32_t bufferId)
{
	ispOutputCount_ = 0;

	FrameBuffer *buffer = cfe_[Cfe::Output0].getBuffers().at(bufferId).buffer;

	LOG(RPI, Debug) << "Input re-queue to ISP, buffer id " << bufferId
			<< ", timestamp: " << buffer->metadata().timestamp;

	isp_[Isp::Input].queueBuffer(buffer);

	/* Fetch an unused config buffer from the stream .*/
	const RPi::BufferObject &config = isp_[Isp::Config].getBuffer(0);

	ASSERT(config.mapped);

	Span<uint8_t> configBuffer = config.mapped->planes()[0];
	be_->Prepare(reinterpret_cast<pisp_be_tiles_config *>(configBuffer.data()));

	config.buffer->_d()->metadata().planes()[0].bytesused = sizeof(pisp_be_tiles_config);
	isp_[Isp::Config].queueBuffer(config.buffer);
}

void PiSPCameraData::tryRunPipeline()
{
	/* If any of our request or buffer queues are empty, we cannot proceed. */
	if (state_ != State::Idle || requestQueue_.empty() || !cfeJobComplete())
		return;

	CfeJob &job = cfeJobQueue_.front();

	/* Take the first request from the queue and action the IPA. */
	Request *request = requestQueue_.front();

	/* See if a new ScalerCrop value needs to be applied. */
	calculateScalerCrop(request->controls());

	/*
	 * Clear the request metadata and fill it with some initial non-IPA
	 * related controls. We clear it first because the request metadata
	 * may have been populated if we have dropped the previous frame.
	 */
	request->metadata().clear();
	fillRequestMetadata(job.sensorControls, request);

	/*
	 * Process all the user controls by the IPA. Once this is complete, we
	 * queue the ISP output buffer listed in the request to start the HW
	 * pipeline.
	 */
	//ipa_->signalQueueRequest(request->controls());

	/* Set our state to say the pipeline is active. */
	state_ = State::Busy;

	unsigned int bayerId = cfe_[Cfe::Output0].getBufferId(job.buffers[&cfe_[Cfe::Output0]]);
	unsigned int statsId = cfe_[Cfe::Stats].getBufferId(job.buffers[&cfe_[Cfe::Stats]]);
	ASSERT(bayerId && statsId);

	std::stringstream ss;
	ss << "Signalling IPA processStats and prepareIsp:"
	   << " Bayer buffer id: " << bayerId
	   << " Stats buffer id: " << statsId;

	ipa::RPi::PrepareParams prepare;
	prepare.buffers.bayer = RPi::MaskBayerData | bayerId;
	prepare.buffers.stats = RPi::MaskStats | statsId;
	prepare.ipaContext = requestQueue_.front()->sequence();
	prepare.delayContext = job.delayContext;
	prepare.sensorControls = std::move(job.sensorControls);
	prepare.requestControls = request->controls();

	if (sensorMetadata_) {
		unsigned int embeddedId = cfe_[Cfe::Embedded].getBufferId(job.buffers[&cfe_[Cfe::Embedded]]);

		ASSERT(embeddedId);
		prepare.buffers.embedded = RPi::MaskEmbeddedData | embeddedId;
		ss << " Embedded buffer id: " << embeddedId;
	}

	LOG(RPI, Debug) << ss.str();

	cfeJobQueue_.pop();
	ipa_->signalPrepareIsp(prepare);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerPiSP)

} /* namespace libcamera */
