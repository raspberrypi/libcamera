/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Pipeline Handler for ARM's Mali-C55 ISP
 */

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <variant>

#include <linux/mali-c55-config.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/property_ids.h>
#include <libcamera/stream.h>

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/mali-c55_ipa_interface.h>
#include <libcamera/ipa/mali-c55_ipa_proxy.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "rzg2l-cru.h"

namespace {

static constexpr unsigned int kMaliC55BufferCount = 8;

bool isFormatRaw(const libcamera::PixelFormat &pixFmt)
{
	return libcamera::PixelFormatInfo::info(pixFmt).colourEncoding ==
	       libcamera::PixelFormatInfo::ColourEncodingRAW;
}

} /* namespace */

namespace libcamera {

LOG_DEFINE_CATEGORY(MaliC55)

const std::map<libcamera::PixelFormat, unsigned int> maliC55FmtToCode = {
	/* \todo Support all formats supported by the driver in libcamera. */

	{ formats::RGB565, MEDIA_BUS_FMT_RGB121212_1X36 },
	{ formats::RGB888, MEDIA_BUS_FMT_RGB121212_1X36 },
	{ formats::YUYV, MEDIA_BUS_FMT_YUV10_1X30 },
	{ formats::UYVY, MEDIA_BUS_FMT_YUV10_1X30 },
	{ formats::R8, MEDIA_BUS_FMT_YUV10_1X30 },
	{ formats::NV12, MEDIA_BUS_FMT_YUV10_1X30 },
	{ formats::NV21, MEDIA_BUS_FMT_YUV10_1X30 },

	/* RAW formats, FR pipe only. */
	{ formats::SGBRG16, MEDIA_BUS_FMT_SGBRG16_1X16 },
	{ formats::SRGGB16, MEDIA_BUS_FMT_SRGGB16_1X16 },
	{ formats::SBGGR16, MEDIA_BUS_FMT_SBGGR16_1X16 },
	{ formats::SGRBG16, MEDIA_BUS_FMT_SGRBG16_1X16 },
};

constexpr Size kMaliC55MinInputSize = { 640, 480 };
constexpr Size kMaliC55MinSize = { 128, 128 };
constexpr Size kMaliC55MaxSize = { 8192, 8192 };
constexpr unsigned int kMaliC55ISPInternalFormat = MEDIA_BUS_FMT_RGB121212_1X36;

struct MaliC55FrameInfo {
	Request *request;

	FrameBuffer *paramBuffer;
	FrameBuffer *statBuffer;
	FrameBuffer *rawBuffer;

	bool paramsDone;
	bool statsDone;
};

class MaliC55CameraData : public Camera::Private
{
public:
	struct Tpg {
		std::vector<Size> sizes(unsigned int mbusCode) const;

		Size resolution_;
		std::unique_ptr<V4L2Subdevice> sd_;
	};

	struct Inline {
		std::unique_ptr<V4L2Subdevice> csi2_;
		std::unique_ptr<CameraSensor> sensor_;
	};

	struct Memory {
		std::unique_ptr<RZG2LCRU> cru_;
	};

	using CameraType = std::variant<Tpg, Inline, Memory>;

	MaliC55CameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	int loadIPA();

	Tpg *initTpg(MediaEntity *entity);
	Inline *initInline(MediaEntity *entity);
	Memory *initMemory(MediaDevice *cruMedia);

	std::vector<Size> sizes(unsigned int mbusCode) const
	{
		return std::visit(utils::overloaded{
			[&](const Tpg &tpg) -> std::vector<Size> {
				return tpg.sizes(mbusCode);
			},
			[&](const Inline &in) -> std::vector<Size> {
				return in.sensor_->sizes(mbusCode);
			},
			[&](const Memory &mem) -> std::vector<Size> {
				return mem.cru_->sizes();
			},
		}, input_);
	}

	V4L2Subdevice *subdev() const
	{
		return std::visit(utils::overloaded{
			[&](const Tpg &tpg) -> V4L2Subdevice * {
				return tpg.sd_.get();
			},
			[&](const Inline &in) -> V4L2Subdevice * {
				return in.sensor_->device();
			},
			[&](const Memory &mem) -> V4L2Subdevice * {
				return mem.cru_->sensor()->device();
			},
		}, input_);
	}

	CameraSensor *sensor() const
	{
		return std::visit(utils::overloaded{
			[&](auto &) -> CameraSensor * {
				ASSERT(false);
				return nullptr;
			},
			[&](const Inline &in) -> CameraSensor * {
				return in.sensor_.get();
			},
			[&](const Memory &mem) -> CameraSensor * {
				return mem.cru_->sensor();
			},
		}, input_);
	}

	Size resolution() const
	{
		return std::visit(utils::overloaded{
			[&](const Tpg &tpg) -> Size {
				return tpg.resolution_;
			},
			[&](const Inline &in) -> Size {
				return in.sensor_->resolution();
			},
			[&](const Memory &mem) -> Size {
				return mem.cru_->resolution();
			},
		}, input_);
	}

	int pixfmtToMbusCode(const PixelFormat &pixFmt) const;
	const PixelFormat &bestRawFormat() const;

	void updateControls(const ControlInfoMap &ipaControls);

	PixelFormat adjustRawFormat(const PixelFormat &pixFmt) const;
	Size adjustRawSizes(const PixelFormat &pixFmt, const Size &rawSize) const;

	Stream frStream_;
	Stream dsStream_;

	std::unique_ptr<ipa::mali_c55::IPAProxyMaliC55> ipa_;
	std::vector<IPABuffer> ipaStatBuffers_;
	std::vector<IPABuffer> ipaParamBuffers_;

	std::unique_ptr<DelayedControls> delayedCtrls_;

	CameraType input_;

private:
	void setSensorControls(const ControlList &sensorControls);
	std::string id_;
};

MaliC55CameraData::Tpg *MaliC55CameraData::initTpg(MediaEntity *entity)
{
	Tpg tpg;

	tpg.sd_ = std::make_unique<V4L2Subdevice>(entity);
	int ret = tpg.sd_->open();
	if (ret) {
		LOG(MaliC55, Error) << "Failed to open TPG subdevice";
		return nullptr;
	}

	/* Replicate the CameraSensor implementation for TPG. */
	V4L2Subdevice::Formats formats = tpg.sd_->formats(0);
	if (formats.empty())
		return nullptr;

	std::vector<Size> tpgSizes;

	for (const auto &format : formats) {
		const std::vector<SizeRange> &ranges = format.second;
		std::transform(ranges.begin(), ranges.end(), std::back_inserter(tpgSizes),
			       [](const SizeRange &range) { return range.max; });
	}

	tpg.resolution_ = tpgSizes.back();

	return &input_.emplace<Tpg>(std::move(tpg));
}

MaliC55CameraData::Inline *MaliC55CameraData::initInline(MediaEntity *sensor)
{
	Inline in;

	/* Register a CameraSensor and create an entity for the CSI-2 receiver. */
	in.sensor_ = CameraSensorFactoryBase::create(sensor);
	if (!in.sensor_)
		return nullptr;

	const MediaPad *sourcePad = sensor->getPadByIndex(0);
	MediaEntity *csiEntity = sourcePad->links()[0]->sink()->entity();

	in.csi2_ = std::make_unique<V4L2Subdevice>(csiEntity);
	int ret = in.csi2_->open();
	if (ret) {
		LOG(MaliC55, Error) << "Failed to open CSI-2 subdevice";
		return nullptr;
	}

	return &input_.emplace<Inline>(std::move(in));
}

MaliC55CameraData::Memory *MaliC55CameraData::initMemory(MediaDevice *cruMedia)
{
	Memory mem;

	mem.cru_ = std::make_unique<RZG2LCRU>();

	int ret = mem.cru_->init(cruMedia);
	if (ret)
		return nullptr;

	return &input_.emplace<Memory>(std::move(mem));
}

std::vector<Size> MaliC55CameraData::Tpg::sizes(unsigned int mbusCode) const
{
	V4L2Subdevice::Formats formats = sd_->formats(0);
	if (formats.empty())
		return {};

	std::vector<Size> sizes;
	const auto &format = formats.find(mbusCode);
	if (format == formats.end())
		return {};

	const std::vector<SizeRange> &ranges = format->second;
	std::transform(ranges.begin(), ranges.end(), std::back_inserter(sizes),
		       [](const SizeRange &range) { return range.max; });

	std::sort(sizes.begin(), sizes.end());

	return sizes;
}

void MaliC55CameraData::setSensorControls(const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
}

/*
 * The Mali C55 ISP can only produce 16-bit RAW output in bypass modes, but the
 * sensors connected to it might produce 8/10/12/16 bits. We simply search the
 * sensor's supported formats for the one with a matching bayer order and the
 * greatest bitdepth.
 */
int MaliC55CameraData::pixfmtToMbusCode(const PixelFormat &pixFmt) const
{
	auto it = maliC55FmtToCode.find(pixFmt);
	if (it == maliC55FmtToCode.end())
		return -EINVAL;

	BayerFormat bayerFormat = BayerFormat::fromMbusCode(it->second);
	if (!bayerFormat.isValid())
		return -EINVAL;

	V4L2Subdevice::Formats formats = subdev()->formats(0);
	unsigned int sensorMbusCode = 0;
	unsigned int bitDepth = 0;

	for (const auto &[code, sizes] : formats) {
		BayerFormat sdBayerFormat = BayerFormat::fromMbusCode(code);
		if (!sdBayerFormat.isValid())
			continue;

		if (sdBayerFormat.order != bayerFormat.order)
			continue;

		if (sdBayerFormat.bitDepth > bitDepth) {
			bitDepth = sdBayerFormat.bitDepth;
			sensorMbusCode = code;
		}
	}

	if (!sensorMbusCode)
		return -EINVAL;

	return sensorMbusCode;
}

/*
 * Find a RAW PixelFormat supported by both the ISP and the sensor.
 *
 * The situation is mildly complicated by the fact that we expect the sensor to
 * output something like RAW8/10/12/16, but the ISP can only accept as input
 * RAW20 and can only produce as output RAW16. The one constant in that is the
 * bayer order of the data, so we'll simply check that the sensor produces a
 * format with a bayer order that matches that of one of the formats we support,
 * and select that.
 */
const PixelFormat &MaliC55CameraData::bestRawFormat() const
{
	static const PixelFormat invalidPixFmt = {};

	for (const auto &fmt : subdev()->formats(0)) {
		BayerFormat sensorBayer = BayerFormat::fromMbusCode(fmt.first);

		if (!sensorBayer.isValid())
			continue;

		for (const auto &[pixFmt, rawCode] : maliC55FmtToCode) {
			if (!isFormatRaw(pixFmt))
				continue;

			BayerFormat bayer = BayerFormat::fromMbusCode(rawCode);
			if (bayer.order == sensorBayer.order)
				return pixFmt;
		}
	}

	LOG(MaliC55, Error) << "Sensor doesn't provide a compatible format";
	return invalidPixFmt;
}

void MaliC55CameraData::updateControls(const ControlInfoMap &ipaControls)
{
	if (std::holds_alternative<Tpg>(input_))
		return;

	IPACameraSensorInfo sensorInfo;
	int ret = sensor()->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(MaliC55, Error) << "Failed to retrieve sensor info";
		return;
	}

	ControlInfoMap::Map controls;
	Rectangle ispMinCrop{ 0, 0, 640, 480 };
	controls[&controls::ScalerCrop] =
		ControlInfo(ispMinCrop, sensorInfo.analogCrop,
			    sensorInfo.analogCrop);

	for (const auto &c : ipaControls)
		controls.emplace(c.first, c.second);

	controlInfo_ = ControlInfoMap(std::move(controls), controls::controls);
}

/*
 * Make sure the provided raw pixel format is supported and adjust it to
 * one of the supported ones if it's not.
 */
PixelFormat MaliC55CameraData::adjustRawFormat(const PixelFormat &rawFmt) const
{
	/* Make sure the RAW mbus code is supported by the image source. */
	int rawCode = pixfmtToMbusCode(rawFmt);
	if (rawCode < 0)
		return bestRawFormat();

	const auto rawSizes = sizes(rawCode);
	if (rawSizes.empty())
		return bestRawFormat();

	return rawFmt;
}

Size MaliC55CameraData::adjustRawSizes(const PixelFormat &rawFmt, const Size &size) const
{
	/* Expand the RAW size to the minimum ISP input size. */
	Size rawSize = size.expandedTo(kMaliC55MinInputSize);

	/* Check if the size is natively supported. */
	int rawCode = pixfmtToMbusCode(rawFmt);
	if (rawCode < 0)
		return {};

	const auto rawSizes = sizes(rawCode);
	auto sizeIt = std::find(rawSizes.begin(), rawSizes.end(), rawSize);
	if (sizeIt != rawSizes.end())
		return rawSize;

	/* Or adjust it to the closest supported size. */
	uint16_t distance = std::numeric_limits<uint16_t>::max();
	Size bestSize;
	for (const Size &sz : rawSizes) {
		uint16_t dist = std::abs(static_cast<int>(rawSize.width) -
					 static_cast<int>(sz.width)) +
				std::abs(static_cast<int>(rawSize.height) -
					 static_cast<int>(sz.height));
		if (dist < distance) {
			dist = distance;
			bestSize = sz;
		}
	}

	return bestSize;
}

int MaliC55CameraData::loadIPA()
{
	int ret;

	/* Do not initialize IPA for TPG. */
	if (std::holds_alternative<Tpg>(input_))
		return 0;

	ipa_ = pipe()->createIPA<ipa::mali_c55::IPAProxyMaliC55>(1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->setSensorControls.connect(this, &MaliC55CameraData::setSensorControls);

	std::string ipaTuningFile = ipa_->configurationFile(sensor()->model() + ".yaml",
							    "uncalibrated.yaml");

	/* We need to inform the IPA of the sensor configuration */
	ipa::mali_c55::IPAConfigInfo ipaConfig{};

	ret = sensor()->sensorInfo(&ipaConfig.sensorInfo);
	if (ret)
		return ret;

	ipaConfig.sensorControls = sensor()->controls();

	ControlInfoMap ipaControls;
	ret = ipa_->init({ ipaTuningFile, sensor()->model() }, ipaConfig,
			 &ipaControls);
	if (ret) {
		LOG(MaliC55, Error) << "Failed to initialise the Mali-C55 IPA";
		return ret;
	}

	updateControls(ipaControls);

	return 0;
}

class MaliC55CameraConfiguration : public CameraConfiguration
{
public:
	MaliC55CameraConfiguration(MaliC55CameraData *data)
		: CameraConfiguration(), data_(data)
	{
	}

	Status validate() override;
	const Transform &combinedTransform() { return combinedTransform_; }

	V4L2SubdeviceFormat sensorFormat_;

private:
	static constexpr unsigned int kMaxStreams = 2;

	const MaliC55CameraData *data_;
	Transform combinedTransform_;
};

CameraConfiguration::Status MaliC55CameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/*
	 * The TPG doesn't support flips, so we only need to calculate a
	 * transform if we have a sensor.
	 */
	if (std::holds_alternative<MaliC55CameraData::Tpg>(data_->input_)) {
		combinedTransform_ = Transform::Rot0;
	} else {
		Orientation requestedOrientation = orientation;
		combinedTransform_ = data_->sensor()->computeTransform(&orientation);
		if (orientation != requestedOrientation)
			status = Adjusted;
	}

	/* Only 2 streams available. */
	if (config_.size() > kMaxStreams) {
		config_.resize(kMaxStreams);
		status = Adjusted;
	}

	bool frPipeAvailable = true;
	StreamConfiguration *rawConfig = nullptr;
	for (StreamConfiguration &config : config_) {
		if (!isFormatRaw(config.pixelFormat))
			continue;

		if (rawConfig) {
			LOG(MaliC55, Error)
				<< "Only a single RAW stream is supported";
			return Invalid;
		}

		rawConfig = &config;
	}

	/*
	 * The C55 can not upscale. Limit the configuration to the ISP
	 * capabilities and the sensor resolution.
	 */
	Size maxSize = kMaliC55MaxSize.boundedTo(data_->resolution());
	if (rawConfig) {
		/*
		 * \todo Take into account the Bayer components ordering once
		 * we support rotations.
		 */
		PixelFormat rawFormat =
			data_->adjustRawFormat(rawConfig->pixelFormat);

		if (!rawFormat.isValid())
			return Invalid;

		if (rawFormat != rawConfig->pixelFormat) {
			LOG(MaliC55, Debug)
				<< "RAW format adjusted to " << rawFormat;
			rawConfig->pixelFormat = rawFormat;
			status = Adjusted;
		}

		Size rawSize =
			data_->adjustRawSizes(rawFormat, rawConfig->size);
		if (rawSize != rawConfig->size) {
			LOG(MaliC55, Debug)
				<< "RAW sizes adjusted to " << rawSize;
			rawConfig->size = rawSize;
			status = Adjusted;
		}

		maxSize = rawSize;

		const PixelFormatInfo &info = PixelFormatInfo::info(rawConfig->pixelFormat);
		rawConfig->stride = info.stride(rawConfig->size.width, 0, 4);
		rawConfig->frameSize = info.frameSize(rawConfig->size, 4);

		rawConfig->setStream(const_cast<Stream *>(&data_->frStream_));
		frPipeAvailable = false;
	}

	/*
	 * Adjust processed streams.
	 *
	 * Compute the minimum sensor size to be later used to select the
	 * sensor configuration.
	 */
	Size minSensorSize = kMaliC55MinInputSize;
	for (StreamConfiguration &config : config_) {
		if (isFormatRaw(config.pixelFormat))
			continue;

		/* Adjust format and size for processed streams. */
		const auto it = maliC55FmtToCode.find(config.pixelFormat);
		if (it == maliC55FmtToCode.end()) {
			LOG(MaliC55, Debug)
				<< "Format adjusted to " << formats::RGB565;
			config.pixelFormat = formats::RGB565;
			status = Adjusted;
		}

		Size size = std::clamp(config.size, kMaliC55MinSize, maxSize);
		if (size != config.size) {
			LOG(MaliC55, Debug)
				<< "Size adjusted to " << size;
			config.size = size;
			status = Adjusted;
		}

		if (minSensorSize < size)
			minSensorSize = size;

		if (frPipeAvailable) {
			config.setStream(const_cast<Stream *>(&data_->frStream_));
			frPipeAvailable = false;
		} else {
			config.setStream(const_cast<Stream *>(&data_->dsStream_));
		}
	}

	/* Compute the sensor format. */

	/* If there's a RAW config, sensor configuration follows it. */
	if (rawConfig) {
		sensorFormat_.code = data_->pixfmtToMbusCode(rawConfig->pixelFormat);
		sensorFormat_.size = rawConfig->size.expandedTo(minSensorSize);

		return status;
	}

	/* If there's no RAW config, compute the sensor configuration here. */
	PixelFormat rawFormat = data_->bestRawFormat();
	if (!rawFormat.isValid())
		return Invalid;

	sensorFormat_.code = data_->pixfmtToMbusCode(rawFormat);

	uint16_t distance = std::numeric_limits<uint16_t>::max();
	const auto sizes = data_->sizes(sensorFormat_.code);
	Size bestSize;
	for (const auto &size : sizes) {
		if (minSensorSize.width > size.width ||
		    minSensorSize.height > size.height)
			continue;

		uint16_t dist = std::abs(static_cast<int>(minSensorSize.width) -
					 static_cast<int>(size.width)) +
				std::abs(static_cast<int>(minSensorSize.height) -
					 static_cast<int>(size.height));
		if (dist < distance) {
			distance = dist;
			bestSize = size;
		}
	}
	sensorFormat_.size = bestSize;

	LOG(MaliC55, Debug) << "Computed sensor configuration " << sensorFormat_;

	return status;
}

class PipelineHandlerMaliC55 : public PipelineHandler
{
public:
	PipelineHandlerMaliC55(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	int allocateBuffers(Camera *camera);
	void freeBuffers(Camera *camera);

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	void imageBufferReady(FrameBuffer *buffer);
	void paramsBufferReady(FrameBuffer *buffer);
	void statsBufferReady(FrameBuffer *buffer);
	void cruBufferReady(FrameBuffer *buffer);
	void paramsComputed(unsigned int requestId, uint32_t bytesused);
	void statsProcessed(unsigned int requestId, const ControlList &metadata);

	bool match(DeviceEnumerator *enumerator) override;

private:
	struct MaliC55Pipe {
		std::unique_ptr<V4L2Subdevice> resizer;
		std::unique_ptr<V4L2VideoDevice> cap;
		MediaLink *link;
		Stream *stream;
	};

	enum {
		MaliC55FR,
		MaliC55DS,
		MaliC55NumPipes,
	};

	MaliC55CameraData *cameraData(Camera *camera)
	{
		return static_cast<MaliC55CameraData *>(camera->_d());
	}

	MaliC55Pipe *pipeFromStream(MaliC55CameraData *data, Stream *stream)
	{
		if (stream == &data->frStream_)
			return &pipes_[MaliC55FR];
		else if (stream == &data->dsStream_)
			return &pipes_[MaliC55DS];
		else
			LOG(MaliC55, Fatal) << "Stream " << stream << " not valid";
		return nullptr;
	}

	MaliC55Pipe *pipeFromStream(MaliC55CameraData *data, const Stream *stream)
	{
		return pipeFromStream(data, const_cast<Stream *>(stream));
	}

	void resetPipes()
	{
		for (MaliC55Pipe &pipe : pipes_)
			pipe.stream = nullptr;
	}

	MaliC55FrameInfo *findFrameInfo(FrameBuffer *buffer);
	MaliC55FrameInfo *findFrameInfo(Request *request);
	void tryComplete(MaliC55FrameInfo *info, bool cancelled = false);

	int configureRawStream(MaliC55CameraData *data,
			       const StreamConfiguration &config,
			       V4L2SubdeviceFormat &subdevFormat);
	int configureProcessedStream(MaliC55CameraData *data,
				     const StreamConfiguration &config,
				     V4L2SubdeviceFormat &subdevFormat);

	MaliC55FrameInfo *prepareFrameInfo(Request *request);
	void queueRequestToCru(MaliC55CameraData *data, Request *request);

	void applyScalerCrop(Camera *camera, const ControlList &controls);

	bool registerMaliCamera(std::unique_ptr<MaliC55CameraData> data,
				const std::string &name);
	bool registerTPGCamera(MediaLink *link);
	bool registerSensorCamera(MediaLink *link);
	bool registerMemoryInputCamera(MediaLink *link);

	std::shared_ptr<MediaDevice> media_;
	std::shared_ptr<MediaDevice> cruMedia_;
	std::unique_ptr<V4L2Subdevice> isp_;
	std::unique_ptr<V4L2VideoDevice> stats_;
	std::unique_ptr<V4L2VideoDevice> params_;
	std::unique_ptr<V4L2Subdevice> ivcSd_;
	std::unique_ptr<V4L2VideoDevice> ivc_;

	std::vector<std::unique_ptr<FrameBuffer>> statsBuffers_;
	std::queue<FrameBuffer *> availableStatsBuffers_;

	std::vector<std::unique_ptr<FrameBuffer>> paramsBuffers_;
	std::queue<FrameBuffer *> availableParamsBuffers_;

	std::map<unsigned int, MaliC55FrameInfo> frameInfoMap_;

	std::array<MaliC55Pipe, MaliC55NumPipes> pipes_;

	bool dsFitted_;
};

PipelineHandlerMaliC55::PipelineHandlerMaliC55(CameraManager *manager)
	: PipelineHandler(manager, kMaliC55BufferCount), dsFitted_(true)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerMaliC55::generateConfiguration(Camera *camera,
					      Span<const StreamRole> roles)
{
	MaliC55CameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<MaliC55CameraConfiguration>(data);
	bool frPipeAvailable = true;

	if (roles.empty())
		return config;

	/* Check if one stream is RAW to reserve the FR pipe for it. */
	if (std::find(roles.begin(), roles.end(), StreamRole::Raw) != roles.end())
		frPipeAvailable = false;

	for (const StreamRole &role : roles) {
		struct MaliC55Pipe *pipe;

		/* Assign pipe for this role. */
		if (role == StreamRole::Raw) {
			pipe = &pipes_[MaliC55FR];
		} else {
			if (frPipeAvailable) {
				pipe = &pipes_[MaliC55FR];
				frPipeAvailable = false;
			} else {
				pipe = &pipes_[MaliC55DS];
			}
		}

		Size size = std::min(Size{ 1920, 1080 }, data->resolution());
		PixelFormat pixelFormat;

		switch (role) {
		case StreamRole::StillCapture:
			size = data->resolution();
			[[fallthrough]];
		case StreamRole::VideoRecording:
			pixelFormat = formats::NV12;
			break;

		case StreamRole::Viewfinder:
			pixelFormat = formats::RGB565;
			break;

		case StreamRole::Raw:
			pixelFormat = data->bestRawFormat();
			if (!pixelFormat.isValid()) {
				LOG(MaliC55, Error)
					<< "Camera does not support RAW formats";
				return nullptr;
			}

			size = data->resolution();
			break;

		default:
			LOG(MaliC55, Error)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		std::map<PixelFormat, std::vector<SizeRange>> formats;
		for (const auto &maliFormat : maliC55FmtToCode) {
			PixelFormat pixFmt = maliFormat.first;
			bool isRaw = isFormatRaw(pixFmt);

			/* RAW formats are only supported on the FR pipe. */
			if (pipe != &pipes_[MaliC55FR] && isRaw)
				continue;

			if (isRaw) {
				/* Make sure the mbus code is supported. */
				int rawCode = data->pixfmtToMbusCode(pixFmt);
				if (rawCode < 0)
					continue;

				const auto sizes = data->sizes(rawCode);
				if (sizes.empty())
					continue;

				/* And list all sizes the sensor can produce. */
				std::vector<SizeRange> sizeRanges;
				std::transform(sizes.begin(), sizes.end(),
					       std::back_inserter(sizeRanges),
					       [](const Size &s) {
						       return SizeRange(s);
					       });

				formats[pixFmt] = sizeRanges;
			} else {
				/* Processed formats are always available. */
				Size maxSize = std::min(kMaliC55MaxSize,
							data->resolution());
				formats[pixFmt] = { kMaliC55MinSize, maxSize };
			}
		}

		StreamFormats streamFormats(formats);
		StreamConfiguration cfg(streamFormats);
		cfg.pixelFormat = pixelFormat;
		cfg.bufferCount = 4;
		cfg.size = size;

		config->addConfiguration(cfg);
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return nullptr;

	return config;
}

int PipelineHandlerMaliC55::configureRawStream(MaliC55CameraData *data,
					       const StreamConfiguration &config,
					       V4L2SubdeviceFormat &subdevFormat)
{
	Stream *stream = config.stream();
	MaliC55Pipe *pipe = pipeFromStream(data, stream);

	if (pipe != &pipes_[MaliC55FR]) {
		LOG(MaliC55, Fatal) << "Only the FR pipe supports RAW capture.";
		return -EINVAL;
	}

	/* Enable the debayer route to set fixed internal format on pad #0. */
	V4L2Subdevice::Routing routing = {};
	routing.emplace_back(V4L2Subdevice::Stream{ 0, 0 },
			     V4L2Subdevice::Stream{ 1, 0 },
			     V4L2_SUBDEV_ROUTE_FL_ACTIVE);

	int ret = pipe->resizer->setRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret)
		return ret;

	unsigned int rawCode = subdevFormat.code;
	subdevFormat.code = kMaliC55ISPInternalFormat;
	ret = pipe->resizer->setFormat(0, &subdevFormat);
	if (ret)
		return ret;

	/* Enable the bypass route and apply RAW formats there. */
	routing.clear();
	routing.emplace_back(V4L2Subdevice::Stream{ 2, 0 },
			     V4L2Subdevice::Stream{ 1, 0 },
			     V4L2_SUBDEV_ROUTE_FL_ACTIVE);
	ret = pipe->resizer->setRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret)
		return ret;

	subdevFormat.code = rawCode;
	ret = pipe->resizer->setFormat(2, &subdevFormat);
	if (ret)
		return ret;

	ret = pipe->resizer->setFormat(1, &subdevFormat);
	if (ret)
		return ret;

	return 0;
}

int PipelineHandlerMaliC55::configureProcessedStream(MaliC55CameraData *data,
						     const StreamConfiguration &config,
						     V4L2SubdeviceFormat &subdevFormat)
{
	Stream *stream = config.stream();
	MaliC55Pipe *pipe = pipeFromStream(data, stream);

	/* Enable the debayer route on the resizer pipe. */
	V4L2Subdevice::Routing routing = {};
	routing.emplace_back(V4L2Subdevice::Stream{ 0, 0 },
			     V4L2Subdevice::Stream{ 1, 0 },
			     V4L2_SUBDEV_ROUTE_FL_ACTIVE);

	int ret = pipe->resizer->setRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret)
		return ret;

	subdevFormat.code = kMaliC55ISPInternalFormat;
	ret = pipe->resizer->setFormat(0, &subdevFormat);
	if (ret)
		return ret;

	/*
	 * Compute the scaler-in to scaler-out ratio: first center-crop to align
	 * the FOV to the desired resolution, then scale to the desired size.
	 */
	Size scalerIn = subdevFormat.size.boundedToAspectRatio(config.size);
	int xCrop = (subdevFormat.size.width - scalerIn.width) / 2;
	int yCrop = (subdevFormat.size.height - scalerIn.height) / 2;
	Rectangle ispCrop = { xCrop, yCrop, scalerIn };
	ret = pipe->resizer->setSelection(0, V4L2_SEL_TGT_CROP, &ispCrop);
	if (ret)
		return ret;

	Rectangle ispCompose = { 0, 0, config.size };
	ret = pipe->resizer->setSelection(0, V4L2_SEL_TGT_COMPOSE, &ispCompose);
	if (ret)
		return ret;

	/*
	 * The source pad format size comes directly from the sink
	 * compose rectangle.
	 */
	subdevFormat.size = ispCompose.size();
	subdevFormat.code = maliC55FmtToCode.find(config.pixelFormat)->second;
	return pipe->resizer->setFormat(1, &subdevFormat);
}

int PipelineHandlerMaliC55::configure(Camera *camera,
				      CameraConfiguration *config)
{
	resetPipes();

	int ret = media_->disableLinks();
	if (ret)
		return ret;

	/* Link the graph depending if we are operating the TPG or a sensor. */
	MaliC55CameraData *data = cameraData(camera);
	ret = std::visit(utils::overloaded{
		[](MaliC55CameraData::Tpg &tpg) {
			const MediaEntity *tpgEntity = tpg.sd_->entity();
			return tpgEntity->getPadByIndex(0)->links()[0]->setEnabled(true);
		},
		[](MaliC55CameraData::Inline &in) {
			const MediaEntity *csi2Entity = in.csi2_->entity();
			return csi2Entity->getPadByIndex(1)->links()[0]->setEnabled(true);
		},
		[&](MaliC55CameraData::Memory &) {
			const MediaEntity *ivcEntity = ivcSd_->entity();
			return ivcEntity->getPadByIndex(1)->links()[0]->setEnabled(true);
		},
	}, data->input_);
	if (ret)
		return ret;

	MaliC55CameraConfiguration *maliConfig =
		static_cast<MaliC55CameraConfiguration *>(config);
	V4L2SubdeviceFormat subdevFormat = maliConfig->sensorFormat_;

	/* Apply format to the origin of the pipeline and propagate it. */
	ret = std::visit(utils::overloaded{
		[&](MaliC55CameraData::Tpg &tpg) {
			return tpg.sd_->setFormat(0, &subdevFormat);
		},
		[&](MaliC55CameraData::Inline &in) {
			int r = in.sensor_->setFormat(&subdevFormat,
						      maliConfig->combinedTransform());
			if (r)
				return r;

			r = in.csi2_->setFormat(0, &subdevFormat);
			if (r)
				return r;

			return in.csi2_->getFormat(1, &subdevFormat);
		},
		[&](MaliC55CameraData::Memory &mem) {
			V4L2DeviceFormat inputFormat;

			int r = mem.cru_->configure(&subdevFormat, &inputFormat);
			if (r)
				return r;

			/* Propagate the CRU format to the IVC input. */
			r = ivcSd_->setFormat(0, &subdevFormat);
			if (r)
				return r;

			r = ivcSd_->getFormat(1, &subdevFormat);
			if (r)
				return r;

			return ivc_->setFormat(&inputFormat);
		},
	}, data->input_);
	if (ret)
		return ret;

	V4L2DeviceFormat statsFormat;
	ret = stats_->getFormat(&statsFormat);
	if (ret)
		return ret;

	if (statsFormat.planes[0].size != sizeof(struct mali_c55_stats_buffer)) {
		LOG(MaliC55, Error) << "3a stats buffer size invalid";
		return -EINVAL;
	}

	/*
	 * Propagate the format to the ISP sink pad and configure the input
	 * crop rectangle (no crop at the moment).
	 */
	ret = isp_->setFormat(0, &subdevFormat);
	if (ret)
		return ret;

	Rectangle ispCrop(0, 0, subdevFormat.size);
	ret = isp_->setSelection(0, V4L2_SEL_TGT_CROP, &ispCrop);
	if (ret)
		return ret;

	/*
	 * Configure the resizer: fixed format the sink pad; use the media
	 * bus code associated with the desired capture format on the source
	 * pad.
	 *
	 * Configure the crop and compose rectangles to match the desired
	 * stream output size
	 *
	 * \todo Make the crop/scaler configurable
	 */
	for (const StreamConfiguration &streamConfig : *config) {
		Stream *stream = streamConfig.stream();
		MaliC55Pipe *pipe = pipeFromStream(data, stream);

		/*
		 * Enable the media link between the pipe's resizer and the
		 * capture video device
		 */

		ret = pipe->link->setEnabled(true);
		if (ret) {
			LOG(MaliC55, Error) << "Couldn't enable resizer's link";
			return ret;
		}

		if (isFormatRaw(streamConfig.pixelFormat))
			ret = configureRawStream(data, streamConfig, subdevFormat);
		else
			ret = configureProcessedStream(data, streamConfig, subdevFormat);
		if (ret) {
			LOG(MaliC55, Error) << "Failed to configure pipeline";
			return ret;
		}

		/* Now apply the pixel format and size to the capture device. */
		V4L2DeviceFormat captureFormat;
		captureFormat.fourcc = pipe->cap->toV4L2PixelFormat(streamConfig.pixelFormat);
		captureFormat.size = streamConfig.size;

		ret = pipe->cap->setFormat(&captureFormat);
		if (ret)
			return ret;

		pipe->stream = stream;
	}

	if (!data->ipa_)
		return 0;

	/*
	 * Enable the media link between the ISP subdevice and the statistics
	 * video device.
	 */
	const MediaEntity *ispEntity = isp_->entity();
	ret = ispEntity->getPadByIndex(3)->links()[0]->setEnabled(true);
	if (ret) {
		LOG(MaliC55, Error) << "Couldn't enable statistics link";
		return ret;
	}

	/*
	 * Enable the media link between the ISP subdevice and the parameters
	 * video device.
	 */
	ret = ispEntity->getPadByIndex(4)->links()[0]->setEnabled(true);
	if (ret) {
		LOG(MaliC55, Error) << "Couldn't enable parameters link";
		return ret;
	}

	/* We need to inform the IPA of the sensor configuration */
	ipa::mali_c55::IPAConfigInfo ipaConfig{};

	ret = data->sensor()->sensorInfo(&ipaConfig.sensorInfo);
	if (ret)
		return ret;

	ipaConfig.sensorControls = data->sensor()->controls();

	/*
	 * And we also need to tell the IPA the bayerOrder of the data (as
	 * affected by any flips that we've configured)
	 */
	const Transform &combinedTransform = maliConfig->combinedTransform();
	BayerFormat::Order bayerOrder = data->sensor()->bayerOrder(combinedTransform);

	ControlInfoMap ipaControls;
	ret = data->ipa_->configure(ipaConfig, utils::to_underlying(bayerOrder),
				    &ipaControls);
	if (ret) {
		LOG(MaliC55, Error) << "Failed to configure IPA";
		return ret;
	}

	data->updateControls(ipaControls);

	return 0;
}

int PipelineHandlerMaliC55::exportFrameBuffers(Camera *camera, Stream *stream,
					       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	MaliC55Pipe *pipe = pipeFromStream(cameraData(camera), stream);
	unsigned int count = stream->configuration().bufferCount;

	return pipe->cap->exportBuffers(count, buffers);
}

void PipelineHandlerMaliC55::freeBuffers(Camera *camera)
{
	MaliC55CameraData *data = cameraData(camera);

	while (!availableStatsBuffers_.empty())
		availableStatsBuffers_.pop();
	while (!availableParamsBuffers_.empty())
		availableParamsBuffers_.pop();

	statsBuffers_.clear();
	paramsBuffers_.clear();

	if (data->ipa_) {
		data->ipa_->unmapBuffers(data->ipaStatBuffers_);
		data->ipa_->unmapBuffers(data->ipaParamBuffers_);
	}
	data->ipaStatBuffers_.clear();
	data->ipaParamBuffers_.clear();

	if (stats_->releaseBuffers())
		LOG(MaliC55, Error) << "Failed to release stats buffers";

	if (params_->releaseBuffers())
		LOG(MaliC55, Error) << "Failed to release params buffers";

	if (auto *mem = std::get_if<MaliC55CameraData::Memory>(&data->input_)) {
		if (ivc_->releaseBuffers())
			LOG(MaliC55, Error) << "Failed to release input buffers";
		if (mem->cru_->freeBuffers())
			LOG(MaliC55, Error) << "Failed to release CRU buffers";
	}

	return;
}

int PipelineHandlerMaliC55::allocateBuffers(Camera *camera)
{
	MaliC55CameraData *data = cameraData(camera);
	unsigned int ipaBufferId = 1;
	int ret;

	auto pushBuffers = [&](const std::vector<std::unique_ptr<FrameBuffer>> &buffers,
			       std::queue<FrameBuffer *> &queue,
			       std::vector<IPABuffer> &ipaBuffers) {
		for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
			Span<const FrameBuffer::Plane> planes = buffer->planes();

			buffer->setCookie(ipaBufferId++);
			ipaBuffers.emplace_back(buffer->cookie(),
						std::vector<FrameBuffer::Plane>{ planes.begin(),
										 planes.end() });
			queue.push(buffer.get());
		}
	};

	if (std::holds_alternative<MaliC55CameraData::Memory>(data->input_)) {
		ret = ivc_->importBuffers(kMaliC55BufferCount);
		if (ret < 0)
			return ret;
	}

	ret = stats_->allocateBuffers(kMaliC55BufferCount, &statsBuffers_);
	if (ret < 0)
		return ret;

	pushBuffers(statsBuffers_, availableStatsBuffers_,
		    data->ipaStatBuffers_);

	ret = params_->allocateBuffers(kMaliC55BufferCount, &paramsBuffers_);
	if (ret < 0)
		return ret;

	pushBuffers(paramsBuffers_, availableParamsBuffers_,
		    data->ipaParamBuffers_);

	if (data->ipa_) {
		data->ipa_->mapBuffers(data->ipaStatBuffers_, true);
		data->ipa_->mapBuffers(data->ipaParamBuffers_, false);
	}

	return 0;
}

int PipelineHandlerMaliC55::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	MaliC55CameraData *data = cameraData(camera);
	int ret;

	ret = allocateBuffers(camera);
	if (ret)
		return ret;

	if (auto *mem = std::get_if<MaliC55CameraData::Memory>(&data->input_)) {
		ret = mem->cru_->start(kMaliC55BufferCount);
		if (ret) {
			LOG(MaliC55, Error)
				<< "Failed to start CRU " << camera->id();
			freeBuffers(camera);
			return ret;
		}

		ret = ivc_->streamOn();
		if (ret) {
			LOG(MaliC55, Error)
				<< "Failed to start IVC" << camera->id();
			freeBuffers(camera);
			return ret;
		}
	}

	if (data->ipa_) {
		ret = data->ipa_->start();
		if (ret) {
			LOG(MaliC55, Error)
				<< "Failed to start IPA" << camera->id();
			freeBuffers(camera);
			return ret;
		}
	}

	for (MaliC55Pipe &pipe : pipes_) {
		if (!pipe.stream)
			continue;

		Stream *stream = pipe.stream;

		ret = pipe.cap->importBuffers(stream->configuration().bufferCount);
		if (ret) {
			LOG(MaliC55, Error) << "Failed to import buffers";
			if (data->ipa_)
				data->ipa_->stop();
			freeBuffers(camera);
			return ret;
		}

		ret = pipe.cap->streamOn();
		if (ret) {
			LOG(MaliC55, Error) << "Failed to start stream";
			if (data->ipa_)
				data->ipa_->stop();
			freeBuffers(camera);
			return ret;
		}
	}

	ret = stats_->streamOn();
	if (ret) {
		LOG(MaliC55, Error) << "Failed to start stats stream";

		if (data->ipa_)
			data->ipa_->stop();

		for (MaliC55Pipe &pipe : pipes_) {
			if (pipe.stream)
				pipe.cap->streamOff();
		}

		freeBuffers(camera);
		return ret;
	}

	ret = params_->streamOn();
	if (ret) {
		LOG(MaliC55, Error) << "Failed to start params stream";

		stats_->streamOff();
		if (data->ipa_)
			data->ipa_->stop();

		for (MaliC55Pipe &pipe : pipes_) {
			if (pipe.stream)
				pipe.cap->streamOff();
		}

		freeBuffers(camera);
		return ret;
	}

	ret = isp_->setFrameStartEnabled(true);
	if (ret)
		LOG(MaliC55, Error) << "Failed to enable frame start events";

	return 0;
}

void PipelineHandlerMaliC55::stopDevice(Camera *camera)
{
	MaliC55CameraData *data = cameraData(camera);

	isp_->setFrameStartEnabled(false);

	if (auto *mem = std::get_if<MaliC55CameraData::Memory>(&data->input_)) {
		ivc_->streamOff();
		mem->cru_->stop();
	}

	for (MaliC55Pipe &pipe : pipes_) {
		if (!pipe.stream)
			continue;

		pipe.cap->streamOff();
		pipe.cap->releaseBuffers();
	}

	stats_->streamOff();
	params_->streamOff();
	if (data->ipa_)
		data->ipa_->stop();
	freeBuffers(camera);
}

void PipelineHandlerMaliC55::applyScalerCrop(Camera *camera,
					     const ControlList &controls)
{
	MaliC55CameraData *data = cameraData(camera);

	const auto &scalerCrop = controls.get<Rectangle>(controls::ScalerCrop);
	if (!scalerCrop)
		return;

	if (std::holds_alternative<MaliC55CameraData::Tpg>(data->input_)) {
		LOG(MaliC55, Error) << "ScalerCrop not supported for TPG";
		return;
	}

	Rectangle nativeCrop = *scalerCrop;

	IPACameraSensorInfo sensorInfo;
	int ret = data->sensor()->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(MaliC55, Error) << "Failed to retrieve sensor info";
		return;
	}

	/*
	 * The ScalerCrop rectangle re-scaling in the ISP crop rectangle
	 * comes straight from the RPi pipeline handler.
	 *
	 * Create a version of the crop rectangle aligned to the analogue crop
	 * rectangle top-left coordinates and scaled in the [analogue crop to
	 * output frame] ratio to take into account binning/skipping on the
	 * sensor.
	 */
	Rectangle ispCrop = nativeCrop.translatedBy(-sensorInfo.analogCrop
							       .topLeft());
	ispCrop.scaleBy(sensorInfo.outputSize, sensorInfo.analogCrop.size());

	/*
	 * The crop rectangle should be:
	 * 1. At least as big as ispMinCropSize_, once that's been
	 *    enlarged to the same aspect ratio.
	 * 2. With the same mid-point, if possible.
	 * 3. But it can't go outside the sensor area.
	 */
	Rectangle ispMinCrop{ 0, 0, 640, 480 };
	Size minSize = ispMinCrop.size().expandedToAspectRatio(nativeCrop.size());
	Size size = ispCrop.size().expandedTo(minSize);
	ispCrop = size.centeredTo(ispCrop.center())
		      .enclosedIn(Rectangle(sensorInfo.outputSize));

	/*
	 * As the resizer can't upscale, the crop rectangle has to be larger
	 * than the larger stream output size.
	 */
	Size maxYuvSize;
	for (MaliC55Pipe &pipe : pipes_) {
		if (!pipe.stream)
			continue;

		const StreamConfiguration &config = pipe.stream->configuration();
		if (isFormatRaw(config.pixelFormat)) {
			LOG(MaliC55, Debug) << "Cannot crop with a RAW stream";
			return;
		}

		Size streamSize = config.size;
		if (streamSize.width > maxYuvSize.width)
			maxYuvSize.width = streamSize.width;
		if (streamSize.height > maxYuvSize.height)
			maxYuvSize.height = streamSize.height;
	}

	ispCrop.size().expandTo(maxYuvSize);

	/*
	 * Now apply the scaler crop to each enabled output. This overrides the
	 * crop configuration performed at configure() time and can cause
	 * square pixels if the crop rectangle and scaler output FOV ratio are
	 * different.
	 */
	for (MaliC55Pipe &pipe : pipes_) {
		if (!pipe.stream)
			continue;

		/* Create a copy to avoid setSelection() to modify ispCrop. */
		Rectangle pipeCrop = ispCrop;
		ret = pipe.resizer->setSelection(0, V4L2_SEL_TGT_CROP, &pipeCrop);
		if (ret) {
			LOG(MaliC55, Error)
				<< "Failed to apply crop to "
				<< (pipe.stream == &data->frStream_ ?
				    "FR" : "DS") << " pipe";
			return;
		}
	}
}

MaliC55FrameInfo *PipelineHandlerMaliC55::prepareFrameInfo(Request *request)
{
	if (availableStatsBuffers_.empty()) {
		LOG(MaliC55, Error) << "Stats buffer underrun";
		return nullptr;
	}

	if (availableParamsBuffers_.empty()) {
		LOG(MaliC55, Error) << "Params buffer underrun";
		return nullptr;
	}

	MaliC55FrameInfo &frameInfo = frameInfoMap_[request->sequence()];
	frameInfo.request = request;
	frameInfo.statBuffer = availableStatsBuffers_.front();
	availableStatsBuffers_.pop();
	frameInfo.paramBuffer = availableParamsBuffers_.front();
	availableParamsBuffers_.pop();

	frameInfo.paramsDone = false;
	frameInfo.statsDone = false;

	return &frameInfo;
}

void PipelineHandlerMaliC55::queueRequestToCru(MaliC55CameraData *data,
					       Request *request)
{
	auto *mem = std::get_if<MaliC55CameraData::Memory>(&data->input_);
	ASSERT(mem);

	FrameBuffer *cruBuffer = mem->cru_->queueBuffer(request);
	ASSERT(cruBuffer);

	auto frameInfo = prepareFrameInfo(request);
	ASSERT(frameInfo);

	frameInfo->rawBuffer = cruBuffer;

	for (auto &[stream, buffer] : request->buffers()) {
		MaliC55Pipe *pipe = pipeFromStream(data, stream);

		pipe->cap->queueBuffer(buffer);
	}

	data->ipa_->queueRequest(request->sequence(), request->controls());
}

int PipelineHandlerMaliC55::queueRequestDevice(Camera *camera, Request *request)
{
	MaliC55CameraData *data = cameraData(camera);

	/*
	 * If we're in memory input mode, we need to queue the Request to the
	 * CRU, otherwise we can just do everything immediately.
	 */
	if (std::holds_alternative<MaliC55CameraData::Memory>(data->input_)) {
		queueRequestToCru(data, request);

		return 0;
	}

	/* Do not run the IPA if the TPG is in use. */
	if (!data->ipa_) {
		MaliC55FrameInfo frameInfo;
		frameInfo.request = request;
		frameInfo.statBuffer = nullptr;
		frameInfo.paramBuffer = nullptr;
		frameInfo.paramsDone = true;
		frameInfo.statsDone = true;

		frameInfoMap_[request->sequence()] = frameInfo;

		for (auto &[stream, buffer] : request->buffers()) {
			MaliC55Pipe *pipe = pipeFromStream(data, stream);

			pipe->cap->queueBuffer(buffer);
		}

		return 0;
	}

	auto frameInfo = prepareFrameInfo(request);
	ASSERT(frameInfo);

	data->ipa_->queueRequest(request->sequence(), request->controls());
	data->ipa_->fillParams(request->sequence(),
			       frameInfo->paramBuffer->cookie());

	return 0;
}

MaliC55FrameInfo *PipelineHandlerMaliC55::findFrameInfo(Request *request)
{
	for (auto &[sequence, info] : frameInfoMap_) {
		if (info.request == request)
			return &info;
	}

	return nullptr;
}

MaliC55FrameInfo *PipelineHandlerMaliC55::findFrameInfo(FrameBuffer *buffer)
{
	for (auto &[sequence, info] : frameInfoMap_) {
		if (info.paramBuffer == buffer ||
		    info.statBuffer == buffer ||
		    info.rawBuffer == buffer)
			return &info;
	}

	return nullptr;
}

void PipelineHandlerMaliC55::tryComplete(MaliC55FrameInfo *info, bool cancelled)
{
	/*
	 * If the buffer has been cancelled, we complete the request without
	 * waiting for the IPA.
	 */
	if (!cancelled && (!info->paramsDone || !info->statsDone))
		return;

	Request *request = info->request;
	if (request->hasPendingBuffers())
		return;

	if (info->statBuffer)
		availableStatsBuffers_.push(info->statBuffer);
	if (info->paramBuffer)
		availableParamsBuffers_.push(info->paramBuffer);

	frameInfoMap_.erase(request->sequence());

	completeRequest(request);
}

void PipelineHandlerMaliC55::imageBufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();
	MaliC55FrameInfo *info = findFrameInfo(request);
	ASSERT(info);

	if (completeBuffer(request, buffer))
		tryComplete(info,
			    buffer->metadata().status == FrameMetadata::FrameCancelled);
}

void PipelineHandlerMaliC55::paramsBufferReady(FrameBuffer *buffer)
{
	MaliC55FrameInfo *info = findFrameInfo(buffer);
	if (!info)
		return;

	info->paramsDone = true;

	tryComplete(info);
}

void PipelineHandlerMaliC55::statsBufferReady(FrameBuffer *buffer)
{
	MaliC55FrameInfo *info = findFrameInfo(buffer);
	if (!info)
		return;

	Request *request = info->request;
	MaliC55CameraData *data = cameraData(request->_d()->camera());

	ControlList sensorControls = data->delayedCtrls_->get(buffer->metadata().sequence);

	data->ipa_->processStats(request->sequence(), buffer->cookie(),
				 sensorControls);
}

void PipelineHandlerMaliC55::cruBufferReady(FrameBuffer *buffer)
{
	/*
	 * If the buffer has been cancelled, do not ask the IPA to prepare
	 * parameters.
	 *
	 * The Request this cancelled buffer belongs to will be handled by
	 * imageBufferReady() as we have queued buffers to the ISP capture
	 * devices at the same time we have queued this buffer to the CRU
	 * when running in m2m mode.
	 */
	if (buffer->metadata().status == FrameMetadata::FrameCancelled)
		return;

	MaliC55FrameInfo *info = findFrameInfo(buffer);
	ASSERT(info);

	Request *request = info->request;
	request->_d()->metadata().set(controls::SensorTimestamp,
				      buffer->metadata().timestamp);

	MaliC55CameraData *data = cameraData(request->_d()->camera());
	data->ipa_->fillParams(request->sequence(), info->paramBuffer->cookie());
}

void PipelineHandlerMaliC55::paramsComputed(unsigned int requestId, uint32_t bytesused)
{
	auto it = frameInfoMap_.find(requestId);
	if (it == frameInfoMap_.end())
		return;

	MaliC55FrameInfo &frameInfo = it->second;
	Request *request = frameInfo.request;
	if (!request)
		return;

	MaliC55CameraData *data = cameraData(request->_d()->camera());

	/*
	 * Queue buffers for stats and params, then queue buffers to the capture
	 * video devices if we're running in Inline mode or with the TPG.
	 *
	 * If we're running in M2M buffers have been queued to the capture
	 * devices at queueRequestToCru() time and here we only have to queue
	 * buffers to the IVC input to start a transfer.
	 */

	frameInfo.paramBuffer->_d()->metadata().planes()[0].bytesused = bytesused;
	params_->queueBuffer(frameInfo.paramBuffer);
	stats_->queueBuffer(frameInfo.statBuffer);

	if (!std::holds_alternative<MaliC55CameraData::Memory>(data->input_)) {
		for (auto &[stream, buffer] : request->buffers()) {
			MaliC55Pipe *pipe = pipeFromStream(data, stream);

			pipe->cap->queueBuffer(buffer);
		}
	} else {
		ivc_->queueBuffer(frameInfo.rawBuffer);
		frameInfo.rawBuffer = nullptr;
	}
}

void PipelineHandlerMaliC55::statsProcessed(unsigned int requestId,
					    const ControlList &metadata)
{
	auto it = frameInfoMap_.find(requestId);
	if (it == frameInfoMap_.end())
		return;

	MaliC55FrameInfo &frameInfo = it->second;
	if (!frameInfo.request)
		return;

	frameInfo.statsDone = true;
	frameInfo.request->_d()->metadata().merge(metadata);

	tryComplete(&frameInfo);
}

bool PipelineHandlerMaliC55::registerMaliCamera(std::unique_ptr<MaliC55CameraData> data,
						const std::string &name)
{
	if (data->loadIPA())
		return false;

	if (data->ipa_) {
		data->ipa_->statsProcessed.connect(this, &PipelineHandlerMaliC55::statsProcessed);
		data->ipa_->paramsComputed.connect(this, &PipelineHandlerMaliC55::paramsComputed);
	}

	std::set<Stream *> streams{ &data->frStream_ };
	if (dsFitted_)
		streams.insert(&data->dsStream_);

	std::shared_ptr<Camera> camera = Camera::create(std::move(data),
							name, streams);
	registerCamera(std::move(camera));

	return true;
}

/*
 * The only camera we support through direct connection to the ISP is the
 * Mali-C55 TPG. Check we have that and warn if not.
 */
bool PipelineHandlerMaliC55::registerTPGCamera(MediaLink *link)
{
	const std::string &name = link->source()->entity()->name();
	if (name != "mali-c55 tpg") {
		LOG(MaliC55, Warning) << "Unsupported direct connection to "
				      << link->source()->entity()->name();
		/*
		 * Return true and just skip registering a camera for this
		 * entity.
		 */
		return true;
	}

	std::unique_ptr<MaliC55CameraData> data =
		std::make_unique<MaliC55CameraData>(this);

	if (!data->initTpg(link->source()->entity()))
		return false;

	return registerMaliCamera(std::move(data), name);
}

/*
 * Register a Camera for each sensor connected to the ISP through a CSI-2
 * receiver.
 *
 * \todo Support more complex topologies, such as video muxes.
 */
bool PipelineHandlerMaliC55::registerSensorCamera(MediaLink *ispLink)
{
	MediaEntity *csi2 = ispLink->source()->entity();
	const MediaPad *csi2Sink = csi2->getPadByIndex(0);

	for (MediaLink *link : csi2Sink->links()) {
		MediaEntity *sensor = link->source()->entity();
		unsigned int function = sensor->function();

		if (function != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		std::unique_ptr<MaliC55CameraData> data =
			std::make_unique<MaliC55CameraData>(this);

		auto *in = data->initInline(sensor);
		if (!in)
			return false;

		data->properties_ = in->sensor_->properties();

		const CameraSensorProperties::SensorDelays &delays =
			in->sensor_->sensorDelays();
		std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
			{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
			{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
		};

		V4L2Subdevice *sensorSubdev = in->sensor_->device();
		data->delayedCtrls_ = std::make_unique<DelayedControls>(sensorSubdev,
									params);
		isp_->frameStart.connect(data->delayedCtrls_.get(),
					 &DelayedControls::applyControls);

		/* \todo Init properties. */

		if (!registerMaliCamera(std::move(data), sensor->name()))
			return false;
	}

	return true;
}

bool PipelineHandlerMaliC55::registerMemoryInputCamera(MediaLink *link)
{
	/*
	 * Errors are not fatal at this point, return true and just skip
	 * registering a camera for this media link.
	 */
	const std::string &name = link->source()->entity()->name();
	if (name != "rzv2h ivc block") {
		LOG(MaliC55, Warning) << "Unsupported direct connection to "
				      << link->source()->entity()->name();
		return true;
	}

	if (!cruMedia_) {
		LOG(MaliC55, Warning) << "Unable to find CRU for memory input";
		return true;
	}

	/*
	 * From now on, errors should be reported up as we have the components
	 * we need to support memory-to-memory and if we fail something has
	 * to be fixed.
	 */

	ivcSd_ = V4L2Subdevice::fromEntityName(media_.get(),
					       "rzv2h ivc block");
	if (!ivcSd_ || ivcSd_->open() < 0)
		return false;

	ivc_ = V4L2VideoDevice::fromEntityName(media_.get(),
					       "rzv2h-ivc");
	if (!ivc_ || ivc_->open() < 0)
		return false;

	std::unique_ptr<MaliC55CameraData> data =
		std::make_unique<MaliC55CameraData>(this);

	auto *mem = data->initMemory(cruMedia_.get());
	if (!mem)
		return false;

	CameraSensor *sensor = data->sensor();
	data->properties_ = sensor->properties();

	const CameraSensorProperties::SensorDelays &delays = sensor->sensorDelays();
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
		{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
	};

	data->delayedCtrls_ =
		std::make_unique<DelayedControls>(sensor->device(), params);
	isp_->frameStart.connect(data->delayedCtrls_.get(),
				 &DelayedControls::applyControls);

	ivc_->bufferReady.connect(mem->cru_.get(), &RZG2LCRU::returnBuffer);

	V4L2VideoDevice *cruOutput = mem->cru_->output();
	cruOutput->bufferReady.connect(this, &PipelineHandlerMaliC55::cruBufferReady);

	return registerMaliCamera(std::move(data), sensor->device()->entity()->name());
}

bool PipelineHandlerMaliC55::match(DeviceEnumerator *enumerator)
{
	const MediaPad *ispSink;

	/*
	 * We search for just the always-available elements of the media graph.
	 * The TPG and the downscale pipe are both optional blocks and may not
	 * be fitted.
	 */
	DeviceMatch c55Dm("mali-c55");
	c55Dm.add("mali-c55 isp");
	c55Dm.add("mali-c55 resizer fr");
	c55Dm.add("mali-c55 fr");
	c55Dm.add("mali-c55 3a stats");
	c55Dm.add("mali-c55 3a params");

	media_ = acquireMediaDevice(enumerator, c55Dm);
	if (!media_)
		return false;

	isp_ = V4L2Subdevice::fromEntityName(media_.get(), "mali-c55 isp");
	if (isp_->open() < 0)
		return false;

	stats_ = V4L2VideoDevice::fromEntityName(media_.get(), "mali-c55 3a stats");
	if (stats_->open() < 0)
		return false;

	params_ = V4L2VideoDevice::fromEntityName(media_.get(), "mali-c55 3a params");
	if (params_->open() < 0)
		return false;

	MaliC55Pipe *frPipe = &pipes_[MaliC55FR];
	frPipe->resizer = V4L2Subdevice::fromEntityName(media_.get(), "mali-c55 resizer fr");
	if (frPipe->resizer->open() < 0)
		return false;

	frPipe->cap = V4L2VideoDevice::fromEntityName(media_.get(), "mali-c55 fr");
	if (frPipe->cap->open() < 0)
		return false;

	frPipe->link = media_->link("mali-c55 resizer fr", 1, "mali-c55 fr", 0);
	if (!frPipe->link) {
		LOG(MaliC55, Error) << "No link between fr resizer and video node";
		return false;
	}

	frPipe->cap->bufferReady.connect(this, &PipelineHandlerMaliC55::imageBufferReady);

	dsFitted_ = !!media_->getEntityByName("mali-c55 ds");
	if (dsFitted_) {
		LOG(MaliC55, Debug) << "Downscaler pipe is fitted";

		MaliC55Pipe *dsPipe = &pipes_[MaliC55DS];

		dsPipe->resizer = V4L2Subdevice::fromEntityName(media_.get(), "mali-c55 resizer ds");
		if (dsPipe->resizer->open() < 0)
			return false;

		dsPipe->cap = V4L2VideoDevice::fromEntityName(media_.get(), "mali-c55 ds");
		if (dsPipe->cap->open() < 0)
			return false;

		dsPipe->link = media_->link("mali-c55 resizer ds", 1,
					    "mali-c55 ds", 0);
		if (!dsPipe->link) {
			LOG(MaliC55, Error) << "No link between ds resizer and video node";
			return false;
		}

		dsPipe->cap->bufferReady.connect(this, &PipelineHandlerMaliC55::imageBufferReady);
	}

	stats_->bufferReady.connect(this, &PipelineHandlerMaliC55::statsBufferReady);
	params_->bufferReady.connect(this, &PipelineHandlerMaliC55::paramsBufferReady);

	/*
	 * We also need to search for the rzg2l-cru CSI-2 receiver. If we find
	 * that then we need to work in memory input mode instead of the inline
	 * mode. The absence of this match is not necessarily a failure at this
	 * point...it depends on the media links that we investigate momentarily.
	 *
	 * This is a bit hacky, because there could be multiple of these media
	 * devices and we're just taking the first. We need modular pipelines to
	 * properly solve the issue.
	 */
	static const std::regex cruCsi2Regex("csi-[0-9a-f]{8}.csi2");
	static const std::regex cruIpRegex("cru-ip-[0-9a-f]{8}.cru[0-9]");

	DeviceMatch cruDm("rzg2l_cru");
	cruDm.add(cruCsi2Regex);
	cruDm.add(cruIpRegex);
	cruDm.add("CRU output");
	cruMedia_ = acquireMediaDevice(enumerator, cruDm);

	ispSink = isp_->entity()->getPadByIndex(0);
	if (!ispSink || ispSink->links().empty()) {
		LOG(MaliC55, Error) << "ISP sink pad error";
		return false;
	}

	/*
	 * We could have several links pointing to the ISP's sink pad, which
	 * will be from entities with one of the following functions:
	 *
	 * MEDIA_ENT_F_CAM_SENSOR - The test pattern generator
	 * MEDIA_ENT_F_VID_IF_BRIDGE - A CSI-2 receiver
	 * MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER - An input device
	 */
	bool registered;
	for (MediaLink *link : ispSink->links()) {
		unsigned int function = link->source()->entity()->function();

		switch (function) {
		case MEDIA_ENT_F_CAM_SENSOR:
			registered = registerTPGCamera(link);
			if (!registered)
				return registered;

			break;
		case MEDIA_ENT_F_VID_IF_BRIDGE:
			registered = registerSensorCamera(link);
			if (!registered)
				return registered;

			break;
		case MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER:
			registered = registerMemoryInputCamera(link);
			if (!registered)
				return registered;

			break;
		default:
			LOG(MaliC55, Error) << "Unsupported entity function";
			return false;
		}
	}

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMaliC55, "mali-c55")

} /* namespace libcamera */
