/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Pipeline handler for virtual cameras
 */

#include "virtual.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <errno.h>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <stdint.h>
#include <string>
#include <time.h>
#include <utility>
#include <vector>

#include <libcamera/base/flags.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/yaml_parser.h"

#include "pipeline/virtual/config_parser.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(Virtual)

namespace {

uint64_t currentTimestamp()
{
	const auto now = std::chrono::steady_clock::now();
	auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(
		now.time_since_epoch());

	return nsecs.count();
}

} /* namespace */

template<class... Ts>
struct overloaded : Ts... {
	using Ts::operator()...;
};
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

class VirtualCameraConfiguration : public CameraConfiguration
{
public:
	static constexpr unsigned int kBufferCount = 4;

	VirtualCameraConfiguration(VirtualCameraData *data);

	Status validate() override;

private:
	const VirtualCameraData *data_;
};

class PipelineHandlerVirtual : public PipelineHandler
{
public:
	PipelineHandlerVirtual(CameraManager *manager);
	~PipelineHandlerVirtual();

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	static bool created_;

	VirtualCameraData *cameraData(Camera *camera)
	{
		return static_cast<VirtualCameraData *>(camera->_d());
	}

	bool initFrameGenerator(Camera *camera);

	DmaBufAllocator dmaBufAllocator_;

	bool resetCreated_ = false;
};

VirtualCameraData::VirtualCameraData(PipelineHandler *pipe,
				     const std::vector<Resolution> &supportedResolutions)
	: Camera::Private(pipe)
{
	config_.resolutions = supportedResolutions;
	for (const auto &resolution : config_.resolutions) {
		if (config_.minResolutionSize.isNull() || config_.minResolutionSize > resolution.size)
			config_.minResolutionSize = resolution.size;

		config_.maxResolutionSize = std::max(config_.maxResolutionSize, resolution.size);
	}

	properties_.set(properties::PixelArrayActiveAreas,
			{ Rectangle(config_.maxResolutionSize) });

	/* \todo Support multiple streams and pass multi_stream_test */
	streamConfigs_.resize(kMaxStream);
}

VirtualCameraConfiguration::VirtualCameraConfiguration(VirtualCameraData *data)
	: CameraConfiguration(), data_(data)
{
}

CameraConfiguration::Status VirtualCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty()) {
		LOG(Virtual, Error) << "Empty config";
		return Invalid;
	}

	/* Only one stream is supported */
	if (config_.size() > VirtualCameraData::kMaxStream) {
		config_.resize(VirtualCameraData::kMaxStream);
		status = Adjusted;
	}

	for (StreamConfiguration &cfg : config_) {
		bool adjusted = false;
		bool found = false;
		for (const auto &resolution : data_->config_.resolutions) {
			if (resolution.size.width == cfg.size.width &&
			    resolution.size.height == cfg.size.height) {
				found = true;
				break;
			}
		}

		if (!found) {
			/*
			 * \todo It's a pipeline's decision to choose a
			 * resolution when the exact one is not supported.
			 * Defining the default logic in PipelineHandler to
			 * find the closest resolution would be nice.
			 */
			cfg.size = data_->config_.maxResolutionSize;
			status = Adjusted;
			adjusted = true;
		}

		if (cfg.pixelFormat != formats::NV12) {
			cfg.pixelFormat = formats::NV12;
			status = Adjusted;
			adjusted = true;
		}

		if (adjusted)
			LOG(Virtual, Info)
				<< "Stream configuration adjusted to " << cfg.toString();

		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
		cfg.stride = info.stride(cfg.size.width, 0, 1);
		cfg.frameSize = info.frameSize(cfg.size, 1);

		cfg.bufferCount = VirtualCameraConfiguration::kBufferCount;
	}

	return status;
}

/* static */
bool PipelineHandlerVirtual::created_ = false;

PipelineHandlerVirtual::PipelineHandlerVirtual(CameraManager *manager)
	: PipelineHandler(manager),
	  dmaBufAllocator_(DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap |
			   DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap |
			   DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf)
{
}

PipelineHandlerVirtual::~PipelineHandlerVirtual()
{
	if (resetCreated_)
		created_ = false;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerVirtual::generateConfiguration(Camera *camera,
					      Span<const StreamRole> roles)
{
	VirtualCameraData *data = cameraData(camera);
	auto config = std::make_unique<VirtualCameraConfiguration>(data);

	if (roles.empty())
		return config;

	for (const StreamRole role : roles) {
		switch (role) {
		case StreamRole::StillCapture:
		case StreamRole::VideoRecording:
		case StreamRole::Viewfinder:
			break;

		case StreamRole::Raw:
		default:
			LOG(Virtual, Error)
				<< "Requested stream role not supported: " << role;
			return {};
		}

		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		PixelFormat pixelFormat = formats::NV12;
		streamFormats[pixelFormat] = { { data->config_.minResolutionSize,
						 data->config_.maxResolutionSize } };
		StreamFormats formats(streamFormats);
		StreamConfiguration cfg(formats);
		cfg.pixelFormat = pixelFormat;
		cfg.size = data->config_.maxResolutionSize;
		cfg.bufferCount = VirtualCameraConfiguration::kBufferCount;

		config->addConfiguration(cfg);
	}

	ASSERT(config->validate() != CameraConfiguration::Invalid);

	return config;
}

int PipelineHandlerVirtual::configure(Camera *camera,
				      CameraConfiguration *config)
{
	VirtualCameraData *data = cameraData(camera);
	for (auto [i, c] : utils::enumerate(*config)) {
		c.setStream(&data->streamConfigs_[i].stream);
		/* Start reading the images/generating test patterns */
		data->streamConfigs_[i].frameGenerator->configure(c.size);
	}

	return 0;
}

int PipelineHandlerVirtual::exportFrameBuffers([[maybe_unused]] Camera *camera,
					       Stream *stream,
					       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	if (!dmaBufAllocator_.isValid())
		return -ENOBUFS;

	const StreamConfiguration &config = stream->configuration();
	const PixelFormatInfo &info = PixelFormatInfo::info(config.pixelFormat);

	std::vector<unsigned int> planeSizes;
	for (size_t i = 0; i < info.numPlanes(); ++i)
		planeSizes.push_back(info.planeSize(config.size, i));

	return dmaBufAllocator_.exportBuffers(config.bufferCount, planeSizes, buffers);
}

int PipelineHandlerVirtual::start([[maybe_unused]] Camera *camera,
				  [[maybe_unused]] const ControlList *controls)
{
	VirtualCameraData *data = cameraData(camera);

	for (auto &s : data->streamConfigs_)
		s.seq = 0;

	return 0;
}

void PipelineHandlerVirtual::stopDevice([[maybe_unused]] Camera *camera)
{
}

int PipelineHandlerVirtual::queueRequestDevice([[maybe_unused]] Camera *camera,
					       Request *request)
{
	VirtualCameraData *data = cameraData(camera);
	const auto timestamp = currentTimestamp();

	for (auto const &[stream, buffer] : request->buffers()) {
		bool found = false;
		/* map buffer and fill test patterns */
		for (auto &streamConfig : data->streamConfigs_) {
			if (stream == &streamConfig.stream) {
				FrameMetadata &fmd = buffer->_d()->metadata();

				fmd.status = FrameMetadata::Status::FrameSuccess;
				fmd.sequence = streamConfig.seq++;
				fmd.timestamp = timestamp;

				for (const auto [i, p] : utils::enumerate(buffer->planes()))
					fmd.planes()[i].bytesused = p.length;

				found = true;

				if (streamConfig.frameGenerator->generateFrame(
					    stream->configuration().size, buffer))
					fmd.status = FrameMetadata::Status::FrameError;

				completeBuffer(request, buffer);
				break;
			}
		}
		ASSERT(found);
	}

	request->metadata().set(controls::SensorTimestamp, timestamp);
	completeRequest(request);

	return 0;
}

bool PipelineHandlerVirtual::match([[maybe_unused]] DeviceEnumerator *enumerator)
{
	if (created_)
		return false;

	created_ = true;

	std::string configFile = configurationFile("virtual", "virtual.yaml", true);
	if (configFile.empty()) {
		LOG(Virtual, Debug)
			<< "Configuration file not found, skipping virtual cameras";
		return false;
	}

	File file(configFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(Virtual, Error)
			<< "Failed to open config file `" << file.fileName() << "`";
		return false;
	}

	ConfigParser parser;
	auto configData = parser.parseConfigFile(file, this);
	if (configData.size() == 0) {
		LOG(Virtual, Error) << "Failed to parse any cameras from the config file: "
				    << file.fileName();
		return false;
	}

	/* Configure and register cameras with configData */
	for (auto &data : configData) {
		std::set<Stream *> streams;
		for (auto &streamConfig : data->streamConfigs_)
			streams.insert(&streamConfig.stream);
		std::string id = data->config_.id;
		std::shared_ptr<Camera> camera = Camera::create(std::move(data), id, streams);

		if (!initFrameGenerator(camera.get())) {
			LOG(Virtual, Error) << "Failed to initialize frame "
					    << "generator for camera: " << id;
			continue;
		}

		registerCamera(std::move(camera));
	}

	resetCreated_ = true;

	return true;
}

bool PipelineHandlerVirtual::initFrameGenerator(Camera *camera)
{
	auto data = cameraData(camera);
	auto &frame = data->config_.frame;
	std::visit(overloaded{
			   [&](TestPattern &testPattern) {
				   for (auto &streamConfig : data->streamConfigs_) {
					   if (testPattern == TestPattern::DiagonalLines)
						   streamConfig.frameGenerator = std::make_unique<DiagonalLinesGenerator>();
					   else
						   streamConfig.frameGenerator = std::make_unique<ColorBarsGenerator>();
				   }
			   },
			   [&](ImageFrames &imageFrames) {
				   for (auto &streamConfig : data->streamConfigs_)
					   streamConfig.frameGenerator = ImageFrameGenerator::create(imageFrames);
			   } },
		   frame);

	for (auto &streamConfig : data->streamConfigs_)
		if (!streamConfig.frameGenerator)
			return false;

	return true;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerVirtual, "virtual")

} /* namespace libcamera */
