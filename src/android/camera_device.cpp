/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_device.cpp - libcamera Android Camera Device
 */

#include "camera_device.h"

#include <algorithm>
#include <fstream>
#include <set>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/unique_fd.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/fence.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include "system/graphics.h"

#include "camera_buffer.h"
#include "camera_hal_config.h"
#include "camera_ops.h"
#include "camera_request.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

namespace {

/*
 * \struct Camera3StreamConfig
 * \brief Data to store StreamConfiguration associated with camera3_stream(s)
 * \var streams List of the pairs of a stream requested by Android HAL client
 * and CameraStream::Type associated with the stream
 * \var config StreamConfiguration for streams
 */
struct Camera3StreamConfig {
	struct Camera3Stream {
		camera3_stream_t *stream;
		CameraStream::Type type;
	};

	std::vector<Camera3Stream> streams;
	StreamConfiguration config;
};

/*
 * Reorder the configurations so that libcamera::Camera can accept them as much
 * as possible. The sort rule is as follows.
 * 1.) The configuration for NV12 request whose resolution is the largest.
 * 2.) The configuration for JPEG request.
 * 3.) Others. Larger resolutions and different formats are put earlier.
 */
void sortCamera3StreamConfigs(std::vector<Camera3StreamConfig> &unsortedConfigs,
			      const camera3_stream_t *jpegStream)
{
	const Camera3StreamConfig *jpegConfig = nullptr;

	std::map<PixelFormat, std::vector<const Camera3StreamConfig *>> formatToConfigs;
	for (const auto &streamConfig : unsortedConfigs) {
		if (jpegStream && !jpegConfig) {
			const auto &streams = streamConfig.streams;
			if (std::find_if(streams.begin(), streams.end(),
					 [jpegStream](const auto &stream) {
						 return stream.stream == jpegStream;
					 }) != streams.end()) {
				jpegConfig = &streamConfig;
				continue;
			}
		}
		formatToConfigs[streamConfig.config.pixelFormat].push_back(&streamConfig);
	}

	if (jpegStream && !jpegConfig)
		LOG(HAL, Fatal) << "No Camera3StreamConfig is found for JPEG";

	for (auto &fmt : formatToConfigs) {
		auto &streamConfigs = fmt.second;

		/* Sorted by resolution. Smaller is put first. */
		std::sort(streamConfigs.begin(), streamConfigs.end(),
			  [](const auto *streamConfigA, const auto *streamConfigB) {
				  const Size &sizeA = streamConfigA->config.size;
				  const Size &sizeB = streamConfigB->config.size;
				  return sizeA < sizeB;
			  });
	}

	std::vector<Camera3StreamConfig> sortedConfigs;
	sortedConfigs.reserve(unsortedConfigs.size());

	/*
	 * NV12 is the most prioritized format. Put the configuration with NV12
	 * and the largest resolution first.
	 */
	const auto nv12It = formatToConfigs.find(formats::NV12);
	if (nv12It != formatToConfigs.end()) {
		auto &nv12Configs = nv12It->second;
		const Camera3StreamConfig *nv12Largest = nv12Configs.back();

		/*
		 * If JPEG will be created from NV12 and the size is larger than
		 * the largest NV12 configurations, then put the NV12
		 * configuration for JPEG first.
		 */
		if (jpegConfig && jpegConfig->config.pixelFormat == formats::NV12) {
			const Size &nv12SizeForJpeg = jpegConfig->config.size;
			const Size &nv12LargestSize = nv12Largest->config.size;

			if (nv12LargestSize < nv12SizeForJpeg) {
				LOG(HAL, Debug) << "Insert " << jpegConfig->config.toString();
				sortedConfigs.push_back(std::move(*jpegConfig));
				jpegConfig = nullptr;
			}
		}

		LOG(HAL, Debug) << "Insert " << nv12Largest->config.toString();
		sortedConfigs.push_back(*nv12Largest);
		nv12Configs.pop_back();

		if (nv12Configs.empty())
			formatToConfigs.erase(nv12It);
	}

	/* If the configuration for JPEG is there, then put it. */
	if (jpegConfig) {
		LOG(HAL, Debug) << "Insert " << jpegConfig->config.toString();
		sortedConfigs.push_back(std::move(*jpegConfig));
		jpegConfig = nullptr;
	}

	/*
	 * Put configurations with different formats and larger resolutions
	 * earlier.
	 */
	while (!formatToConfigs.empty()) {
		for (auto it = formatToConfigs.begin(); it != formatToConfigs.end();) {
			auto &configs = it->second;
			LOG(HAL, Debug) << "Insert " << configs.back()->config.toString();
			sortedConfigs.push_back(*configs.back());
			configs.pop_back();

			if (configs.empty())
				it = formatToConfigs.erase(it);
			else
				it++;
		}
	}

	ASSERT(sortedConfigs.size() == unsortedConfigs.size());

	unsortedConfigs = sortedConfigs;
}

const char *rotationToString(int rotation)
{
	switch (rotation) {
	case CAMERA3_STREAM_ROTATION_0:
		return "0";
	case CAMERA3_STREAM_ROTATION_90:
		return "90";
	case CAMERA3_STREAM_ROTATION_180:
		return "180";
	case CAMERA3_STREAM_ROTATION_270:
		return "270";
	}
	return "INVALID";
}

const char *directionToString(int stream_type)
{
	switch (stream_type) {
	case CAMERA3_STREAM_OUTPUT:
		return "Output";
	case CAMERA3_STREAM_INPUT:
		return "Input";
	case CAMERA3_STREAM_BIDIRECTIONAL:
		return "Bidirectional";
	default:
		LOG(HAL, Warning) << "Unknown stream type: " << stream_type;
		return "Unknown";
	}
}

#if defined(OS_CHROMEOS)
/*
 * Check whether the crop_rotate_scale_degrees values for all streams in
 * the list are valid according to the Chrome OS camera HAL API.
 */
bool validateCropRotate(const camera3_stream_configuration_t &streamList)
{
	ASSERT(streamList.num_streams > 0);
	const int cropRotateScaleDegrees =
		streamList.streams[0]->crop_rotate_scale_degrees;
	for (unsigned int i = 0; i < streamList.num_streams; ++i) {
		const camera3_stream_t &stream = *streamList.streams[i];

		switch (stream.crop_rotate_scale_degrees) {
		case CAMERA3_STREAM_ROTATION_0:
		case CAMERA3_STREAM_ROTATION_90:
		case CAMERA3_STREAM_ROTATION_270:
			break;

		/* 180° rotation is specified by Chrome OS as invalid. */
		case CAMERA3_STREAM_ROTATION_180:
		default:
			LOG(HAL, Error) << "Invalid crop_rotate_scale_degrees: "
					<< stream.crop_rotate_scale_degrees;
			return false;
		}

		if (cropRotateScaleDegrees != stream.crop_rotate_scale_degrees) {
			LOG(HAL, Error) << "crop_rotate_scale_degrees in all "
					<< "streams are not identical";
			return false;
		}
	}

	return true;
}
#endif

} /* namespace */

/*
 * \class CameraDevice
 *
 * The CameraDevice class wraps a libcamera::Camera instance, and implements
 * the camera3_device_t interface, bridging calls received from the Android
 * camera service to the CameraDevice.
 *
 * The class translates parameters and operations from the Camera HALv3 API to
 * the libcamera API to provide static information for a Camera, create request
 * templates for it, process capture requests and then deliver capture results
 * back to the framework using the designated callbacks.
 */

CameraDevice::CameraDevice(unsigned int id, std::shared_ptr<Camera> camera)
	: id_(id), state_(State::Stopped), camera_(std::move(camera)),
	  facing_(CAMERA_FACING_FRONT), orientation_(0)
{
	camera_->requestCompleted.connect(this, &CameraDevice::requestComplete);

	maker_ = "libcamera";
	model_ = "cameraModel";

	/* \todo Support getting properties on Android */
	std::ifstream fstream("/var/cache/camera/camera.prop");
	if (!fstream.is_open())
		return;

	std::string line;
	while (std::getline(fstream, line)) {
		std::string::size_type delimPos = line.find("=");
		if (delimPos == std::string::npos)
			continue;
		std::string key = line.substr(0, delimPos);
		std::string val = line.substr(delimPos + 1);

		if (!key.compare("ro.product.model"))
			model_ = val;
		else if (!key.compare("ro.product.manufacturer"))
			maker_ = val;
	}
}

CameraDevice::~CameraDevice() = default;

std::unique_ptr<CameraDevice> CameraDevice::create(unsigned int id,
						   std::shared_ptr<Camera> cam)
{
	return std::unique_ptr<CameraDevice>(
		new CameraDevice(id, std::move(cam)));
}

/*
 * Initialize the camera static information retrieved from the
 * Camera::properties or from the cameraConfigData.
 *
 * cameraConfigData is optional for external camera devices and can be
 * nullptr.
 *
 * This function is called before the camera device is opened.
 */
int CameraDevice::initialize(const CameraConfigData *cameraConfigData)
{
	/*
	 * Initialize orientation and facing side of the camera.
	 *
	 * If the libcamera::Camera provides those information as retrieved
	 * from firmware use them, otherwise fallback to values parsed from
	 * the configuration file. If the configuration file is not available
	 * the camera is external so its location and rotation can be safely
	 * defaulted.
	 */
	const ControlList &properties = camera_->properties();

	const auto &location = properties.get(properties::Location);
	if (location) {
		switch (*location) {
		case properties::CameraLocationFront:
			facing_ = CAMERA_FACING_FRONT;
			break;
		case properties::CameraLocationBack:
			facing_ = CAMERA_FACING_BACK;
			break;
		case properties::CameraLocationExternal:
			/*
			 * If the camera is reported as external, but the
			 * CameraHalManager has overriden it, use what is
			 * reported in the configuration file. This typically
			 * happens for UVC cameras reported as 'External' by
			 * libcamera but installed in fixed position on the
			 * device.
			 */
			if (cameraConfigData && cameraConfigData->facing != -1)
				facing_ = cameraConfigData->facing;
			else
				facing_ = CAMERA_FACING_EXTERNAL;
			break;
		}

		if (cameraConfigData && cameraConfigData->facing != -1 &&
		    facing_ != cameraConfigData->facing) {
			LOG(HAL, Warning)
				<< "Camera location does not match"
				<< " configuration file. Using " << facing_;
		}
	} else if (cameraConfigData) {
		if (cameraConfigData->facing == -1) {
			LOG(HAL, Error)
				<< "Camera facing not in configuration file";
			return -EINVAL;
		}
		facing_ = cameraConfigData->facing;
	} else {
		facing_ = CAMERA_FACING_EXTERNAL;
	}

	/*
	 * The Android orientation metadata specifies its rotation correction
	 * value in clockwise direction whereas libcamera specifies the
	 * rotation property in anticlockwise direction. Read the libcamera's
	 * rotation property (anticlockwise) and compute the corresponding
	 * value for clockwise direction as required by the Android orientation
	 * metadata.
	 */
	const auto &rotation = properties.get(properties::Rotation);
	if (rotation) {
		orientation_ = (360 - *rotation) % 360;
		if (cameraConfigData && cameraConfigData->rotation != -1 &&
		    orientation_ != cameraConfigData->rotation) {
			LOG(HAL, Warning)
				<< "Camera orientation does not match"
				<< " configuration file. Using " << orientation_;
		}
	} else if (cameraConfigData) {
		if (cameraConfigData->rotation == -1) {
			LOG(HAL, Error)
				<< "Camera rotation not in configuration file";
			return -EINVAL;
		}
		orientation_ = cameraConfigData->rotation;
	} else {
		orientation_ = 0;
	}

	return capabilities_.initialize(camera_, orientation_, facing_);
}

/*
 * Open a camera device. The static information on the camera shall have been
 * initialized with a call to CameraDevice::initialize().
 */
int CameraDevice::open(const hw_module_t *hardwareModule)
{
	int ret = camera_->acquire();
	if (ret) {
		LOG(HAL, Error) << "Failed to acquire the camera";
		return ret;
	}

	/* Initialize the hw_device_t in the instance camera3_module_t. */
	camera3Device_.common.tag = HARDWARE_DEVICE_TAG;
	camera3Device_.common.version = CAMERA_DEVICE_API_VERSION_3_3;
	camera3Device_.common.module = (hw_module_t *)hardwareModule;
	camera3Device_.common.close = hal_dev_close;

	/*
	 * The camera device operations. These actually implement
	 * the Android Camera HALv3 interface.
	 */
	camera3Device_.ops = &hal_dev_ops;
	camera3Device_.priv = this;

	return 0;
}

void CameraDevice::close()
{
	stop();

	camera_->release();
}

void CameraDevice::flush()
{
	{
		MutexLocker stateLock(stateMutex_);
		if (state_ != State::Running)
			return;

		state_ = State::Flushing;
	}

	camera_->stop();

	MutexLocker stateLock(stateMutex_);
	state_ = State::Stopped;
}

void CameraDevice::stop()
{
	MutexLocker stateLock(stateMutex_);
	if (state_ == State::Stopped)
		return;

	camera_->stop();

	{
		MutexLocker descriptorsLock(descriptorsMutex_);
		descriptors_ = {};
	}

	streams_.clear();

	state_ = State::Stopped;
}

unsigned int CameraDevice::maxJpegBufferSize() const
{
	return capabilities_.maxJpegBufferSize();
}

void CameraDevice::setCallbacks(const camera3_callback_ops_t *callbacks)
{
	callbacks_ = callbacks;
}

const camera_metadata_t *CameraDevice::getStaticMetadata()
{
	return capabilities_.staticMetadata()->getMetadata();
}

/*
 * Produce a metadata pack to be used as template for a capture request.
 */
const camera_metadata_t *CameraDevice::constructDefaultRequestSettings(int type)
{
	auto it = requestTemplates_.find(type);
	if (it != requestTemplates_.end())
		return it->second->getMetadata();

	/* Use the capture intent matching the requested template type. */
	std::unique_ptr<CameraMetadata> requestTemplate;
	uint8_t captureIntent;
	switch (type) {
	case CAMERA3_TEMPLATE_PREVIEW:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
		requestTemplate = capabilities_.requestTemplatePreview();
		break;
	case CAMERA3_TEMPLATE_STILL_CAPTURE:
		/*
		 * Use the preview template for still capture, they only differ
		 * for the torch mode we currently do not support.
		 */
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE;
		requestTemplate = capabilities_.requestTemplateStill();
		break;
	case CAMERA3_TEMPLATE_VIDEO_RECORD:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD;
		requestTemplate = capabilities_.requestTemplateVideo();
		break;
	case CAMERA3_TEMPLATE_VIDEO_SNAPSHOT:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT;
		requestTemplate = capabilities_.requestTemplateVideo();
		break;
	case CAMERA3_TEMPLATE_MANUAL:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_MANUAL;
		requestTemplate = capabilities_.requestTemplateManual();
		break;
	/* \todo Implement templates generation for the remaining use cases. */
	case CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG:
	default:
		LOG(HAL, Error) << "Unsupported template request type: " << type;
		return nullptr;
	}

	if (!requestTemplate || !requestTemplate->isValid()) {
		LOG(HAL, Error) << "Failed to construct request template";
		return nullptr;
	}

	requestTemplate->updateEntry(ANDROID_CONTROL_CAPTURE_INTENT,
				     captureIntent);

	requestTemplates_[type] = std::move(requestTemplate);
	return requestTemplates_[type]->getMetadata();
}

/*
 * Inspect the stream_list to produce a list of StreamConfiguration to
 * be use to configure the Camera.
 */
int CameraDevice::configureStreams(camera3_stream_configuration_t *stream_list)
{
	/* Before any configuration attempt, stop the camera. */
	stop();

	if (stream_list->num_streams == 0) {
		LOG(HAL, Error) << "No streams in configuration";
		return -EINVAL;
	}

#if defined(OS_CHROMEOS)
	if (!validateCropRotate(*stream_list))
		return -EINVAL;
#endif

	/*
	 * Generate an empty configuration, and construct a StreamConfiguration
	 * for each camera3_stream to add to it.
	 */
	std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration();
	if (!config) {
		LOG(HAL, Error) << "Failed to generate camera configuration";
		return -EINVAL;
	}

	/*
	 * Clear and remove any existing configuration from previous calls, and
	 * ensure the required entries are available without further
	 * reallocation.
	 */
	streams_.clear();
	streams_.reserve(stream_list->num_streams);

	std::vector<Camera3StreamConfig> streamConfigs;
	streamConfigs.reserve(stream_list->num_streams);

	/* First handle all non-MJPEG streams. */
	camera3_stream_t *jpegStream = nullptr;
	for (unsigned int i = 0; i < stream_list->num_streams; ++i) {
		camera3_stream_t *stream = stream_list->streams[i];
		Size size(stream->width, stream->height);

		PixelFormat format = capabilities_.toPixelFormat(stream->format);

		LOG(HAL, Info) << "Stream #" << i
			       << ", direction: " << directionToString(stream->stream_type)
			       << ", width: " << stream->width
			       << ", height: " << stream->height
			       << ", format: " << utils::hex(stream->format)
			       << ", rotation: " << rotationToString(stream->rotation)
#if defined(OS_CHROMEOS)
			       << ", crop_rotate_scale_degrees: "
			       << rotationToString(stream->crop_rotate_scale_degrees)
#endif
			       << " (" << format << ")";

		if (!format.isValid())
			return -EINVAL;

		/* \todo Support rotation. */
		if (stream->rotation != CAMERA3_STREAM_ROTATION_0) {
			LOG(HAL, Error) << "Rotation is not supported";
			return -EINVAL;
		}
#if defined(OS_CHROMEOS)
		if (stream->crop_rotate_scale_degrees != CAMERA3_STREAM_ROTATION_0) {
			LOG(HAL, Error) << "Rotation is not supported";
			return -EINVAL;
		}
#endif

		/* Defer handling of MJPEG streams until all others are known. */
		if (stream->format == HAL_PIXEL_FORMAT_BLOB) {
			if (jpegStream) {
				LOG(HAL, Error)
					<< "Multiple JPEG streams are not supported";
				return -EINVAL;
			}

			jpegStream = stream;
			continue;
		}

		/*
		 * While gralloc usage flags are supposed to report usage
		 * patterns to select a suitable buffer allocation strategy, in
		 * practice they're also used to make other decisions, such as
		 * selecting the actual format for the IMPLEMENTATION_DEFINED
		 * HAL pixel format. To avoid issues, we thus have to set the
		 * GRALLOC_USAGE_HW_CAMERA_WRITE flag unconditionally, even for
		 * streams that will be produced in software.
		 */
		stream->usage |= GRALLOC_USAGE_HW_CAMERA_WRITE;

		/*
		 * If a CameraStream with the same size and format as the
		 * current stream has already been requested, associate the two.
		 */
		auto iter = std::find_if(
			streamConfigs.begin(), streamConfigs.end(),
			[&size, &format](const Camera3StreamConfig &streamConfig) {
				return streamConfig.config.size == size &&
				       streamConfig.config.pixelFormat == format;
			});
		if (iter != streamConfigs.end()) {
			/* Add usage to copy the buffer in streams[0] to stream. */
			iter->streams[0].stream->usage |= GRALLOC_USAGE_SW_READ_OFTEN;
			stream->usage |= GRALLOC_USAGE_SW_WRITE_OFTEN;
			iter->streams.push_back({ stream, CameraStream::Type::Mapped });
			continue;
		}

		Camera3StreamConfig streamConfig;
		streamConfig.streams = { { stream, CameraStream::Type::Direct } };
		streamConfig.config.size = size;
		streamConfig.config.pixelFormat = format;
		streamConfigs.push_back(std::move(streamConfig));
	}

	/* Now handle the MJPEG streams, adding a new stream if required. */
	if (jpegStream) {
		CameraStream::Type type;
		int index = -1;

		/* Search for a compatible stream in the non-JPEG ones. */
		for (size_t i = 0; i < streamConfigs.size(); ++i) {
			Camera3StreamConfig &streamConfig = streamConfigs[i];
			const auto &cfg = streamConfig.config;

			/*
			 * \todo The PixelFormat must also be compatible with
			 * the encoder.
			 */
			if (cfg.size.width != jpegStream->width ||
			    cfg.size.height != jpegStream->height)
				continue;

			LOG(HAL, Info)
				<< "Android JPEG stream mapped to libcamera stream " << i;

			type = CameraStream::Type::Mapped;
			index = i;

			/*
			 * The source stream will be read by software to
			 * produce the JPEG stream.
			 */
			camera3_stream_t *stream = streamConfig.streams[0].stream;
			stream->usage |= GRALLOC_USAGE_SW_READ_OFTEN;
			break;
		}

		/*
		 * Without a compatible match for JPEG encoding we must
		 * introduce a new stream to satisfy the request requirements.
		 */
		if (index < 0) {
			/*
			 * \todo The pixelFormat should be a 'best-fit' choice
			 * and may require a validation cycle. This is not yet
			 * handled, and should be considered as part of any
			 * stream configuration reworks.
			 */
			Camera3StreamConfig streamConfig;
			streamConfig.config.size.width = jpegStream->width;
			streamConfig.config.size.height = jpegStream->height;
			streamConfig.config.pixelFormat = formats::NV12;
			streamConfigs.push_back(std::move(streamConfig));

			LOG(HAL, Info) << "Adding " << streamConfig.config.toString()
				       << " for MJPEG support";

			type = CameraStream::Type::Internal;
			index = streamConfigs.size() - 1;
		}

		/* The JPEG stream will be produced by software. */
		jpegStream->usage |= GRALLOC_USAGE_SW_WRITE_OFTEN;

		streamConfigs[index].streams.push_back({ jpegStream, type });
	}

	sortCamera3StreamConfigs(streamConfigs, jpegStream);
	for (const auto &streamConfig : streamConfigs) {
		config->addConfiguration(streamConfig.config);

		CameraStream *sourceStream = nullptr;
		for (auto &stream : streamConfig.streams) {
			streams_.emplace_back(this, config.get(), stream.type,
					      stream.stream, sourceStream,
					      config->size() - 1);
			stream.stream->priv = static_cast<void *>(&streams_.back());

			/*
			 * The streamConfig.streams vector contains as its first
			 * element a Direct (or Internal) stream, and then an
			 * optional set of Mapped streams derived from the
			 * Direct stream. Cache the Direct stream pointer, to
			 * be used when constructing the subsequent mapped
			 * streams.
			 */
			if (stream.type == CameraStream::Type::Direct)
				sourceStream = &streams_.back();
		}
	}

	switch (config->validate()) {
	case CameraConfiguration::Valid:
		break;
	case CameraConfiguration::Adjusted:
		LOG(HAL, Info) << "Camera configuration adjusted";

		for (const StreamConfiguration &cfg : *config)
			LOG(HAL, Info) << " - " << cfg.toString();

		return -EINVAL;
	case CameraConfiguration::Invalid:
		LOG(HAL, Info) << "Camera configuration invalid";
		return -EINVAL;
	}

	/*
	 * Once the CameraConfiguration has been adjusted/validated
	 * it can be applied to the camera.
	 */
	int ret = camera_->configure(config.get());
	if (ret) {
		LOG(HAL, Error) << "Failed to configure camera '"
				<< camera_->id() << "'";
		return ret;
	}

	/*
	 * Configure the HAL CameraStream instances using the associated
	 * StreamConfiguration and set the number of required buffers in
	 * the Android camera3_stream_t.
	 */
	for (CameraStream &cameraStream : streams_) {
		ret = cameraStream.configure();
		if (ret) {
			LOG(HAL, Error) << "Failed to configure camera stream";
			return ret;
		}
	}

	config_ = std::move(config);
	return 0;
}

std::unique_ptr<FrameBuffer>
CameraDevice::createFrameBuffer(const buffer_handle_t camera3buffer,
				PixelFormat pixelFormat, const Size &size)
{
	CameraBuffer buf(camera3buffer, pixelFormat, size, PROT_READ);
	if (!buf.isValid()) {
		LOG(HAL, Fatal) << "Failed to create CameraBuffer";
		return nullptr;
	}

	std::vector<FrameBuffer::Plane> planes(buf.numPlanes());
	for (size_t i = 0; i < buf.numPlanes(); ++i) {
		SharedFD fd{ camera3buffer->data[i] };
		if (!fd.isValid()) {
			LOG(HAL, Fatal) << "No valid fd";
			return nullptr;
		}

		planes[i].fd = fd;
		planes[i].offset = buf.offset(i);
		planes[i].length = buf.size(i);
	}

	return std::make_unique<FrameBuffer>(planes);
}

int CameraDevice::processControls(Camera3RequestDescriptor *descriptor)
{
	const CameraMetadata &settings = descriptor->settings_;
	if (!settings.isValid())
		return 0;

	/* Translate the Android request settings to libcamera controls. */
	ControlList &controls = descriptor->request_->controls();
	camera_metadata_ro_entry_t entry;
	if (settings.getEntry(ANDROID_SCALER_CROP_REGION, &entry)) {
		const int32_t *data = entry.data.i32;
		Rectangle cropRegion{ data[0], data[1],
				      static_cast<unsigned int>(data[2]),
				      static_cast<unsigned int>(data[3]) };
		controls.set(controls::ScalerCrop, cropRegion);
	}

	if (settings.getEntry(ANDROID_SENSOR_TEST_PATTERN_MODE, &entry)) {
		const int32_t data = *entry.data.i32;
		int32_t testPatternMode = controls::draft::TestPatternModeOff;
		switch (data) {
		case ANDROID_SENSOR_TEST_PATTERN_MODE_OFF:
			testPatternMode = controls::draft::TestPatternModeOff;
			break;

		case ANDROID_SENSOR_TEST_PATTERN_MODE_SOLID_COLOR:
			testPatternMode = controls::draft::TestPatternModeSolidColor;
			break;

		case ANDROID_SENSOR_TEST_PATTERN_MODE_COLOR_BARS:
			testPatternMode = controls::draft::TestPatternModeColorBars;
			break;

		case ANDROID_SENSOR_TEST_PATTERN_MODE_COLOR_BARS_FADE_TO_GRAY:
			testPatternMode = controls::draft::TestPatternModeColorBarsFadeToGray;
			break;

		case ANDROID_SENSOR_TEST_PATTERN_MODE_PN9:
			testPatternMode = controls::draft::TestPatternModePn9;
			break;

		case ANDROID_SENSOR_TEST_PATTERN_MODE_CUSTOM1:
			testPatternMode = controls::draft::TestPatternModeCustom1;
			break;

		default:
			LOG(HAL, Error)
				<< "Unknown test pattern mode: " << data;

			return -EINVAL;
		}

		controls.set(controls::draft::TestPatternMode, testPatternMode);
	}

	return 0;
}

void CameraDevice::abortRequest(Camera3RequestDescriptor *descriptor) const
{
	notifyError(descriptor->frameNumber_, nullptr, CAMERA3_MSG_ERROR_REQUEST);

	for (auto &buffer : descriptor->buffers_)
		buffer.status = Camera3RequestDescriptor::Status::Error;

	descriptor->status_ = Camera3RequestDescriptor::Status::Error;
}

bool CameraDevice::isValidRequest(camera3_capture_request_t *camera3Request) const
{
	if (!camera3Request) {
		LOG(HAL, Error) << "No capture request provided";
		return false;
	}

	if (!camera3Request->num_output_buffers ||
	    !camera3Request->output_buffers) {
		LOG(HAL, Error) << "No output buffers provided";
		return false;
	}

	/* configureStreams() has not been called or has failed. */
	if (streams_.empty() || !config_) {
		LOG(HAL, Error) << "No stream is configured";
		return false;
	}

	for (uint32_t i = 0; i < camera3Request->num_output_buffers; i++) {
		const camera3_stream_buffer_t &outputBuffer =
			camera3Request->output_buffers[i];
		if (!outputBuffer.buffer || !(*outputBuffer.buffer)) {
			LOG(HAL, Error) << "Invalid native handle";
			return false;
		}

		const native_handle_t *handle = *outputBuffer.buffer;
		constexpr int kNativeHandleMaxFds = 1024;
		if (handle->numFds < 0 || handle->numFds > kNativeHandleMaxFds) {
			LOG(HAL, Error)
				<< "Invalid number of fds (" << handle->numFds
				<< ") in buffer " << i;
			return false;
		}

		constexpr int kNativeHandleMaxInts = 1024;
		if (handle->numInts < 0 || handle->numInts > kNativeHandleMaxInts) {
			LOG(HAL, Error)
				<< "Invalid number of ints (" << handle->numInts
				<< ") in buffer " << i;
			return false;
		}

		const camera3_stream *camera3Stream = outputBuffer.stream;
		if (!camera3Stream)
			return false;

		const CameraStream *cameraStream =
			static_cast<CameraStream *>(camera3Stream->priv);

		auto found = std::find_if(streams_.begin(), streams_.end(),
					  [cameraStream](const CameraStream &stream) {
						  return &stream == cameraStream;
					  });
		if (found == streams_.end()) {
			LOG(HAL, Error)
				<< "No corresponding configured stream found";
			return false;
		}
	}

	return true;
}

int CameraDevice::processCaptureRequest(camera3_capture_request_t *camera3Request)
{
	if (!isValidRequest(camera3Request))
		return -EINVAL;

	/*
	 * Save the request descriptors for use at completion time.
	 * The descriptor and the associated memory reserved here are freed
	 * at request complete time.
	 */
	auto descriptor = std::make_unique<Camera3RequestDescriptor>(camera_.get(),
								     camera3Request);

	/*
	 * \todo The Android request model is incremental, settings passed in
	 * previous requests are to be effective until overridden explicitly in
	 * a new request. Do we need to cache settings incrementally here, or is
	 * it handled by the Android camera service ?
	 */
	if (camera3Request->settings)
		lastSettings_ = camera3Request->settings;
	else
		descriptor->settings_ = lastSettings_;

	LOG(HAL, Debug) << "Queueing request " << descriptor->request_->cookie()
			<< " with " << descriptor->buffers_.size() << " streams";

	/*
	 * Process all the Direct and Internal streams first, they map directly
	 * to a libcamera stream. Streams of type Mapped will be handled later.
	 *
	 * Collect the CameraStream associated to each requested capture stream.
	 * Since requestedStreams is an std:set<>, no duplications can happen.
	 */
	std::set<CameraStream *> requestedStreams;
	for (const auto &[i, buffer] : utils::enumerate(descriptor->buffers_)) {
		CameraStream *cameraStream = buffer.stream;
		camera3_stream_t *camera3Stream = cameraStream->camera3Stream();

		std::stringstream ss;
		ss << i << " - (" << camera3Stream->width << "x"
		   << camera3Stream->height << ")"
		   << "[" << utils::hex(camera3Stream->format) << "] -> "
		   << "(" << cameraStream->configuration().size << ")["
		   << cameraStream->configuration().pixelFormat << "]";

		/*
		 * Inspect the camera stream type, create buffers opportunely
		 * and add them to the Request if required.
		 */
		FrameBuffer *frameBuffer = nullptr;
		UniqueFD acquireFence;

		MutexLocker lock(descriptor->streamsProcessMutex_);

		switch (cameraStream->type()) {
		case CameraStream::Type::Mapped:
			/* Mapped streams will be handled in the next loop. */
			continue;

		case CameraStream::Type::Direct:
			/*
			 * Create a libcamera buffer using the dmabuf
			 * descriptors of the camera3Buffer for each stream and
			 * associate it with the Camera3RequestDescriptor for
			 * lifetime management only.
			 */
			buffer.frameBuffer =
				createFrameBuffer(*buffer.camera3Buffer,
						  cameraStream->configuration().pixelFormat,
						  cameraStream->configuration().size);
			frameBuffer = buffer.frameBuffer.get();
			acquireFence = std::move(buffer.fence);
			LOG(HAL, Debug) << ss.str() << " (direct)";
			break;

		case CameraStream::Type::Internal:
			/*
			 * Get the frame buffer from the CameraStream internal
			 * buffer pool.
			 *
			 * The buffer has to be returned to the CameraStream
			 * once it has been processed.
			 */
			frameBuffer = cameraStream->getBuffer();
			buffer.internalBuffer = frameBuffer;
			LOG(HAL, Debug) << ss.str() << " (internal)";

			descriptor->pendingStreamsToProcess_.insert(
				{ cameraStream, &buffer });
			break;
		}

		if (!frameBuffer) {
			LOG(HAL, Error) << "Failed to create frame buffer";
			return -ENOMEM;
		}

		auto fence = std::make_unique<Fence>(std::move(acquireFence));
		descriptor->request_->addBuffer(cameraStream->stream(),
						frameBuffer, std::move(fence));

		requestedStreams.insert(cameraStream);
	}

	/*
	 * Now handle the Mapped streams. If no buffer has been added for them
	 * because their corresponding direct source stream is not part of this
	 * particular request, add one here.
	 */
	for (const auto &[i, buffer] : utils::enumerate(descriptor->buffers_)) {
		CameraStream *cameraStream = buffer.stream;
		camera3_stream_t *camera3Stream = cameraStream->camera3Stream();

		if (cameraStream->type() != CameraStream::Type::Mapped)
			continue;

		LOG(HAL, Debug) << i << " - (" << camera3Stream->width << "x"
				<< camera3Stream->height << ")"
				<< "[" << utils::hex(camera3Stream->format) << "] -> "
				<< "(" << cameraStream->configuration().size << ")["
				<< cameraStream->configuration().pixelFormat << "]"
				<< " (mapped)";

		MutexLocker lock(descriptor->streamsProcessMutex_);
		descriptor->pendingStreamsToProcess_.insert({ cameraStream, &buffer });

		/*
		 * Make sure the CameraStream this stream is mapped on has been
		 * added to the request.
		 */
		CameraStream *sourceStream = cameraStream->sourceStream();
		ASSERT(sourceStream);
		if (requestedStreams.find(sourceStream) != requestedStreams.end())
			continue;

		/*
		 * If that's not the case, we need to add a buffer to the request
		 * for this stream.
		 */
		FrameBuffer *frameBuffer = cameraStream->getBuffer();
		buffer.internalBuffer = frameBuffer;

		descriptor->request_->addBuffer(sourceStream->stream(),
						frameBuffer, nullptr);

		requestedStreams.erase(sourceStream);
	}

	/*
	 * Translate controls from Android to libcamera and queue the request
	 * to the camera.
	 */
	int ret = processControls(descriptor.get());
	if (ret)
		return ret;

	/*
	 * If flush is in progress set the request status to error and place it
	 * on the queue to be later completed. If the camera has been stopped we
	 * have to re-start it to be able to process the request.
	 */
	MutexLocker stateLock(stateMutex_);

	if (state_ == State::Flushing) {
		Camera3RequestDescriptor *rawDescriptor = descriptor.get();
		{
			MutexLocker descriptorsLock(descriptorsMutex_);
			descriptors_.push(std::move(descriptor));
		}
		abortRequest(rawDescriptor);
		completeDescriptor(rawDescriptor);

		return 0;
	}

	if (state_ == State::Stopped) {
		ret = camera_->start();
		if (ret) {
			LOG(HAL, Error) << "Failed to start camera";
			return ret;
		}

		state_ = State::Running;
	}

	Request *request = descriptor->request_.get();

	{
		MutexLocker descriptorsLock(descriptorsMutex_);
		descriptors_.push(std::move(descriptor));
	}

	camera_->queueRequest(request);

	return 0;
}

void CameraDevice::requestComplete(Request *request)
{
	Camera3RequestDescriptor *descriptor =
		reinterpret_cast<Camera3RequestDescriptor *>(request->cookie());

	/*
	 * Prepare the capture result for the Android camera stack.
	 *
	 * The buffer status is set to Success and later changed to Error if
	 * post-processing/compression fails.
	 */
	for (auto &buffer : descriptor->buffers_) {
		CameraStream *stream = buffer.stream;

		/*
		 * Streams of type Direct have been queued to the
		 * libcamera::Camera and their acquire fences have
		 * already been waited on by the library.
		 *
		 * Acquire fences of streams of type Internal and Mapped
		 * will be handled during post-processing.
		 */
		if (stream->type() == CameraStream::Type::Direct) {
			/* If handling of the fence has failed restore buffer.fence. */
			std::unique_ptr<Fence> fence = buffer.frameBuffer->releaseFence();
			if (fence)
				buffer.fence = fence->release();
		}
		buffer.status = Camera3RequestDescriptor::Status::Success;
	}

	/*
	 * If the Request has failed, abort the request by notifying the error
	 * and complete the request with all buffers in error state.
	 */
	if (request->status() != Request::RequestComplete) {
		LOG(HAL, Error) << "Request " << request->cookie()
				<< " not successfully completed: "
				<< request->status();

		abortRequest(descriptor);
		completeDescriptor(descriptor);

		return;
	}

	/*
	 * Notify shutter as soon as we have verified we have a valid request.
	 *
	 * \todo The shutter event notification should be sent to the framework
	 * as soon as possible, earlier than request completion time.
	 */
	uint64_t sensorTimestamp = static_cast<uint64_t>(request->metadata()
								 .get(controls::SensorTimestamp)
								 .value_or(0));
	notifyShutter(descriptor->frameNumber_, sensorTimestamp);

	LOG(HAL, Debug) << "Request " << request->cookie() << " completed with "
			<< descriptor->request_->buffers().size() << " streams";

	/*
	 * Generate the metadata associated with the captured buffers.
	 *
	 * Notify if the metadata generation has failed, but continue processing
	 * buffers and return an empty metadata pack.
	 */
	descriptor->resultMetadata_ = getResultMetadata(*descriptor);
	if (!descriptor->resultMetadata_) {
		notifyError(descriptor->frameNumber_, nullptr, CAMERA3_MSG_ERROR_RESULT);

		/*
		 * The camera framework expects an empty metadata pack on error.
		 *
		 * \todo Check that the post-processor code handles this situation
		 * correctly.
		 */
		descriptor->resultMetadata_ = std::make_unique<CameraMetadata>(0, 0);
	}

	/* Handle post-processing. */
	MutexLocker locker(descriptor->streamsProcessMutex_);

	/*
	 * Queue all the post-processing streams request at once. The completion
	 * slot streamProcessingComplete() can only execute when we are out
	 * this critical section. This helps to handle synchronous errors here
	 * itself.
	 */
	auto iter = descriptor->pendingStreamsToProcess_.begin();
	while (iter != descriptor->pendingStreamsToProcess_.end()) {
		CameraStream *stream = iter->first;
		Camera3RequestDescriptor::StreamBuffer *buffer = iter->second;

		FrameBuffer *src = request->findBuffer(stream->stream());
		if (!src) {
			LOG(HAL, Error) << "Failed to find a source stream buffer";
			setBufferStatus(*buffer, Camera3RequestDescriptor::Status::Error);
			iter = descriptor->pendingStreamsToProcess_.erase(iter);
			continue;
		}

		buffer->srcBuffer = src;

		++iter;
		int ret = stream->process(buffer);
		if (ret) {
			setBufferStatus(*buffer, Camera3RequestDescriptor::Status::Error);
			descriptor->pendingStreamsToProcess_.erase(stream);

			/*
			 * If the framebuffer is internal to CameraStream return
			 * it back now that we're done processing it.
			 */
			if (buffer->internalBuffer)
				stream->putBuffer(buffer->internalBuffer);
		}
	}

	if (descriptor->pendingStreamsToProcess_.empty()) {
		locker.unlock();
		completeDescriptor(descriptor);
	}
}

/**
 * \brief Complete the Camera3RequestDescriptor
 * \param[in] descriptor The Camera3RequestDescriptor that has completed
 *
 * The function marks the Camera3RequestDescriptor as 'complete'. It shall be
 * called when all the streams in the Camera3RequestDescriptor have completed
 * capture (or have been generated via post-processing) and the request is ready
 * to be sent back to the framework.
 *
 * \context This function is \threadsafe.
 */
void CameraDevice::completeDescriptor(Camera3RequestDescriptor *descriptor)
{
	MutexLocker lock(descriptorsMutex_);
	descriptor->complete_ = true;

	sendCaptureResults();
}

/**
 * \brief Sequentially send capture results to the framework
 *
 * Iterate over the descriptors queue to send completed descriptors back to the
 * framework, in the same order as they have been queued. For each complete
 * descriptor, populate a locally-scoped camera3_capture_result_t from the
 * descriptor, send the capture result back by calling the
 * process_capture_result() callback, and remove the descriptor from the queue.
 * Stop iterating if the descriptor at the front of the queue is not complete.
 *
 * This function should never be called directly in the codebase. Use
 * completeDescriptor() instead.
 */
void CameraDevice::sendCaptureResults()
{
	while (!descriptors_.empty() && !descriptors_.front()->isPending()) {
		auto descriptor = std::move(descriptors_.front());
		descriptors_.pop();

		camera3_capture_result_t captureResult = {};

		captureResult.frame_number = descriptor->frameNumber_;

		if (descriptor->resultMetadata_)
			captureResult.result =
				descriptor->resultMetadata_->getMetadata();

		std::vector<camera3_stream_buffer_t> resultBuffers;
		resultBuffers.reserve(descriptor->buffers_.size());

		for (auto &buffer : descriptor->buffers_) {
			camera3_buffer_status status = CAMERA3_BUFFER_STATUS_ERROR;

			if (buffer.status == Camera3RequestDescriptor::Status::Success)
				status = CAMERA3_BUFFER_STATUS_OK;

			/*
			 * Pass the buffer fence back to the camera framework as
			 * a release fence. This instructs the framework to wait
			 * on the acquire fence in case we haven't done so
			 * ourselves for any reason.
			 */
			resultBuffers.push_back({ buffer.stream->camera3Stream(),
						  buffer.camera3Buffer, status,
						  -1, buffer.fence.release() });
		}

		captureResult.num_output_buffers = resultBuffers.size();
		captureResult.output_buffers = resultBuffers.data();

		if (descriptor->status_ == Camera3RequestDescriptor::Status::Success)
			captureResult.partial_result = 1;

		callbacks_->process_capture_result(callbacks_, &captureResult);
	}
}

void CameraDevice::setBufferStatus(Camera3RequestDescriptor::StreamBuffer &streamBuffer,
				   Camera3RequestDescriptor::Status status)
{
	streamBuffer.status = status;
	if (status != Camera3RequestDescriptor::Status::Success) {
		notifyError(streamBuffer.request->frameNumber_,
			    streamBuffer.stream->camera3Stream(),
			    CAMERA3_MSG_ERROR_BUFFER);

		/* Also set error status on entire request descriptor. */
		streamBuffer.request->status_ =
			Camera3RequestDescriptor::Status::Error;
	}
}

/**
 * \brief Handle post-processing completion of a stream in a capture request
 * \param[in] streamBuffer The StreamBuffer for which processing is complete
 * \param[in] status Stream post-processing status
 *
 * This function is called from the post-processor's thread whenever a camera
 * stream has finished post processing. The corresponding entry is dropped from
 * the descriptor's pendingStreamsToProcess_ map.
 *
 * If the pendingStreamsToProcess_ map is then empty, all streams requiring to
 * be generated from post-processing have been completed. Mark the descriptor as
 * complete using completeDescriptor() in that case.
 */
void CameraDevice::streamProcessingComplete(Camera3RequestDescriptor::StreamBuffer *streamBuffer,
					    Camera3RequestDescriptor::Status status)
{
	setBufferStatus(*streamBuffer, status);

	/*
	 * If the framebuffer is internal to CameraStream return it back now
	 * that we're done processing it.
	 */
	if (streamBuffer->internalBuffer)
		streamBuffer->stream->putBuffer(streamBuffer->internalBuffer);

	Camera3RequestDescriptor *request = streamBuffer->request;

	{
		MutexLocker locker(request->streamsProcessMutex_);

		request->pendingStreamsToProcess_.erase(streamBuffer->stream);
		if (!request->pendingStreamsToProcess_.empty())
			return;
	}

	completeDescriptor(streamBuffer->request);
}

std::string CameraDevice::logPrefix() const
{
	return "'" + camera_->id() + "'";
}

void CameraDevice::notifyShutter(uint32_t frameNumber, uint64_t timestamp)
{
	camera3_notify_msg_t notify = {};

	notify.type = CAMERA3_MSG_SHUTTER;
	notify.message.shutter.frame_number = frameNumber;
	notify.message.shutter.timestamp = timestamp;

	callbacks_->notify(callbacks_, &notify);
}

void CameraDevice::notifyError(uint32_t frameNumber, camera3_stream_t *stream,
			       camera3_error_msg_code code) const
{
	camera3_notify_msg_t notify = {};

	notify.type = CAMERA3_MSG_ERROR;
	notify.message.error.error_stream = stream;
	notify.message.error.frame_number = frameNumber;
	notify.message.error.error_code = code;

	callbacks_->notify(callbacks_, &notify);
}

/*
 * Produce a set of fixed result metadata.
 */
std::unique_ptr<CameraMetadata>
CameraDevice::getResultMetadata(const Camera3RequestDescriptor &descriptor) const
{
	const ControlList &metadata = descriptor.request_->metadata();
	const CameraMetadata &settings = descriptor.settings_;
	camera_metadata_ro_entry_t entry;
	bool found;

	/*
	 * \todo Keep this in sync with the actual number of entries.
	 * Currently: 40 entries, 156 bytes
	 *
	 * Reserve more space for the JPEG metadata set by the post-processor.
	 * Currently:
	 * ANDROID_JPEG_GPS_COORDINATES (double x 3) = 24 bytes
	 * ANDROID_JPEG_GPS_PROCESSING_METHOD (byte x 32) = 32 bytes
	 * ANDROID_JPEG_GPS_TIMESTAMP (int64) = 8 bytes
	 * ANDROID_JPEG_SIZE (int32_t) = 4 bytes
	 * ANDROID_JPEG_QUALITY (byte) = 1 byte
	 * ANDROID_JPEG_ORIENTATION (int32_t) = 4 bytes
	 * ANDROID_JPEG_THUMBNAIL_QUALITY (byte) = 1 byte
	 * ANDROID_JPEG_THUMBNAIL_SIZE (int32 x 2) = 8 bytes
	 * Total bytes for JPEG metadata: 82
	 */
	std::unique_ptr<CameraMetadata> resultMetadata =
		std::make_unique<CameraMetadata>(88, 166);
	if (!resultMetadata->isValid()) {
		LOG(HAL, Error) << "Failed to allocate result metadata";
		return nullptr;
	}

	/*
	 * \todo The value of the results metadata copied from the settings
	 * will have to be passed to the libcamera::Camera and extracted
	 * from libcamera::Request::metadata.
	 */

	uint8_t value = ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF;
	resultMetadata->addEntry(ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
				 value);

	value = ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_ANTIBANDING_MODE, value);

	int32_t value32 = 0;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
				 value32);

	value = ANDROID_CONTROL_AE_LOCK_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_LOCK, value);

	value = ANDROID_CONTROL_AE_MODE_ON;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_MODE, value);

	if (settings.getEntry(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, &entry))
		/*
		 * \todo Retrieve the AE FPS range from the libcamera metadata.
		 * As libcamera does not support that control, as a temporary
		 * workaround return what the framework asked.
		 */
		resultMetadata->addEntry(ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
					 entry.data.i32, 2);

	found = settings.getEntry(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, &entry);
	value = found ? *entry.data.u8 :
			(uint8_t)ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, value);

	value = ANDROID_CONTROL_AE_STATE_CONVERGED;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_STATE, value);

	value = ANDROID_CONTROL_AF_MODE_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_AF_MODE, value);

	value = ANDROID_CONTROL_AF_STATE_INACTIVE;
	resultMetadata->addEntry(ANDROID_CONTROL_AF_STATE, value);

	value = ANDROID_CONTROL_AF_TRIGGER_IDLE;
	resultMetadata->addEntry(ANDROID_CONTROL_AF_TRIGGER, value);

	value = ANDROID_CONTROL_AWB_MODE_AUTO;
	resultMetadata->addEntry(ANDROID_CONTROL_AWB_MODE, value);

	value = ANDROID_CONTROL_AWB_LOCK_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_AWB_LOCK, value);

	value = ANDROID_CONTROL_AWB_STATE_CONVERGED;
	resultMetadata->addEntry(ANDROID_CONTROL_AWB_STATE, value);

	value = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
	resultMetadata->addEntry(ANDROID_CONTROL_CAPTURE_INTENT, value);

	value = ANDROID_CONTROL_EFFECT_MODE_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_EFFECT_MODE, value);

	value = ANDROID_CONTROL_MODE_AUTO;
	resultMetadata->addEntry(ANDROID_CONTROL_MODE, value);

	value = ANDROID_CONTROL_SCENE_MODE_DISABLED;
	resultMetadata->addEntry(ANDROID_CONTROL_SCENE_MODE, value);

	value = ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE, value);

	value = ANDROID_FLASH_MODE_OFF;
	resultMetadata->addEntry(ANDROID_FLASH_MODE, value);

	value = ANDROID_FLASH_STATE_UNAVAILABLE;
	resultMetadata->addEntry(ANDROID_FLASH_STATE, value);

	if (settings.getEntry(ANDROID_LENS_APERTURE, &entry))
		resultMetadata->addEntry(ANDROID_LENS_APERTURE, entry.data.f, 1);

	float focal_length = 1.0;
	resultMetadata->addEntry(ANDROID_LENS_FOCAL_LENGTH, focal_length);

	value = ANDROID_LENS_STATE_STATIONARY;
	resultMetadata->addEntry(ANDROID_LENS_STATE, value);

	value = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
	resultMetadata->addEntry(ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
				 value);

	value32 = ANDROID_SENSOR_TEST_PATTERN_MODE_OFF;
	resultMetadata->addEntry(ANDROID_SENSOR_TEST_PATTERN_MODE, value32);

	value = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
	resultMetadata->addEntry(ANDROID_STATISTICS_FACE_DETECT_MODE, value);

	value = ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_OFF;
	resultMetadata->addEntry(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
				 value);

	value = ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE_OFF;
	resultMetadata->addEntry(ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE, value);

	value = ANDROID_STATISTICS_SCENE_FLICKER_NONE;
	resultMetadata->addEntry(ANDROID_STATISTICS_SCENE_FLICKER, value);

	value = ANDROID_NOISE_REDUCTION_MODE_OFF;
	resultMetadata->addEntry(ANDROID_NOISE_REDUCTION_MODE, value);

	/* 33.3 msec */
	const int64_t rolling_shutter_skew = 33300000;
	resultMetadata->addEntry(ANDROID_SENSOR_ROLLING_SHUTTER_SKEW,
				 rolling_shutter_skew);

	/* Add metadata tags reported by libcamera. */
	const int64_t timestamp = metadata.get(controls::SensorTimestamp).value_or(0);
	resultMetadata->addEntry(ANDROID_SENSOR_TIMESTAMP, timestamp);

	const auto &pipelineDepth = metadata.get(controls::draft::PipelineDepth);
	if (pipelineDepth)
		resultMetadata->addEntry(ANDROID_REQUEST_PIPELINE_DEPTH,
					 *pipelineDepth);

	const auto &exposureTime = metadata.get(controls::ExposureTime);
	if (exposureTime)
		resultMetadata->addEntry(ANDROID_SENSOR_EXPOSURE_TIME,
					 *exposureTime * 1000ULL);

	const auto &frameDuration = metadata.get(controls::FrameDuration);
	if (frameDuration)
		resultMetadata->addEntry(ANDROID_SENSOR_FRAME_DURATION,
					 *frameDuration * 1000);

	const auto &scalerCrop = metadata.get(controls::ScalerCrop);
	if (scalerCrop) {
		const Rectangle &crop = *scalerCrop;
		int32_t cropRect[] = {
			crop.x, crop.y, static_cast<int32_t>(crop.width),
			static_cast<int32_t>(crop.height),
		};
		resultMetadata->addEntry(ANDROID_SCALER_CROP_REGION, cropRect);
	}

	const auto &testPatternMode = metadata.get(controls::draft::TestPatternMode);
	if (testPatternMode)
		resultMetadata->addEntry(ANDROID_SENSOR_TEST_PATTERN_MODE,
					 *testPatternMode);

	/*
	 * Return the result metadata pack even is not valid: get() will return
	 * nullptr.
	 */
	if (!resultMetadata->isValid()) {
		LOG(HAL, Error) << "Failed to construct result metadata";
	}

	if (resultMetadata->resized()) {
		auto [entryCount, dataCount] = resultMetadata->usage();
		LOG(HAL, Info)
			<< "Result metadata resized: " << entryCount
			<< " entries and " << dataCount << " bytes used";
	}

	return resultMetadata;
}
