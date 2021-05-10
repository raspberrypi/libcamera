/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_device.cpp - libcamera Android Camera Device
 */

#include "camera_device.h"
#include "camera_hal_config.h"
#include "camera_ops.h"
#include "post_processor.h"

#include <array>
#include <cmath>
#include <fstream>
#include <sys/mman.h>
#include <tuple>
#include <unistd.h>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/thread.h"
#include "libcamera/internal/utils.h"

#include "system/graphics.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

namespace {

/*
 * \var camera3Resolutions
 * \brief The list of image resolutions defined as mandatory to be supported by
 * the Android Camera3 specification
 */
const std::vector<Size> camera3Resolutions = {
	{ 320, 240 },
	{ 640, 480 },
	{ 1280, 720 },
	{ 1920, 1080 }
};

/*
 * \struct Camera3Format
 * \brief Data associated with an Android format identifier
 * \var libcameraFormats List of libcamera pixel formats compatible with the
 * Android format
 * \var name The human-readable representation of the Android format code
 */
struct Camera3Format {
	std::vector<PixelFormat> libcameraFormats;
	bool mandatory;
	const char *name;
};

/*
 * \var camera3FormatsMap
 * \brief Associate Android format code with ancillary data
 */
const std::map<int, const Camera3Format> camera3FormatsMap = {
	{
		HAL_PIXEL_FORMAT_BLOB, {
			{ formats::MJPEG },
			true,
			"BLOB"
		}
	}, {
		HAL_PIXEL_FORMAT_YCbCr_420_888, {
			{ formats::NV12, formats::NV21 },
			true,
			"YCbCr_420_888"
		}
	}, {
		/*
		 * \todo Translate IMPLEMENTATION_DEFINED inspecting the gralloc
		 * usage flag. For now, copy the YCbCr_420 configuration.
		 */
		HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, {
			{ formats::NV12, formats::NV21 },
			true,
			"IMPLEMENTATION_DEFINED"
		}
	}, {
		HAL_PIXEL_FORMAT_RAW10, {
			{
				formats::SBGGR10_CSI2P,
				formats::SGBRG10_CSI2P,
				formats::SGRBG10_CSI2P,
				formats::SRGGB10_CSI2P
			},
			false,
			"RAW10"
		}
	}, {
		HAL_PIXEL_FORMAT_RAW12, {
			{
				formats::SBGGR12_CSI2P,
				formats::SGBRG12_CSI2P,
				formats::SGRBG12_CSI2P,
				formats::SRGGB12_CSI2P
			},
			false,
			"RAW12"
		}
	}, {
		HAL_PIXEL_FORMAT_RAW16, {
			{
				formats::SBGGR16,
				formats::SGBRG16,
				formats::SGRBG16,
				formats::SRGGB16
			},
			false,
			"RAW16"
		}
	}, {
		HAL_PIXEL_FORMAT_RAW_OPAQUE, {
			{
				formats::SBGGR10_IPU3,
				formats::SGBRG10_IPU3,
				formats::SGRBG10_IPU3,
				formats::SRGGB10_IPU3
			},
			false,
			"RAW_OPAQUE"
		}
	},
};

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

bool isValidRequest(camera3_capture_request_t *camera3Request)
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
	}

	return true;
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
 * \struct Camera3RequestDescriptor
 *
 * A utility structure that groups information about a capture request to be
 * later re-used at request complete time to notify the framework.
 */

CameraDevice::Camera3RequestDescriptor::Camera3RequestDescriptor(
	Camera *camera, const camera3_capture_request_t *camera3Request)
{
	frameNumber_ = camera3Request->frame_number;

	/* Copy the camera3 request stream information for later access. */
	const uint32_t numBuffers = camera3Request->num_output_buffers;
	buffers_.resize(numBuffers);
	for (uint32_t i = 0; i < numBuffers; i++)
		buffers_[i] = camera3Request->output_buffers[i];

	/*
	 * FrameBuffer instances created by wrapping a camera3 provided dmabuf
	 * are emplaced in this vector of unique_ptr<> for lifetime management.
	 */
	frameBuffers_.reserve(numBuffers);

	/* Clone the controls associated with the camera3 request. */
	settings_ = CameraMetadata(camera3Request->settings);

	/*
	 * Create the CaptureRequest, stored as a unique_ptr<> to tie its
	 * lifetime to the descriptor.
	 */
	request_ = std::make_unique<CaptureRequest>(camera);
}

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
 * This method is called before the camera device is opened.
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

	if (properties.contains(properties::Location)) {
		int32_t location = properties.get(properties::Location);
		switch (location) {
		case properties::CameraLocationFront:
			facing_ = CAMERA_FACING_FRONT;
			break;
		case properties::CameraLocationBack:
			facing_ = CAMERA_FACING_BACK;
			break;
		case properties::CameraLocationExternal:
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
	if (properties.contains(properties::Rotation)) {
		int rotation = properties.get(properties::Rotation);
		orientation_ = (360 - rotation) % 360;
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

	/* Acquire the camera and initialize available stream configurations. */
	int ret = camera_->acquire();
	if (ret) {
		LOG(HAL, Error) << "Failed to temporarily acquire the camera";
		return ret;
	}

	ret = initializeStreamConfigurations();
	camera_->release();
	return ret;
}

std::vector<Size> CameraDevice::getYUVResolutions(CameraConfiguration *cameraConfig,
						  const PixelFormat &pixelFormat,
						  const std::vector<Size> &resolutions)
{
	std::vector<Size> supportedResolutions;

	StreamConfiguration &cfg = cameraConfig->at(0);
	for (const Size &res : resolutions) {
		cfg.pixelFormat = pixelFormat;
		cfg.size = res;

		CameraConfiguration::Status status = cameraConfig->validate();
		if (status != CameraConfiguration::Valid) {
			LOG(HAL, Debug) << cfg.toString() << " not supported";
			continue;
		}

		LOG(HAL, Debug) << cfg.toString() << " supported";

		supportedResolutions.push_back(res);
	}

	return supportedResolutions;
}

std::vector<Size> CameraDevice::getRawResolutions(const libcamera::PixelFormat &pixelFormat)
{
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StreamRole::Raw });
	StreamConfiguration &cfg = cameraConfig->at(0);
	const StreamFormats &formats = cfg.formats();
	std::vector<Size> supportedResolutions = formats.sizes(pixelFormat);

	return supportedResolutions;
}

/*
 * Initialize the format conversion map to translate from Android format
 * identifier to libcamera pixel formats and fill in the list of supported
 * stream configurations to be reported to the Android camera framework through
 * the static stream configuration metadata.
 */
int CameraDevice::initializeStreamConfigurations()
{
	/*
	 * Get the maximum output resolutions
	 * \todo Get this from the camera properties once defined
	 */
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StillCapture });
	if (!cameraConfig) {
		LOG(HAL, Error) << "Failed to get maximum resolution";
		return -EINVAL;
	}
	StreamConfiguration &cfg = cameraConfig->at(0);

	/*
	 * \todo JPEG - Adjust the maximum available resolution by taking the
	 * JPEG encoder requirements into account (alignment and aspect ratio).
	 */
	const Size maxRes = cfg.size;
	LOG(HAL, Debug) << "Maximum supported resolution: " << maxRes.toString();

	/*
	 * Build the list of supported image resolutions.
	 *
	 * The resolutions listed in camera3Resolution are mandatory to be
	 * supported, up to the camera maximum resolution.
	 *
	 * Augment the list by adding resolutions calculated from the camera
	 * maximum one.
	 */
	std::vector<Size> cameraResolutions;
	std::copy_if(camera3Resolutions.begin(), camera3Resolutions.end(),
		     std::back_inserter(cameraResolutions),
		     [&](const Size &res) { return res < maxRes; });

	/*
	 * The Camera3 specification suggests adding 1/2 and 1/4 of the maximum
	 * resolution.
	 */
	for (unsigned int divider = 2;; divider <<= 1) {
		Size derivedSize{
			maxRes.width / divider,
			maxRes.height / divider,
		};

		if (derivedSize.width < 320 ||
		    derivedSize.height < 240)
			break;

		cameraResolutions.push_back(derivedSize);
	}
	cameraResolutions.push_back(maxRes);

	/* Remove duplicated entries from the list of supported resolutions. */
	std::sort(cameraResolutions.begin(), cameraResolutions.end());
	auto last = std::unique(cameraResolutions.begin(), cameraResolutions.end());
	cameraResolutions.erase(last, cameraResolutions.end());

	/*
	 * Build the list of supported camera formats.
	 *
	 * To each Android format a list of compatible libcamera formats is
	 * associated. The first libcamera format that tests successful is added
	 * to the format translation map used when configuring the streams.
	 * It is then tested against the list of supported camera resolutions to
	 * build the stream configuration map reported through the camera static
	 * metadata.
	 */
	Size maxJpegSize;
	for (const auto &format : camera3FormatsMap) {
		int androidFormat = format.first;
		const Camera3Format &camera3Format = format.second;
		const std::vector<PixelFormat> &libcameraFormats =
			camera3Format.libcameraFormats;

		LOG(HAL, Debug) << "Trying to map Android format "
				<< camera3Format.name;

		/*
		 * JPEG is always supported, either produced directly by the
		 * camera, or encoded in the HAL.
		 */
		if (androidFormat == HAL_PIXEL_FORMAT_BLOB) {
			formatsMap_[androidFormat] = formats::MJPEG;
			LOG(HAL, Debug) << "Mapped Android format "
					<< camera3Format.name << " to "
					<< formats::MJPEG.toString()
					<< " (fixed mapping)";
			continue;
		}

		/*
		 * Test the libcamera formats that can produce images
		 * compatible with the format defined by Android.
		 */
		PixelFormat mappedFormat;
		for (const PixelFormat &pixelFormat : libcameraFormats) {

			LOG(HAL, Debug) << "Testing " << pixelFormat.toString();

			/*
			 * The stream configuration size can be adjusted,
			 * not the pixel format.
			 *
			 * \todo This could be simplified once all pipeline
			 * handlers will report the StreamFormats list of
			 * supported formats.
			 */
			cfg.pixelFormat = pixelFormat;

			CameraConfiguration::Status status = cameraConfig->validate();
			if (status != CameraConfiguration::Invalid &&
			    cfg.pixelFormat == pixelFormat) {
				mappedFormat = pixelFormat;
				break;
			}
		}

		if (!mappedFormat.isValid()) {
			/* If the format is not mandatory, skip it. */
			if (!camera3Format.mandatory)
				continue;

			LOG(HAL, Error)
				<< "Failed to map mandatory Android format "
				<< camera3Format.name << " ("
				<< utils::hex(androidFormat) << "): aborting";
			return -EINVAL;
		}

		/*
		 * Record the mapping and then proceed to generate the
		 * stream configurations map, by testing the image resolutions.
		 */
		formatsMap_[androidFormat] = mappedFormat;
		LOG(HAL, Debug) << "Mapped Android format "
				<< camera3Format.name << " to "
				<< mappedFormat.toString();

		std::vector<Size> resolutions;
		const PixelFormatInfo &info = PixelFormatInfo::info(mappedFormat);
		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW)
			resolutions = getRawResolutions(mappedFormat);
		else
			resolutions = getYUVResolutions(cameraConfig.get(),
							mappedFormat,
							cameraResolutions);

		for (const Size &res : resolutions) {
			streamConfigurations_.push_back({ res, androidFormat });

			/*
			 * If the format is HAL_PIXEL_FORMAT_YCbCr_420_888
			 * from which JPEG is produced, add an entry for
			 * the JPEG stream.
			 *
			 * \todo Wire the JPEG encoder to query the supported
			 * sizes provided a list of formats it can encode.
			 *
			 * \todo Support JPEG streams produced by the Camera
			 * natively.
			 */
			if (androidFormat == HAL_PIXEL_FORMAT_YCbCr_420_888) {
				streamConfigurations_.push_back(
					{ res, HAL_PIXEL_FORMAT_BLOB });
				maxJpegSize = std::max(maxJpegSize, res);
			}
		}

		/*
		 * \todo Calculate the maximum JPEG buffer size by asking the
		 * encoder giving the maximum frame size required.
		 */
		maxJpegBufferSize_ = maxJpegSize.width * maxJpegSize.height * 1.5;
	}

	LOG(HAL, Debug) << "Collected stream configuration map: ";
	for (const auto &entry : streamConfigurations_)
		LOG(HAL, Debug) << "{ " << entry.resolution.toString() << " - "
				<< utils::hex(entry.androidFormat) << " }";

	return 0;
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
	streams_.clear();

	stop();

	camera_->release();
}

void CameraDevice::stop()
{
	if (state_ == State::Stopped)
		return;

	worker_.stop();
	camera_->stop();

	descriptors_.clear();
	state_ = State::Stopped;
}

void CameraDevice::setCallbacks(const camera3_callback_ops_t *callbacks)
{
	callbacks_ = callbacks;
}

/*
 * Return static information for the camera.
 */
const camera_metadata_t *CameraDevice::getStaticMetadata()
{
	if (staticMetadata_)
		return staticMetadata_->get();

	staticMetadata_ = std::make_unique<CameraMetadata>(64, 1024);
	if (!staticMetadata_->isValid()) {
		LOG(HAL, Error) << "Failed to allocate static metadata";
		staticMetadata_.reset();
		return nullptr;
	}

	const ControlInfoMap &controlsInfo = camera_->controls();
	const ControlList &properties = camera_->properties();

	/* Color correction static metadata. */
	{
		std::vector<uint8_t> data;
		data.reserve(3);
		const auto &infoMap = controlsInfo.find(&controls::draft::ColorCorrectionAberrationMode);
		if (infoMap != controlsInfo.end()) {
			for (const auto &value : infoMap->second.values())
				data.push_back(value.get<int32_t>());
		} else {
			data.push_back(ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF);
		}
		staticMetadata_->addEntry(ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
					  data);
	}

	/* Control static metadata. */
	std::vector<uint8_t> aeAvailableAntiBandingModes = {
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
				  aeAvailableAntiBandingModes);

	std::vector<uint8_t> aeAvailableModes = {
		ANDROID_CONTROL_AE_MODE_ON,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_AVAILABLE_MODES,
				  aeAvailableModes);

	int64_t minFrameDurationNsec = -1;
	int64_t maxFrameDurationNsec = -1;
	const auto frameDurationsInfo = controlsInfo.find(&controls::FrameDurationLimits);
	if (frameDurationsInfo != controlsInfo.end()) {
		minFrameDurationNsec = frameDurationsInfo->second.min().get<int64_t>() * 1000;
		maxFrameDurationNsec = frameDurationsInfo->second.max().get<int64_t>() * 1000;

		/*
		 * Adjust the minimum frame duration to comply with Android
		 * requirements. The camera service mandates all preview/record
		 * streams to have a minimum frame duration < 33,366 milliseconds
		 * (see MAX_PREVIEW_RECORD_DURATION_NS in the camera service
		 * implementation).
		 *
		 * If we're close enough (+ 500 useconds) to that value, round
		 * the minimum frame duration of the camera to an accepted
		 * value.
		 */
		static constexpr int64_t MAX_PREVIEW_RECORD_DURATION_NS = 1e9 / 29.97;
		if (minFrameDurationNsec > MAX_PREVIEW_RECORD_DURATION_NS &&
		    minFrameDurationNsec < MAX_PREVIEW_RECORD_DURATION_NS + 500000)
			minFrameDurationNsec = MAX_PREVIEW_RECORD_DURATION_NS - 1000;

		/*
		 * The AE routine frame rate limits are computed using the frame
		 * duration limits, as libcamera clips the AE routine to the
		 * frame durations.
		 */
		int32_t maxFps = std::round(1e9 / minFrameDurationNsec);
		int32_t minFps = std::round(1e9 / maxFrameDurationNsec);
		minFps = std::max(1, minFps);

		/*
		 * Force rounding errors so that we have the proper frame
		 * durations for when we reuse these variables later
		 */
		minFrameDurationNsec = 1e9 / maxFps;
		maxFrameDurationNsec = 1e9 / minFps;

		/*
		 * Register to the camera service {min, max} and {max, max}
		 * intervals as requested by the metadata documentation.
		 */
		int32_t availableAeFpsTarget[] = {
			minFps, maxFps, maxFps, maxFps
		};
		staticMetadata_->addEntry(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
					  availableAeFpsTarget);
	}

	std::vector<int32_t> aeCompensationRange = {
		0, 0,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_COMPENSATION_RANGE,
				  aeCompensationRange);

	const camera_metadata_rational_t aeCompensationStep[] = {
		{ 0, 1 }
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_COMPENSATION_STEP,
				  aeCompensationStep, 1);

	std::vector<uint8_t> availableAfModes = {
		ANDROID_CONTROL_AF_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AF_AVAILABLE_MODES,
				  availableAfModes);

	std::vector<uint8_t> availableEffects = {
		ANDROID_CONTROL_EFFECT_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_EFFECTS,
				  availableEffects);

	std::vector<uint8_t> availableSceneModes = {
		ANDROID_CONTROL_SCENE_MODE_DISABLED,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
				  availableSceneModes);

	std::vector<uint8_t> availableStabilizationModes = {
		ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
				  availableStabilizationModes);

	/*
	 * \todo Inspect the Camera capabilities to report the available
	 * AWB modes. Default to AUTO as CTS tests require it.
	 */
	std::vector<uint8_t> availableAwbModes = {
		ANDROID_CONTROL_AWB_MODE_AUTO,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AWB_AVAILABLE_MODES,
				  availableAwbModes);

	std::vector<int32_t> availableMaxRegions = {
		0, 0, 0,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_MAX_REGIONS,
				  availableMaxRegions);

	std::vector<uint8_t> sceneModesOverride = {
		ANDROID_CONTROL_AE_MODE_ON,
		ANDROID_CONTROL_AWB_MODE_AUTO,
		ANDROID_CONTROL_AF_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_SCENE_MODE_OVERRIDES,
				  sceneModesOverride);

	uint8_t aeLockAvailable = ANDROID_CONTROL_AE_LOCK_AVAILABLE_FALSE;
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_LOCK_AVAILABLE,
				  aeLockAvailable);

	uint8_t awbLockAvailable = ANDROID_CONTROL_AWB_LOCK_AVAILABLE_FALSE;
	staticMetadata_->addEntry(ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
				  awbLockAvailable);

	char availableControlModes = ANDROID_CONTROL_MODE_AUTO;
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_MODES,
				  availableControlModes);

	/* JPEG static metadata. */

	/*
	 * Create the list of supported thumbnail sizes by inspecting the
	 * available JPEG resolutions collected in streamConfigurations_ and
	 * generate one entry for each aspect ratio.
	 *
	 * The JPEG thumbnailer can freely scale, so pick an arbitrary
	 * (160, 160) size as the bounding rectangle, which is then cropped to
	 * the different supported aspect ratios.
	 */
	constexpr Size maxJpegThumbnail(160, 160);
	std::vector<Size> thumbnailSizes;
	thumbnailSizes.push_back({ 0, 0 });
	for (const auto &entry : streamConfigurations_) {
		if (entry.androidFormat != HAL_PIXEL_FORMAT_BLOB)
			continue;

		Size thumbnailSize = maxJpegThumbnail
				     .boundedToAspectRatio({ entry.resolution.width,
							     entry.resolution.height });
		thumbnailSizes.push_back(thumbnailSize);
	}

	std::sort(thumbnailSizes.begin(), thumbnailSizes.end());
	auto last = std::unique(thumbnailSizes.begin(), thumbnailSizes.end());
	thumbnailSizes.erase(last, thumbnailSizes.end());

	/* Transform sizes in to a list of integers that can be consumed. */
	std::vector<int32_t> thumbnailEntries;
	thumbnailEntries.reserve(thumbnailSizes.size() * 2);
	for (const auto &size : thumbnailSizes) {
		thumbnailEntries.push_back(size.width);
		thumbnailEntries.push_back(size.height);
	}
	staticMetadata_->addEntry(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
				  thumbnailEntries);

	staticMetadata_->addEntry(ANDROID_JPEG_MAX_SIZE, maxJpegBufferSize_);

	/* Sensor static metadata. */
	std::array<int32_t, 2> pixelArraySize;
	{
		const Size &size = properties.get(properties::PixelArraySize);
		pixelArraySize[0] = size.width;
		pixelArraySize[1] = size.height;
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
					  pixelArraySize);
	}

	if (properties.contains(properties::UnitCellSize)) {
		const Size &cellSize = properties.get<Size>(properties::UnitCellSize);
		std::array<float, 2> physicalSize{
			cellSize.width * pixelArraySize[0] / 1e6f,
			cellSize.height * pixelArraySize[1] / 1e6f
		};
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
					  physicalSize);
	}

	{
		const Span<const Rectangle> &rects =
			properties.get(properties::PixelArrayActiveAreas);
		std::vector<int32_t> data{
			static_cast<int32_t>(rects[0].x),
			static_cast<int32_t>(rects[0].y),
			static_cast<int32_t>(rects[0].width),
			static_cast<int32_t>(rects[0].height),
		};
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
					  data);
	}

	int32_t sensitivityRange[] = {
		32, 2400,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
				  sensitivityRange);

	/* Report the color filter arrangement if the camera reports it. */
	if (properties.contains(properties::draft::ColorFilterArrangement)) {
		uint8_t filterArr = properties.get(properties::draft::ColorFilterArrangement);
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
					  filterArr);
	}

	const auto &exposureInfo = controlsInfo.find(&controls::ExposureTime);
	if (exposureInfo != controlsInfo.end()) {
		int64_t exposureTimeRange[2] = {
			exposureInfo->second.min().get<int32_t>() * 1000LL,
			exposureInfo->second.max().get<int32_t>() * 1000LL,
		};
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
					  exposureTimeRange, 2);
	}

	staticMetadata_->addEntry(ANDROID_SENSOR_ORIENTATION, orientation_);

	std::vector<int32_t> testPatterModes = {
		ANDROID_SENSOR_TEST_PATTERN_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
				  testPatterModes);

	uint8_t timestampSource = ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
				  timestampSource);

	if (maxFrameDurationNsec > 0)
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_MAX_FRAME_DURATION,
					  maxFrameDurationNsec);

	/* Statistics static metadata. */
	uint8_t faceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
	staticMetadata_->addEntry(ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
				  &faceDetectMode, 1);

	int32_t maxFaceCount = 0;
	staticMetadata_->addEntry(ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
				  maxFaceCount);

	{
		std::vector<uint8_t> data;
		data.reserve(2);
		const auto &infoMap = controlsInfo.find(&controls::draft::LensShadingMapMode);
		if (infoMap != controlsInfo.end()) {
			for (const auto &value : infoMap->second.values())
				data.push_back(value.get<int32_t>());
		} else {
			data.push_back(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_OFF);
		}
		staticMetadata_->addEntry(ANDROID_STATISTICS_INFO_AVAILABLE_LENS_SHADING_MAP_MODES,
					  data);
	}

	/* Sync static metadata. */
	int32_t maxLatency = ANDROID_SYNC_MAX_LATENCY_UNKNOWN;
	staticMetadata_->addEntry(ANDROID_SYNC_MAX_LATENCY, maxLatency);

	/* Flash static metadata. */
	char flashAvailable = ANDROID_FLASH_INFO_AVAILABLE_FALSE;
	staticMetadata_->addEntry(ANDROID_FLASH_INFO_AVAILABLE,
				  flashAvailable);

	/* Lens static metadata. */
	std::vector<float> lensApertures = {
		2.53 / 100,
	};
	staticMetadata_->addEntry(ANDROID_LENS_INFO_AVAILABLE_APERTURES,
				  lensApertures);

	uint8_t lensFacing;
	switch (facing_) {
	default:
	case CAMERA_FACING_FRONT:
		lensFacing = ANDROID_LENS_FACING_FRONT;
		break;
	case CAMERA_FACING_BACK:
		lensFacing = ANDROID_LENS_FACING_BACK;
		break;
	case CAMERA_FACING_EXTERNAL:
		lensFacing = ANDROID_LENS_FACING_EXTERNAL;
		break;
	}
	staticMetadata_->addEntry(ANDROID_LENS_FACING, lensFacing);

	std::vector<float> lensFocalLengths = {
		1,
	};
	staticMetadata_->addEntry(ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
				  lensFocalLengths);

	std::vector<uint8_t> opticalStabilizations = {
		ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
				  opticalStabilizations);

	float hypeFocalDistance = 0;
	staticMetadata_->addEntry(ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
				  hypeFocalDistance);

	float minFocusDistance = 0;
	staticMetadata_->addEntry(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
				  minFocusDistance);

	/* Noise reduction modes. */
	{
		std::vector<uint8_t> data;
		data.reserve(5);
		const auto &infoMap = controlsInfo.find(&controls::draft::NoiseReductionMode);
		if (infoMap != controlsInfo.end()) {
			for (const auto &value : infoMap->second.values())
				data.push_back(value.get<int32_t>());
		} else {
			data.push_back(ANDROID_NOISE_REDUCTION_MODE_OFF);
		}
		staticMetadata_->addEntry(ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
					  data);
	}

	/* Scaler static metadata. */

	/*
	 * \todo The digital zoom factor is a property that depends on the
	 * desired output configuration and the sensor frame size input to the
	 * ISP. This information is not available to the Android HAL, not at
	 * initialization time at least.
	 *
	 * As a workaround rely on pipeline handlers initializing the
	 * ScalerCrop control with the camera default configuration and use the
	 * maximum and minimum crop rectangles to calculate the digital zoom
	 * factor.
	 */
	float maxZoom = 1.0f;
	const auto scalerCrop = controlsInfo.find(&controls::ScalerCrop);
	if (scalerCrop != controlsInfo.end()) {
		Rectangle min = scalerCrop->second.min().get<Rectangle>();
		Rectangle max = scalerCrop->second.max().get<Rectangle>();
		maxZoom = std::min(1.0f * max.width / min.width,
				   1.0f * max.height / min.height);
	}
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
				  maxZoom);

	std::vector<uint32_t> availableStreamConfigurations;
	availableStreamConfigurations.reserve(streamConfigurations_.size() * 4);
	for (const auto &entry : streamConfigurations_) {
		availableStreamConfigurations.push_back(entry.androidFormat);
		availableStreamConfigurations.push_back(entry.resolution.width);
		availableStreamConfigurations.push_back(entry.resolution.height);
		availableStreamConfigurations.push_back(
			ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT);
	}
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
				  availableStreamConfigurations.data(),
				  availableStreamConfigurations.size());

	std::vector<int64_t> availableStallDurations = {
		ANDROID_SCALER_AVAILABLE_FORMATS_BLOB, 2560, 1920, 33333333,
	};
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
				  availableStallDurations);

	/* Use the minimum frame duration for all the YUV/RGB formats. */
	if (minFrameDurationNsec > 0) {
		std::vector<int64_t> minFrameDurations;
		minFrameDurations.reserve(streamConfigurations_.size() * 4);
		for (const auto &entry : streamConfigurations_) {
			minFrameDurations.push_back(entry.androidFormat);
			minFrameDurations.push_back(entry.resolution.width);
			minFrameDurations.push_back(entry.resolution.height);
			minFrameDurations.push_back(minFrameDurationNsec);
		}
		staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
					  minFrameDurations);
	}

	uint8_t croppingType = ANDROID_SCALER_CROPPING_TYPE_CENTER_ONLY;
	staticMetadata_->addEntry(ANDROID_SCALER_CROPPING_TYPE, croppingType);

	/* Info static metadata. */
	uint8_t supportedHWLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
	staticMetadata_->addEntry(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
				  supportedHWLevel);

	/* Request static metadata. */
	int32_t partialResultCount = 1;
	staticMetadata_->addEntry(ANDROID_REQUEST_PARTIAL_RESULT_COUNT,
				  partialResultCount);

	{
		/* Default the value to 2 if not reported by the camera. */
		uint8_t maxPipelineDepth = 2;
		const auto &infoMap = controlsInfo.find(&controls::draft::PipelineDepth);
		if (infoMap != controlsInfo.end())
			maxPipelineDepth = infoMap->second.max().get<int32_t>();
		staticMetadata_->addEntry(ANDROID_REQUEST_PIPELINE_MAX_DEPTH,
					  maxPipelineDepth);
	}

	/* LIMITED does not support reprocessing. */
	uint32_t maxNumInputStreams = 0;
	staticMetadata_->addEntry(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
				  maxNumInputStreams);

	std::vector<uint8_t> availableCapabilities = {
		ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE,
	};

	/* Report if camera supports RAW. */
	bool rawStreamAvailable = false;
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StreamRole::Raw });
	if (cameraConfig && !cameraConfig->empty()) {
		const PixelFormatInfo &info =
			PixelFormatInfo::info(cameraConfig->at(0).pixelFormat);
		/* Only advertise RAW support if RAW16 is possible. */
		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW &&
		    info.bitsPerPixel == 16) {
			rawStreamAvailable = true;
			availableCapabilities.push_back(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_RAW);
		}
	}

	/* Number of { RAW, YUV, JPEG } supported output streams */
	int32_t numOutStreams[] = { rawStreamAvailable, 2, 1 };
	staticMetadata_->addEntry(ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS,
				  numOutStreams);

	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
				  availableCapabilities);

	std::vector<int32_t> availableCharacteristicsKeys = {
		ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
		ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
		ANDROID_CONTROL_AE_AVAILABLE_MODES,
		ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
		ANDROID_CONTROL_AE_COMPENSATION_RANGE,
		ANDROID_CONTROL_AE_COMPENSATION_STEP,
		ANDROID_CONTROL_AE_LOCK_AVAILABLE,
		ANDROID_CONTROL_AF_AVAILABLE_MODES,
		ANDROID_CONTROL_AVAILABLE_EFFECTS,
		ANDROID_CONTROL_AVAILABLE_MODES,
		ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
		ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
		ANDROID_CONTROL_AWB_AVAILABLE_MODES,
		ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
		ANDROID_CONTROL_MAX_REGIONS,
		ANDROID_CONTROL_SCENE_MODE_OVERRIDES,
		ANDROID_FLASH_INFO_AVAILABLE,
		ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
		ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
		ANDROID_JPEG_MAX_SIZE,
		ANDROID_LENS_FACING,
		ANDROID_LENS_INFO_AVAILABLE_APERTURES,
		ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
		ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
		ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
		ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
		ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
		ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
		ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
		ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS,
		ANDROID_REQUEST_PARTIAL_RESULT_COUNT,
		ANDROID_REQUEST_PIPELINE_MAX_DEPTH,
		ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
		ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
		ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
		ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
		ANDROID_SCALER_CROPPING_TYPE,
		ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
		ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
		ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
		ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
		ANDROID_SENSOR_INFO_MAX_FRAME_DURATION,
		ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
		ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
		ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
		ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
		ANDROID_SENSOR_ORIENTATION,
		ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
		ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
		ANDROID_SYNC_MAX_LATENCY,
	};
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
				  availableCharacteristicsKeys);

	std::vector<int32_t> availableRequestKeys = {
		ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE,
		ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
		ANDROID_CONTROL_AE_LOCK,
		ANDROID_CONTROL_AE_MODE,
		ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER,
		ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
		ANDROID_CONTROL_AF_MODE,
		ANDROID_CONTROL_AF_TRIGGER,
		ANDROID_CONTROL_AWB_LOCK,
		ANDROID_CONTROL_AWB_MODE,
		ANDROID_CONTROL_CAPTURE_INTENT,
		ANDROID_CONTROL_EFFECT_MODE,
		ANDROID_CONTROL_MODE,
		ANDROID_CONTROL_SCENE_MODE,
		ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
		ANDROID_FLASH_MODE,
		ANDROID_JPEG_ORIENTATION,
		ANDROID_JPEG_QUALITY,
		ANDROID_JPEG_THUMBNAIL_QUALITY,
		ANDROID_JPEG_THUMBNAIL_SIZE,
		ANDROID_LENS_APERTURE,
		ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
		ANDROID_NOISE_REDUCTION_MODE,
		ANDROID_SCALER_CROP_REGION,
		ANDROID_STATISTICS_FACE_DETECT_MODE
	};
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
				  availableRequestKeys);

	std::vector<int32_t> availableResultKeys = {
		ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE,
		ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
		ANDROID_CONTROL_AE_LOCK,
		ANDROID_CONTROL_AE_MODE,
		ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER,
		ANDROID_CONTROL_AE_STATE,
		ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
		ANDROID_CONTROL_AF_MODE,
		ANDROID_CONTROL_AF_STATE,
		ANDROID_CONTROL_AF_TRIGGER,
		ANDROID_CONTROL_AWB_LOCK,
		ANDROID_CONTROL_AWB_MODE,
		ANDROID_CONTROL_AWB_STATE,
		ANDROID_CONTROL_CAPTURE_INTENT,
		ANDROID_CONTROL_EFFECT_MODE,
		ANDROID_CONTROL_MODE,
		ANDROID_CONTROL_SCENE_MODE,
		ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
		ANDROID_FLASH_MODE,
		ANDROID_FLASH_STATE,
		ANDROID_JPEG_GPS_COORDINATES,
		ANDROID_JPEG_GPS_PROCESSING_METHOD,
		ANDROID_JPEG_GPS_TIMESTAMP,
		ANDROID_JPEG_ORIENTATION,
		ANDROID_JPEG_QUALITY,
		ANDROID_JPEG_SIZE,
		ANDROID_JPEG_THUMBNAIL_QUALITY,
		ANDROID_JPEG_THUMBNAIL_SIZE,
		ANDROID_LENS_APERTURE,
		ANDROID_LENS_FOCAL_LENGTH,
		ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
		ANDROID_LENS_STATE,
		ANDROID_NOISE_REDUCTION_MODE,
		ANDROID_REQUEST_PIPELINE_DEPTH,
		ANDROID_SCALER_CROP_REGION,
		ANDROID_SENSOR_EXPOSURE_TIME,
		ANDROID_SENSOR_FRAME_DURATION,
		ANDROID_SENSOR_ROLLING_SHUTTER_SKEW,
		ANDROID_SENSOR_TEST_PATTERN_MODE,
		ANDROID_SENSOR_TIMESTAMP,
		ANDROID_STATISTICS_FACE_DETECT_MODE,
		ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
		ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE,
		ANDROID_STATISTICS_SCENE_FLICKER,
	};
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
				  availableResultKeys);

	if (!staticMetadata_->isValid()) {
		LOG(HAL, Error) << "Failed to construct static metadata";
		staticMetadata_.reset();
		return nullptr;
	}

	if (staticMetadata_->resized()) {
		auto [entryCount, dataCount] = staticMetadata_->usage();
		LOG(HAL, Info)
			<< "Static metadata resized: " << entryCount
			<< " entries and " << dataCount << " bytes used";
	}

	return staticMetadata_->get();
}

std::unique_ptr<CameraMetadata> CameraDevice::requestTemplatePreview()
{
	/*
	 * \todo Keep this in sync with the actual number of entries.
	 * Currently: 20 entries, 35 bytes
	 */
	auto requestTemplate = std::make_unique<CameraMetadata>(21, 36);
	if (!requestTemplate->isValid()) {
		return nullptr;
	}

	/* Get the FPS range registered in the static metadata. */
	camera_metadata_ro_entry_t entry;
	bool found = staticMetadata_->getEntry(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
					       &entry);
	if (!found) {
		LOG(HAL, Error) << "Cannot create capture template without FPS range";
		return nullptr;
	}

	/*
	 * Assume the AE_AVAILABLE_TARGET_FPS_RANGE static metadata
	 * has been assembled as {{min, max} {max, max}}.
	 */
	requestTemplate->addEntry(ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
				  entry.data.i32, 2);

	uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_MODE, aeMode);

	int32_t aeExposureCompensation = 0;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
				  aeExposureCompensation);

	uint8_t aePrecaptureTrigger = ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER,
				  aePrecaptureTrigger);

	uint8_t aeLock = ANDROID_CONTROL_AE_LOCK_OFF;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_LOCK, aeLock);

	uint8_t aeAntibandingMode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_ANTIBANDING_MODE,
				  aeAntibandingMode);

	uint8_t afMode = ANDROID_CONTROL_AF_MODE_OFF;
	requestTemplate->addEntry(ANDROID_CONTROL_AF_MODE, afMode);

	uint8_t afTrigger = ANDROID_CONTROL_AF_TRIGGER_IDLE;
	requestTemplate->addEntry(ANDROID_CONTROL_AF_TRIGGER, afTrigger);

	uint8_t awbMode = ANDROID_CONTROL_AWB_MODE_AUTO;
	requestTemplate->addEntry(ANDROID_CONTROL_AWB_MODE, awbMode);

	uint8_t awbLock = ANDROID_CONTROL_AWB_LOCK_OFF;
	requestTemplate->addEntry(ANDROID_CONTROL_AWB_LOCK, awbLock);

	uint8_t flashMode = ANDROID_FLASH_MODE_OFF;
	requestTemplate->addEntry(ANDROID_FLASH_MODE, flashMode);

	uint8_t faceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
	requestTemplate->addEntry(ANDROID_STATISTICS_FACE_DETECT_MODE,
				  faceDetectMode);

	uint8_t noiseReduction = ANDROID_NOISE_REDUCTION_MODE_OFF;
	requestTemplate->addEntry(ANDROID_NOISE_REDUCTION_MODE,
				  noiseReduction);

	uint8_t aberrationMode = ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF;
	requestTemplate->addEntry(ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
				  aberrationMode);

	uint8_t controlMode = ANDROID_CONTROL_MODE_AUTO;
	requestTemplate->addEntry(ANDROID_CONTROL_MODE, controlMode);

	float lensAperture = 2.53 / 100;
	requestTemplate->addEntry(ANDROID_LENS_APERTURE, lensAperture);

	uint8_t opticalStabilization = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
	requestTemplate->addEntry(ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
				  opticalStabilization);

	uint8_t captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
	requestTemplate->addEntry(ANDROID_CONTROL_CAPTURE_INTENT,
				  captureIntent);

	return requestTemplate;
}

std::unique_ptr<CameraMetadata> CameraDevice::requestTemplateVideo()
{
	std::unique_ptr<CameraMetadata> previewTemplate = requestTemplatePreview();
	if (!previewTemplate)
		return nullptr;

	/*
	 * The video template requires a fixed FPS range. Everything else
	 * stays the same as the preview template.
	 */
	camera_metadata_ro_entry_t entry;
	staticMetadata_->getEntry(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
				  &entry);

	/*
	 * Assume the AE_AVAILABLE_TARGET_FPS_RANGE static metadata
	 * has been assembled as {{min, max} {max, max}}.
	 */
	previewTemplate->updateEntry(ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
				     entry.data.i32 + 2, 2);

	return previewTemplate;
}

/*
 * Produce a metadata pack to be used as template for a capture request.
 */
const camera_metadata_t *CameraDevice::constructDefaultRequestSettings(int type)
{
	auto it = requestTemplates_.find(type);
	if (it != requestTemplates_.end())
		return it->second->get();

	/* Use the capture intent matching the requested template type. */
	std::unique_ptr<CameraMetadata> requestTemplate;
	uint8_t captureIntent;
	switch (type) {
	case CAMERA3_TEMPLATE_PREVIEW:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
		requestTemplate = requestTemplatePreview();
		break;
	case CAMERA3_TEMPLATE_STILL_CAPTURE:
		/*
		 * Use the preview template for still capture, they only differ
		 * for the torch mode we currently do not support.
		 */
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE;
		requestTemplate = requestTemplatePreview();
		break;
	case CAMERA3_TEMPLATE_VIDEO_RECORD:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD;
		requestTemplate = requestTemplateVideo();
		break;
	case CAMERA3_TEMPLATE_VIDEO_SNAPSHOT:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT;
		requestTemplate = requestTemplateVideo();
		break;
	/* \todo Implement templates generation for the remaining use cases. */
	case CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG:
	case CAMERA3_TEMPLATE_MANUAL:
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
	return requestTemplates_[type]->get();
}

PixelFormat CameraDevice::toPixelFormat(int format) const
{
	/* Translate Android format code to libcamera pixel format. */
	auto it = formatsMap_.find(format);
	if (it == formatsMap_.end()) {
		LOG(HAL, Error) << "Requested format " << utils::hex(format)
				<< " not supported";
		return PixelFormat();
	}

	return it->second;
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
	config_ = camera_->generateConfiguration();
	if (!config_) {
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

		PixelFormat format = toPixelFormat(stream->format);

		LOG(HAL, Info) << "Stream #" << i
			       << ", direction: " << stream->stream_type
			       << ", width: " << stream->width
			       << ", height: " << stream->height
			       << ", format: " << utils::hex(stream->format)
			       << ", rotation: " << rotationToString(stream->rotation)
#if defined(OS_CHROMEOS)
			       << ", crop_rotate_scale_degrees: "
			       << rotationToString(stream->crop_rotate_scale_degrees)
#endif
			       << " (" << format.toString() << ")";

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

		Camera3StreamConfig streamConfig;
		streamConfig.streams = { { stream, CameraStream::Type::Direct } };
		streamConfig.config.size = size;
		streamConfig.config.pixelFormat = format;
		streamConfigs.push_back(std::move(streamConfig));

		/* This stream will be produced by hardware. */
		stream->usage |= GRALLOC_USAGE_HW_CAMERA_WRITE;
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
		config_->addConfiguration(streamConfig.config);

		for (auto &stream : streamConfig.streams) {
			streams_.emplace_back(this, stream.type, stream.stream,
					      config_->size() - 1);
			stream.stream->priv = static_cast<void *>(&streams_.back());
		}
	}

	switch (config_->validate()) {
	case CameraConfiguration::Valid:
		break;
	case CameraConfiguration::Adjusted:
		LOG(HAL, Info) << "Camera configuration adjusted";

		for (const StreamConfiguration &cfg : *config_)
			LOG(HAL, Info) << " - " << cfg.toString();

		config_.reset();
		return -EINVAL;
	case CameraConfiguration::Invalid:
		LOG(HAL, Info) << "Camera configuration invalid";
		config_.reset();
		return -EINVAL;
	}

	/*
	 * Once the CameraConfiguration has been adjusted/validated
	 * it can be applied to the camera.
	 */
	int ret = camera_->configure(config_.get());
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

	return 0;
}

FrameBuffer *CameraDevice::createFrameBuffer(const buffer_handle_t camera3buffer)
{
	std::vector<FrameBuffer::Plane> planes;
	for (int i = 0; i < camera3buffer->numFds; i++) {
		/* Skip unused planes. */
		if (camera3buffer->data[i] == -1)
			break;

		FrameBuffer::Plane plane;
		plane.fd = FileDescriptor(camera3buffer->data[i]);
		if (!plane.fd.isValid()) {
			LOG(HAL, Error) << "Failed to obtain FileDescriptor ("
					<< camera3buffer->data[i] << ") "
					<< " on plane " << i;
			return nullptr;
		}

		off_t length = lseek(plane.fd.fd(), 0, SEEK_END);
		if (length == -1) {
			LOG(HAL, Error) << "Failed to query plane length";
			return nullptr;
		}

		plane.length = length;
		planes.push_back(std::move(plane));
	}

	return new FrameBuffer(std::move(planes));
}

int CameraDevice::processControls(Camera3RequestDescriptor *descriptor)
{
	const CameraMetadata &settings = descriptor->settings_;
	if (!settings.isValid())
		return 0;

	/* Translate the Android request settings to libcamera controls. */
	camera_metadata_ro_entry_t entry;
	if (settings.getEntry(ANDROID_SCALER_CROP_REGION, &entry)) {
		const int32_t *data = entry.data.i32;
		Rectangle cropRegion{ data[0], data[1],
				      static_cast<unsigned int>(data[2]),
				      static_cast<unsigned int>(data[3]) };
		ControlList &controls = descriptor->request_->controls();
		controls.set(controls::ScalerCrop, cropRegion);
	}

	return 0;
}

int CameraDevice::processCaptureRequest(camera3_capture_request_t *camera3Request)
{
	if (!isValidRequest(camera3Request))
		return -EINVAL;

	/* Start the camera if that's the first request we handle. */
	if (state_ == State::Stopped) {
		worker_.start();

		int ret = camera_->start();
		if (ret) {
			LOG(HAL, Error) << "Failed to start camera";
			return ret;
		}

		state_ = State::Running;
	}

	/*
	 * Save the request descriptors for use at completion time.
	 * The descriptor and the associated memory reserved here are freed
	 * at request complete time.
	 */
	Camera3RequestDescriptor descriptor(camera_.get(), camera3Request);

	/*
	 * \todo The Android request model is incremental, settings passed in
	 * previous requests are to be effective until overridden explicitly in
	 * a new request. Do we need to cache settings incrementally here, or is
	 * it handled by the Android camera service ?
	 */
	if (camera3Request->settings)
		lastSettings_ = camera3Request->settings;
	else
		descriptor.settings_ = lastSettings_;

	LOG(HAL, Debug) << "Queueing request " << descriptor.request_->cookie()
			<< " with " << descriptor.buffers_.size() << " streams";
	for (unsigned int i = 0; i < descriptor.buffers_.size(); ++i) {
		const camera3_stream_buffer_t &camera3Buffer = descriptor.buffers_[i];
		camera3_stream *camera3Stream = camera3Buffer.stream;
		CameraStream *cameraStream = static_cast<CameraStream *>(camera3Stream->priv);

		std::stringstream ss;
		ss << i << " - (" << camera3Stream->width << "x"
		   << camera3Stream->height << ")"
		   << "[" << utils::hex(camera3Stream->format) << "] -> "
		   << "(" << cameraStream->configuration().size.toString() << ")["
		   << cameraStream->configuration().pixelFormat.toString() << "]";

		/*
		 * Inspect the camera stream type, create buffers opportunely
		 * and add them to the Request if required.
		 */
		FrameBuffer *buffer = nullptr;
		switch (cameraStream->type()) {
		case CameraStream::Type::Mapped:
			/*
			 * Mapped streams don't need buffers added to the
			 * Request.
			 */
			LOG(HAL, Debug) << ss.str() << " (mapped)";
			continue;

		case CameraStream::Type::Direct:
			/*
			 * Create a libcamera buffer using the dmabuf
			 * descriptors of the camera3Buffer for each stream and
			 * associate it with the Camera3RequestDescriptor for
			 * lifetime management only.
			 */
			buffer = createFrameBuffer(*camera3Buffer.buffer);
			descriptor.frameBuffers_.emplace_back(buffer);
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
			buffer = cameraStream->getBuffer();
			LOG(HAL, Debug) << ss.str() << " (internal)";
			break;
		}

		if (!buffer) {
			LOG(HAL, Error) << "Failed to create buffer";
			return -ENOMEM;
		}

		descriptor.request_->addBuffer(cameraStream->stream(), buffer,
						camera3Buffer.acquire_fence);
	}

	/*
	 * Translate controls from Android to libcamera and queue the request
	 * to the CameraWorker thread.
	 */
	int ret = processControls(&descriptor);
	if (ret)
		return ret;

	worker_.queueRequest(descriptor.request_.get());

	{
		MutexLocker lock(mutex_);
		descriptors_[descriptor.request_->cookie()] = std::move(descriptor);
	}

	return 0;
}

void CameraDevice::requestComplete(Request *request)
{
	decltype(descriptors_)::node_type node;
	{
		MutexLocker lock(mutex_);
		auto it = descriptors_.find(request->cookie());
		if (it == descriptors_.end()) {
			/*
			 * \todo Clarify if the Camera has to be closed on
			 * ERROR_DEVICE and possibly demote the Fatal to simple
			 * Error.
			 */
			notifyError(0, nullptr, CAMERA3_MSG_ERROR_DEVICE);
			LOG(HAL, Fatal)
				<< "Unknown request: " << request->cookie();

			return;
		}

		node = descriptors_.extract(it);
	}
	Camera3RequestDescriptor &descriptor = node.mapped();

	/*
	 * Prepare the capture result for the Android camera stack.
	 *
	 * The buffer status is set to OK and later changed to ERROR if
	 * post-processing/compression fails.
	 */
	camera3_capture_result_t captureResult = {};
	captureResult.frame_number = descriptor.frameNumber_;
	captureResult.num_output_buffers = descriptor.buffers_.size();
	for (camera3_stream_buffer_t &buffer : descriptor.buffers_) {
		buffer.acquire_fence = -1;
		buffer.release_fence = -1;
		buffer.status = CAMERA3_BUFFER_STATUS_OK;
	}
	captureResult.output_buffers = descriptor.buffers_.data();
	captureResult.partial_result = 1;

	/*
	 * If the Request has failed, abort the request by notifying the error
	 * and complete the request with all buffers in error state.
	 */
	if (request->status() != Request::RequestComplete) {
		LOG(HAL, Error) << "Request " << request->cookie()
				<< " not successfully completed: "
				<< request->status();

		notifyError(descriptor.frameNumber_, nullptr,
			    CAMERA3_MSG_ERROR_REQUEST);

		captureResult.partial_result = 0;
		for (camera3_stream_buffer_t &buffer : descriptor.buffers_)
			buffer.status = CAMERA3_BUFFER_STATUS_ERROR;
		callbacks_->process_capture_result(callbacks_, &captureResult);

		return;
	}

	/*
	 * Notify shutter as soon as we have verified we have a valid request.
	 *
	 * \todo The shutter event notification should be sent to the framework
	 * as soon as possible, earlier than request completion time.
	 */
	uint64_t sensorTimestamp = static_cast<uint64_t>(request->metadata()
							 .get(controls::SensorTimestamp));
	notifyShutter(descriptor.frameNumber_, sensorTimestamp);

	LOG(HAL, Debug) << "Request " << request->cookie() << " completed with "
			<< descriptor.buffers_.size() << " streams";

	/*
	 * Generate the metadata associated with the captured buffers.
	 *
	 * Notify if the metadata generation has failed, but continue processing
	 * buffers and return an empty metadata pack.
	 */
	std::unique_ptr<CameraMetadata> resultMetadata = getResultMetadata(descriptor);
	if (!resultMetadata) {
		notifyError(descriptor.frameNumber_, nullptr, CAMERA3_MSG_ERROR_RESULT);

		/* The camera framework expects an empy metadata pack on error. */
		resultMetadata = std::make_unique<CameraMetadata>(0, 0);
	}

	/* Handle any JPEG compression. */
	for (camera3_stream_buffer_t &buffer : descriptor.buffers_) {
		CameraStream *cameraStream =
			static_cast<CameraStream *>(buffer.stream->priv);

		if (cameraStream->camera3Stream().format != HAL_PIXEL_FORMAT_BLOB)
			continue;

		FrameBuffer *src = request->findBuffer(cameraStream->stream());
		if (!src) {
			LOG(HAL, Error) << "Failed to find a source stream buffer";
			buffer.status = CAMERA3_BUFFER_STATUS_ERROR;
			notifyError(descriptor.frameNumber_, buffer.stream,
				    CAMERA3_MSG_ERROR_BUFFER);
			continue;
		}

		int ret = cameraStream->process(*src, *buffer.buffer,
						descriptor.settings_,
						resultMetadata.get());
		/*
		 * Return the FrameBuffer to the CameraStream now that we're
		 * done processing it.
		 */
		if (cameraStream->type() == CameraStream::Type::Internal)
			cameraStream->putBuffer(src);

		if (ret) {
			buffer.status = CAMERA3_BUFFER_STATUS_ERROR;
			notifyError(descriptor.frameNumber_, buffer.stream,
				    CAMERA3_MSG_ERROR_BUFFER);
		}
	}

	captureResult.result = resultMetadata->get();
	callbacks_->process_capture_result(callbacks_, &captureResult);
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
			       camera3_error_msg_code code)
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
		std::make_unique<CameraMetadata>(44, 166);
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
	const int64_t timestamp = metadata.get(controls::SensorTimestamp);
	resultMetadata->addEntry(ANDROID_SENSOR_TIMESTAMP, timestamp);

	if (metadata.contains(controls::draft::PipelineDepth)) {
		uint8_t pipeline_depth =
			metadata.get<int32_t>(controls::draft::PipelineDepth);
		resultMetadata->addEntry(ANDROID_REQUEST_PIPELINE_DEPTH,
					 pipeline_depth);
	}

	if (metadata.contains(controls::ExposureTime)) {
		int64_t exposure = metadata.get(controls::ExposureTime) * 1000ULL;
		resultMetadata->addEntry(ANDROID_SENSOR_EXPOSURE_TIME, exposure);
	}

	if (metadata.contains(controls::FrameDuration)) {
		int64_t duration = metadata.get(controls::FrameDuration) * 1000;
		resultMetadata->addEntry(ANDROID_SENSOR_FRAME_DURATION,
					 duration);
	}

	if (metadata.contains(controls::ScalerCrop)) {
		Rectangle crop = metadata.get(controls::ScalerCrop);
		int32_t cropRect[] = {
			crop.x, crop.y, static_cast<int32_t>(crop.width),
			static_cast<int32_t>(crop.height),
		};
		resultMetadata->addEntry(ANDROID_SCALER_CROP_REGION, cropRect);
	}

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
