/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_device.cpp - libcamera Android Camera Device
 */

#include "camera_device.h"
#include "camera_ops.h"

#include <sys/mman.h>
#include <tuple>
#include <vector>

#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/utils.h"

#include "camera_metadata.h"
#include "system/graphics.h"

#include "jpeg/encoder_libjpeg.h"

using namespace libcamera;

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

} /* namespace */

LOG_DECLARE_CATEGORY(HAL);

class MappedCamera3Buffer : public MappedBuffer
{
public:
	MappedCamera3Buffer(const buffer_handle_t camera3buffer, int flags);
};

MappedCamera3Buffer::MappedCamera3Buffer(const buffer_handle_t camera3buffer,
					 int flags)
{
	maps_.reserve(camera3buffer->numFds);
	error_ = 0;

	for (int i = 0; i < camera3buffer->numFds; i++) {
		if (camera3buffer->data[i] == -1)
			continue;

		off_t length = lseek(camera3buffer->data[i], 0, SEEK_END);
		if (length < 0) {
			error_ = -errno;
			LOG(HAL, Error) << "Failed to query plane length";
			break;
		}

		void *address = mmap(nullptr, length, flags, MAP_SHARED,
				     camera3buffer->data[i], 0);
		if (address == MAP_FAILED) {
			error_ = -errno;
			LOG(HAL, Error) << "Failed to mmap plane";
			break;
		}

		maps_.emplace_back(static_cast<uint8_t *>(address),
				   static_cast<size_t>(length));
	}
}

CameraStream::CameraStream(PixelFormat f, Size s)
	: index(-1), format(f), size(s), jpeg(nullptr)
{
}

CameraStream::~CameraStream()
{
	delete jpeg;
};

/*
 * \struct Camera3RequestDescriptor
 *
 * A utility structure that groups information about a capture request to be
 * later re-used at request complete time to notify the framework.
 */

CameraDevice::Camera3RequestDescriptor::Camera3RequestDescriptor(
		unsigned int frameNumber, unsigned int numBuffers)
	: frameNumber(frameNumber), numBuffers(numBuffers)
{
	buffers = new camera3_stream_buffer_t[numBuffers];
	frameBuffers.reserve(numBuffers);
}

CameraDevice::Camera3RequestDescriptor::~Camera3RequestDescriptor()
{
	delete[] buffers;
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

CameraDevice::CameraDevice(unsigned int id, const std::shared_ptr<Camera> &camera)
	: id_(id), running_(false), camera_(camera), staticMetadata_(nullptr),
	  facing_(CAMERA_FACING_FRONT), orientation_(0)
{
	camera_->requestCompleted.connect(this, &CameraDevice::requestComplete);

	/*
	 * \todo Determine a more accurate value for this during
	 *  streamConfiguration.
	 */
	maxJpegBufferSize_ = 13 << 20; /* 13631488 from USB HAL */
}

CameraDevice::~CameraDevice()
{
	if (staticMetadata_)
		delete staticMetadata_;

	for (auto &it : requestTemplates_)
		delete it.second;
}

/*
 * Initialize the camera static information.
 * This method is called before the camera device is opened.
 */
int CameraDevice::initialize()
{
	/* Initialize orientation and facing side of the camera. */
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
	}

	/*
	 * The Android orientation metadata and libcamera rotation property are
	 * defined differently but have identical numerical values for Android
	 * devices such as phones and tablets.
	 */
	if (properties.contains(properties::Rotation))
		orientation_ = properties.get(properties::Rotation);

	int ret = camera_->acquire();
	if (ret) {
		LOG(HAL, Error) << "Failed to temporarily acquire the camera";
		return ret;
	}

	ret = initializeStreamConfigurations();
	camera_->release();
	return ret;
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
	for (const auto &format : camera3FormatsMap) {
		int androidFormat = format.first;
		const Camera3Format &camera3Format = format.second;
		const std::vector<PixelFormat> &libcameraFormats =
			camera3Format.libcameraFormats;

		/*
		 * Test the libcamera formats that can produce images
		 * compatible with the format defined by Android.
		 */
		PixelFormat mappedFormat;
		for (const PixelFormat &pixelFormat : libcameraFormats) {
			/* \todo Fixed mapping for JPEG. */
			if (androidFormat == HAL_PIXEL_FORMAT_BLOB) {
				mappedFormat = formats::MJPEG;
				break;
			}

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
		if (camera3Format.mandatory && !mappedFormat.isValid()) {
			LOG(HAL, Error) << "Failed to map Android format "
					<< camera3Format.name << " ("
					<< utils::hex(androidFormat) << ")";
			return -EINVAL;
		}

		/*
		 * Record the mapping and then proceed to generate the
		 * stream configurations map, by testing the image resolutions.
		 */
		formatsMap_[androidFormat] = mappedFormat;

		for (const Size &res : cameraResolutions) {
			cfg.pixelFormat = mappedFormat;
			cfg.size = res;

			CameraConfiguration::Status status = cameraConfig->validate();
			/*
			 * Unconditionally report we can produce JPEG.
			 *
			 * \todo The JPEG stream will be implemented as an
			 * HAL-only stream, but some cameras can produce it
			 * directly. As of now, claim support for JPEG without
			 * inspecting where the JPEG stream is produced.
			 */
			if (androidFormat != HAL_PIXEL_FORMAT_BLOB &&
			    status != CameraConfiguration::Valid)
				continue;

			streamConfigurations_.push_back({ res, androidFormat });
		}
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
	camera_->stop();
	camera_->release();

	running_ = false;
}

void CameraDevice::setCallbacks(const camera3_callback_ops_t *callbacks)
{
	callbacks_ = callbacks;
}

std::tuple<uint32_t, uint32_t> CameraDevice::calculateStaticMetadataSize()
{
	/*
	 * \todo Keep this in sync with the actual number of entries.
	 * Currently: 51 entries, 687 bytes of static metadata
	 */
	uint32_t numEntries = 51;
	uint32_t byteSize = 687;

	/*
	 * Calculate space occupation in bytes for dynamically built metadata
	 * entries.
	 *
	 * Each stream configuration entry requires 52 bytes:
	 * 4 32bits integers for ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS
	 * 4 64bits integers for ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS
	 */
	byteSize += streamConfigurations_.size() * 48;

	return std::make_tuple(numEntries, byteSize);
}

/*
 * Return static information for the camera.
 */
const camera_metadata_t *CameraDevice::getStaticMetadata()
{
	if (staticMetadata_)
		return staticMetadata_->get();

	/*
	 * The here reported metadata are enough to implement a basic capture
	 * example application, but a real camera implementation will require
	 * more.
	 */
	uint32_t numEntries;
	uint32_t byteSize;
	std::tie(numEntries, byteSize) = calculateStaticMetadataSize();
	staticMetadata_ = new CameraMetadata(numEntries, byteSize);
	if (!staticMetadata_->isValid()) {
		LOG(HAL, Error) << "Failed to allocate static metadata";
		delete staticMetadata_;
		staticMetadata_ = nullptr;
		return nullptr;
	}

	/* Color correction static metadata. */
	std::vector<uint8_t> aberrationModes = {
		ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
				  aberrationModes.data(),
				  aberrationModes.size());

	/* Control static metadata. */
	std::vector<uint8_t> aeAvailableAntiBandingModes = {
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
				  aeAvailableAntiBandingModes.data(),
				  aeAvailableAntiBandingModes.size());

	std::vector<uint8_t> aeAvailableModes = {
		ANDROID_CONTROL_AE_MODE_ON,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_AVAILABLE_MODES,
				  aeAvailableModes.data(),
				  aeAvailableModes.size());

	std::vector<int32_t> availableAeFpsTarget = {
		15, 30,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
				  availableAeFpsTarget.data(),
				  availableAeFpsTarget.size());

	std::vector<int32_t> aeCompensationRange = {
		0, 0,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_COMPENSATION_RANGE,
				  aeCompensationRange.data(),
				  aeCompensationRange.size());

	const camera_metadata_rational_t aeCompensationStep[] = {
		{ 0, 1 }
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_COMPENSATION_STEP,
				  aeCompensationStep, 1);

	std::vector<uint8_t> availableAfModes = {
		ANDROID_CONTROL_AF_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AF_AVAILABLE_MODES,
				  availableAfModes.data(),
				  availableAfModes.size());

	std::vector<uint8_t> availableEffects = {
		ANDROID_CONTROL_EFFECT_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_EFFECTS,
				  availableEffects.data(),
				  availableEffects.size());

	std::vector<uint8_t> availableSceneModes = {
		ANDROID_CONTROL_SCENE_MODE_DISABLED,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
				  availableSceneModes.data(),
				  availableSceneModes.size());

	std::vector<uint8_t> availableStabilizationModes = {
		ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
				  availableStabilizationModes.data(),
				  availableStabilizationModes.size());

	std::vector<uint8_t> availableAwbModes = {
		ANDROID_CONTROL_AWB_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AWB_AVAILABLE_MODES,
				  availableAwbModes.data(),
				  availableAwbModes.size());

	std::vector<int32_t> availableMaxRegions = {
		0, 0, 0,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_MAX_REGIONS,
				  availableMaxRegions.data(),
				  availableMaxRegions.size());

	std::vector<uint8_t> sceneModesOverride = {
		ANDROID_CONTROL_AE_MODE_ON,
		ANDROID_CONTROL_AWB_MODE_AUTO,
		ANDROID_CONTROL_AF_MODE_AUTO,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_SCENE_MODE_OVERRIDES,
				  sceneModesOverride.data(),
				  sceneModesOverride.size());

	uint8_t aeLockAvailable = ANDROID_CONTROL_AE_LOCK_AVAILABLE_FALSE;
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_LOCK_AVAILABLE,
				  &aeLockAvailable, 1);

	uint8_t awbLockAvailable = ANDROID_CONTROL_AWB_LOCK_AVAILABLE_FALSE;
	staticMetadata_->addEntry(ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
				  &awbLockAvailable, 1);

	char availableControlModes = ANDROID_CONTROL_MODE_AUTO;
	staticMetadata_->addEntry(ANDROID_CONTROL_AVAILABLE_MODES,
				  &availableControlModes, 1);

	/* JPEG static metadata. */
	std::vector<int32_t> availableThumbnailSizes = {
		0, 0,
	};
	staticMetadata_->addEntry(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
				  availableThumbnailSizes.data(),
				  availableThumbnailSizes.size());

	/*
	 * \todo Calculate the maximum JPEG buffer size by asking the encoder
	 * giving the maximum frame size required.
	 */
	staticMetadata_->addEntry(ANDROID_JPEG_MAX_SIZE, &maxJpegBufferSize_, 1);

	/* Sensor static metadata. */
	int32_t pixelArraySize[] = {
		2592, 1944,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
				  &pixelArraySize, 2);

	int32_t sensorSizes[] = {
		0, 0, 2560, 1920,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
				  &sensorSizes, 4);

	int32_t sensitivityRange[] = {
		32, 2400,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
				  &sensitivityRange, 2);

	uint16_t filterArr = ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_GRBG;
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
				  &filterArr, 1);

	int64_t exposureTimeRange[] = {
		100000, 200000000,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
				  &exposureTimeRange, 2);

	staticMetadata_->addEntry(ANDROID_SENSOR_ORIENTATION, &orientation_, 1);

	std::vector<int32_t> testPatterModes = {
		ANDROID_SENSOR_TEST_PATTERN_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
				  testPatterModes.data(),
				  testPatterModes.size());

	std::vector<float> physicalSize = {
		2592, 1944,
	};
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
				  physicalSize.data(),
				  physicalSize.size());

	uint8_t timestampSource = ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
				  &timestampSource, 1);

	/* Statistics static metadata. */
	uint8_t faceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
	staticMetadata_->addEntry(ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
				  &faceDetectMode, 1);

	int32_t maxFaceCount = 0;
	staticMetadata_->addEntry(ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
				  &maxFaceCount, 1);

	/* Sync static metadata. */
	int32_t maxLatency = ANDROID_SYNC_MAX_LATENCY_UNKNOWN;
	staticMetadata_->addEntry(ANDROID_SYNC_MAX_LATENCY, &maxLatency, 1);

	/* Flash static metadata. */
	char flashAvailable = ANDROID_FLASH_INFO_AVAILABLE_FALSE;
	staticMetadata_->addEntry(ANDROID_FLASH_INFO_AVAILABLE,
				  &flashAvailable, 1);

	/* Lens static metadata. */
	std::vector<float> lensApertures = {
		2.53 / 100,
	};
	staticMetadata_->addEntry(ANDROID_LENS_INFO_AVAILABLE_APERTURES,
				  lensApertures.data(),
				  lensApertures.size());

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
	staticMetadata_->addEntry(ANDROID_LENS_FACING, &lensFacing, 1);

	std::vector<float> lensFocalLenghts = {
		1,
	};
	staticMetadata_->addEntry(ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
				  lensFocalLenghts.data(),
				  lensFocalLenghts.size());

	std::vector<uint8_t> opticalStabilizations = {
		ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF,
	};
	staticMetadata_->addEntry(ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
				  opticalStabilizations.data(),
				  opticalStabilizations.size());

	float hypeFocalDistance = 0;
	staticMetadata_->addEntry(ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
				  &hypeFocalDistance, 1);

	float minFocusDistance = 0;
	staticMetadata_->addEntry(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
				  &minFocusDistance, 1);

	/* Noise reduction modes. */
	uint8_t noiseReductionModes = ANDROID_NOISE_REDUCTION_MODE_OFF;
	staticMetadata_->addEntry(ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
				  &noiseReductionModes, 1);

	/* Scaler static metadata. */
	float maxDigitalZoom = 1;
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
				  &maxDigitalZoom, 1);

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
				  availableStallDurations.data(),
				  availableStallDurations.size());

	/* \todo Collect the minimum frame duration from the camera. */
	std::vector<int64_t> minFrameDurations;
	minFrameDurations.reserve(streamConfigurations_.size() * 4);
	for (const auto &entry : streamConfigurations_) {
		minFrameDurations.push_back(entry.androidFormat);
		minFrameDurations.push_back(entry.resolution.width);
		minFrameDurations.push_back(entry.resolution.height);
		minFrameDurations.push_back(33333333);
	}
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
				  minFrameDurations.data(),
				  minFrameDurations.size());

	uint8_t croppingType = ANDROID_SCALER_CROPPING_TYPE_CENTER_ONLY;
	staticMetadata_->addEntry(ANDROID_SCALER_CROPPING_TYPE, &croppingType, 1);

	/* Info static metadata. */
	uint8_t supportedHWLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
	staticMetadata_->addEntry(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
				  &supportedHWLevel, 1);

	/* Request static metadata. */
	int32_t partialResultCount = 1;
	staticMetadata_->addEntry(ANDROID_REQUEST_PARTIAL_RESULT_COUNT,
				  &partialResultCount, 1);

	uint8_t maxPipelineDepth = 2;
	staticMetadata_->addEntry(ANDROID_REQUEST_PIPELINE_MAX_DEPTH,
				  &maxPipelineDepth, 1);

	/* LIMITED does not support reprocessing. */
	uint32_t maxNumInputStreams = 0;
	staticMetadata_->addEntry(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
				  &maxNumInputStreams, 1);

	std::vector<uint8_t> availableCapabilities = {
		ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE,
	};

	/* Report if camera supports RAW. */
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StillCaptureRaw });
	if (cameraConfig && !cameraConfig->empty()) {
		const PixelFormatInfo &info =
			PixelFormatInfo::info(cameraConfig->at(0).pixelFormat);
		if (info.colourEncoding == PixelFormatInfo::ColourEncodingRAW)
			availableCapabilities.push_back(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_RAW);
	}

	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
				  availableCapabilities.data(),
				  availableCapabilities.size());

	std::vector<int32_t> availableCharacteristicsKeys = {
		ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
		ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
		ANDROID_CONTROL_AE_AVAILABLE_MODES,
		ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
		ANDROID_CONTROL_AE_COMPENSATION_RANGE,
		ANDROID_CONTROL_AE_COMPENSATION_STEP,
		ANDROID_CONTROL_AF_AVAILABLE_MODES,
		ANDROID_CONTROL_AVAILABLE_EFFECTS,
		ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
		ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
		ANDROID_CONTROL_AWB_AVAILABLE_MODES,
		ANDROID_CONTROL_MAX_REGIONS,
		ANDROID_CONTROL_SCENE_MODE_OVERRIDES,
		ANDROID_CONTROL_AE_LOCK_AVAILABLE,
		ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
		ANDROID_CONTROL_AVAILABLE_MODES,
		ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
		ANDROID_JPEG_MAX_SIZE,
		ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
		ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
		ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
		ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
		ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
		ANDROID_SENSOR_ORIENTATION,
		ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
		ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
		ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
		ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
		ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
		ANDROID_SYNC_MAX_LATENCY,
		ANDROID_FLASH_INFO_AVAILABLE,
		ANDROID_LENS_INFO_AVAILABLE_APERTURES,
		ANDROID_LENS_FACING,
		ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
		ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
		ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
		ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
		ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
		ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
		ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
		ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
		ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
		ANDROID_SCALER_CROPPING_TYPE,
		ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
		ANDROID_REQUEST_PARTIAL_RESULT_COUNT,
		ANDROID_REQUEST_PIPELINE_MAX_DEPTH,
		ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
		ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
	};
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
				  availableCharacteristicsKeys.data(),
				  availableCharacteristicsKeys.size());

	std::vector<int32_t> availableRequestKeys = {
		ANDROID_CONTROL_AE_MODE,
		ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
		ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER,
		ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
		ANDROID_CONTROL_AE_ANTIBANDING_MODE,
		ANDROID_CONTROL_AE_LOCK,
		ANDROID_CONTROL_AF_TRIGGER,
		ANDROID_CONTROL_AWB_MODE,
		ANDROID_CONTROL_AWB_LOCK,
		ANDROID_FLASH_MODE,
		ANDROID_STATISTICS_FACE_DETECT_MODE,
		ANDROID_NOISE_REDUCTION_MODE,
		ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
		ANDROID_LENS_APERTURE,
		ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
		ANDROID_CONTROL_MODE,
		ANDROID_CONTROL_CAPTURE_INTENT,
	};
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
				  availableRequestKeys.data(),
				  availableRequestKeys.size());

	std::vector<int32_t> availableResultKeys = {
		ANDROID_CONTROL_AE_STATE,
		ANDROID_CONTROL_AE_LOCK,
		ANDROID_CONTROL_AF_STATE,
		ANDROID_CONTROL_AWB_STATE,
		ANDROID_CONTROL_AWB_LOCK,
		ANDROID_LENS_STATE,
		ANDROID_SCALER_CROP_REGION,
		ANDROID_SENSOR_TIMESTAMP,
		ANDROID_SENSOR_ROLLING_SHUTTER_SKEW,
		ANDROID_SENSOR_EXPOSURE_TIME,
		ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
		ANDROID_STATISTICS_SCENE_FLICKER,
		ANDROID_JPEG_SIZE,
		ANDROID_JPEG_QUALITY,
		ANDROID_JPEG_ORIENTATION,
	};
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
				  availableResultKeys.data(),
				  availableResultKeys.size());

	if (!staticMetadata_->isValid()) {
		LOG(HAL, Error) << "Failed to construct static metadata";
		delete staticMetadata_;
		staticMetadata_ = nullptr;
		return nullptr;
	}

	return staticMetadata_->get();
}

CameraMetadata *CameraDevice::requestTemplatePreview()
{
	/*
	 * \todo Keep this in sync with the actual number of entries.
	 * Currently: 20 entries, 35 bytes
	 */
	CameraMetadata *requestTemplate = new CameraMetadata(20, 35);
	if (!requestTemplate->isValid()) {
		delete requestTemplate;
		return nullptr;
	}

	uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_MODE,
				  &aeMode, 1);

	int32_t aeExposureCompensation = 0;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
				  &aeExposureCompensation, 1);

	uint8_t aePrecaptureTrigger = ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER,
				  &aePrecaptureTrigger, 1);

	uint8_t aeLock = ANDROID_CONTROL_AE_LOCK_OFF;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_LOCK,
				  &aeLock, 1);

	std::vector<int32_t> aeFpsTarget = {
		15, 30,
	};
	requestTemplate->addEntry(ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
				  aeFpsTarget.data(),
				  aeFpsTarget.size());

	uint8_t aeAntibandingMode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO;
	requestTemplate->addEntry(ANDROID_CONTROL_AE_ANTIBANDING_MODE,
				  &aeAntibandingMode, 1);

	uint8_t afTrigger = ANDROID_CONTROL_AF_TRIGGER_IDLE;
	requestTemplate->addEntry(ANDROID_CONTROL_AF_TRIGGER,
				  &afTrigger, 1);

	uint8_t awbMode = ANDROID_CONTROL_AWB_MODE_AUTO;
	requestTemplate->addEntry(ANDROID_CONTROL_AWB_MODE,
				  &awbMode, 1);

	uint8_t awbLock = ANDROID_CONTROL_AWB_LOCK_OFF;
	requestTemplate->addEntry(ANDROID_CONTROL_AWB_LOCK,
				  &awbLock, 1);

	uint8_t flashMode = ANDROID_FLASH_MODE_OFF;
	requestTemplate->addEntry(ANDROID_FLASH_MODE,
				  &flashMode, 1);

	uint8_t faceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
	requestTemplate->addEntry(ANDROID_STATISTICS_FACE_DETECT_MODE,
				  &faceDetectMode, 1);

	uint8_t noiseReduction = ANDROID_NOISE_REDUCTION_MODE_OFF;
	requestTemplate->addEntry(ANDROID_NOISE_REDUCTION_MODE,
				  &noiseReduction, 1);

	uint8_t aberrationMode = ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF;
	requestTemplate->addEntry(ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
				  &aberrationMode, 1);

	uint8_t controlMode = ANDROID_CONTROL_MODE_AUTO;
	requestTemplate->addEntry(ANDROID_CONTROL_MODE, &controlMode, 1);

	float lensAperture = 2.53 / 100;
	requestTemplate->addEntry(ANDROID_LENS_APERTURE, &lensAperture, 1);

	uint8_t opticalStabilization = ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF;
	requestTemplate->addEntry(ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
				  &opticalStabilization, 1);

	uint8_t captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
	requestTemplate->addEntry(ANDROID_CONTROL_CAPTURE_INTENT,
				  &captureIntent, 1);

	return requestTemplate;
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
	CameraMetadata *requestTemplate;
	uint8_t captureIntent;
	switch (type) {
	case CAMERA3_TEMPLATE_PREVIEW:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
		break;
	case CAMERA3_TEMPLATE_STILL_CAPTURE:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE;
		break;
	case CAMERA3_TEMPLATE_VIDEO_RECORD:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD;
		break;
	case CAMERA3_TEMPLATE_VIDEO_SNAPSHOT:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT;
		break;
	case CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_ZERO_SHUTTER_LAG;
		break;
	case CAMERA3_TEMPLATE_MANUAL:
		captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_MANUAL;
		break;
	default:
		LOG(HAL, Error) << "Invalid template request type: " << type;
		return nullptr;
	}

	requestTemplate = requestTemplatePreview();
	if (!requestTemplate || !requestTemplate->isValid()) {
		LOG(HAL, Error) << "Failed to construct request template";
		delete requestTemplate;
		return nullptr;
	}

	requestTemplate->updateEntry(ANDROID_CONTROL_CAPTURE_INTENT,
				     &captureIntent, 1);

	requestTemplates_[type] = requestTemplate;
	return requestTemplate->get();
}

PixelFormat CameraDevice::toPixelFormat(int format)
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
	 * re-allcoation.
	 */
	streams_.clear();
	streams_.reserve(stream_list->num_streams);

	/*
	 * Track actually created streams, as there may not be a 1:1 mapping of
	 * camera3 streams to libcamera streams.
	 */
	unsigned int streamIndex = 0;

	/* First handle all non-MJPEG streams. */
	for (unsigned int i = 0; i < stream_list->num_streams; ++i) {
		camera3_stream_t *stream = stream_list->streams[i];
		Size size(stream->width, stream->height);

		PixelFormat format = toPixelFormat(stream->format);

		LOG(HAL, Info) << "Stream #" << i
			       << ", direction: " << stream->stream_type
			       << ", width: " << stream->width
			       << ", height: " << stream->height
			       << ", format: " << utils::hex(stream->format)
			       << " (" << format.toString() << ")";

		if (!format.isValid())
			return -EINVAL;

		streams_.emplace_back(format, size);
		stream->priv = static_cast<void *>(&streams_[i]);

		/* Defer handling of MJPEG streams until all others are known. */
		if (format == formats::MJPEG)
			continue;

		StreamConfiguration streamConfiguration;

		streamConfiguration.size = size;
		streamConfiguration.pixelFormat = format;

		config_->addConfiguration(streamConfiguration);

		streams_[i].index = streamIndex++;
	}

	/* Now handle MJPEG streams, adding a new stream if required. */
	for (unsigned int i = 0; i < stream_list->num_streams; ++i) {
		camera3_stream_t *stream = stream_list->streams[i];
		bool match = false;

		if (streams_[i].format != formats::MJPEG)
			continue;

		/* Search for a compatible stream */
		for (unsigned int j = 0; j < config_->size(); j++) {
			StreamConfiguration &cfg = config_->at(j);

			/*
			 * \todo The PixelFormat must also be compatible with
			 * the encoder.
			 */
			if (cfg.size == streams_[i].size) {
				LOG(HAL, Info) << "Stream " << i
					       << " using libcamera stream " << j;

				match = true;
				streams_[i].index = j;
			}
		}

		/*
		 * Without a compatible match for JPEG encoding we must
		 * introduce a new stream to satisfy the request requirements.
		 */
		if (!match) {
			StreamConfiguration streamConfiguration;

			/*
			 * \todo The pixelFormat should be a 'best-fit' choice
			 * and may require a validation cycle. This is not yet
			 * handled, and should be considered as part of any
			 * stream configuration reworks.
			 */
			streamConfiguration.size.width = stream->width;
			streamConfiguration.size.height = stream->height;
			streamConfiguration.pixelFormat = formats::NV12;

			LOG(HAL, Info) << "Adding " << streamConfiguration.toString()
				       << " for MJPEG support";

			config_->addConfiguration(streamConfiguration);
			streams_[i].index = streamIndex++;
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

	for (unsigned int i = 0; i < stream_list->num_streams; ++i) {
		camera3_stream_t *stream = stream_list->streams[i];
		CameraStream *cameraStream = &streams_[i];
		StreamConfiguration &cfg = config_->at(cameraStream->index);

		/* Use the bufferCount confirmed by the validation process. */
		stream->max_buffers = cfg.bufferCount;

		/*
		 * Construct a software encoder for MJPEG streams from the
		 * chosen libcamera source stream.
		 */
		if (cameraStream->format == formats::MJPEG) {
			cameraStream->jpeg = new EncoderLibJpeg();
			int ret = cameraStream->jpeg->configure(cfg);
			if (ret) {
				LOG(HAL, Error)
					<< "Failed to configure encoder";
				return ret;
			}
		}
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

int CameraDevice::processCaptureRequest(camera3_capture_request_t *camera3Request)
{
	if (!camera3Request->num_output_buffers) {
		LOG(HAL, Error) << "No output buffers provided";
		return -EINVAL;
	}

	/* Start the camera if that's the first request we handle. */
	if (!running_) {
		int ret = camera_->start();
		if (ret) {
			LOG(HAL, Error) << "Failed to start camera";
			return ret;
		}

		running_ = true;
	}

	/*
	 * Queue a request for the Camera with the provided dmabuf file
	 * descriptors.
	 */
	const camera3_stream_buffer_t *camera3Buffers =
					camera3Request->output_buffers;

	/*
	 * Save the request descriptors for use at completion time.
	 * The descriptor and the associated memory reserved here are freed
	 * at request complete time.
	 */
	Camera3RequestDescriptor *descriptor =
		new Camera3RequestDescriptor(camera3Request->frame_number,
					     camera3Request->num_output_buffers);

	Request *request =
		camera_->createRequest(reinterpret_cast<uint64_t>(descriptor));

	for (unsigned int i = 0; i < descriptor->numBuffers; ++i) {
		CameraStream *cameraStream =
			static_cast<CameraStream *>(camera3Buffers[i].stream->priv);

		/*
		 * Keep track of which stream the request belongs to and store
		 * the native buffer handles.
		 */
		descriptor->buffers[i].stream = camera3Buffers[i].stream;
		descriptor->buffers[i].buffer = camera3Buffers[i].buffer;

		/* Software streams are handled after hardware streams complete. */
		if (cameraStream->format == formats::MJPEG)
			continue;

		/*
		 * Create a libcamera buffer using the dmabuf descriptors of
		 * the camera3Buffer for each stream. The FrameBuffer is
		 * directly associated with the Camera3RequestDescriptor for
		 * lifetime management only.
		 */
		FrameBuffer *buffer = createFrameBuffer(*camera3Buffers[i].buffer);
		if (!buffer) {
			LOG(HAL, Error) << "Failed to create buffer";
			delete request;
			delete descriptor;
			return -ENOMEM;
		}
		descriptor->frameBuffers.emplace_back(buffer);

		StreamConfiguration *streamConfiguration = &config_->at(cameraStream->index);
		Stream *stream = streamConfiguration->stream();

		request->addBuffer(stream, buffer);
	}

	int ret = camera_->queueRequest(request);
	if (ret) {
		LOG(HAL, Error) << "Failed to queue request";
		delete request;
		delete descriptor;
		return ret;
	}

	return 0;
}

void CameraDevice::requestComplete(Request *request)
{
	const Request::BufferMap &buffers = request->buffers();
	camera3_buffer_status status = CAMERA3_BUFFER_STATUS_OK;
	std::unique_ptr<CameraMetadata> resultMetadata;
	Camera3RequestDescriptor *descriptor =
		reinterpret_cast<Camera3RequestDescriptor *>(request->cookie());

	if (request->status() != Request::RequestComplete) {
		LOG(HAL, Error) << "Request not successfully completed: "
				<< request->status();
		status = CAMERA3_BUFFER_STATUS_ERROR;
	}

	/*
	 * \todo The timestamp used for the metadata is currently always taken
	 * from the first buffer (which may be the first stream) in the Request.
	 * It might be appropriate to return a 'correct' (as determined by
	 * pipeline handlers) timestamp in the Request itself.
	 */
	FrameBuffer *buffer = buffers.begin()->second;
	resultMetadata = getResultMetadata(descriptor->frameNumber,
					   buffer->metadata().timestamp);

	/* Handle any JPEG compression. */
	for (unsigned int i = 0; i < descriptor->numBuffers; ++i) {
		CameraStream *cameraStream =
			static_cast<CameraStream *>(descriptor->buffers[i].stream->priv);

		if (cameraStream->format != formats::MJPEG)
			continue;

		Encoder *encoder = cameraStream->jpeg;
		if (!encoder) {
			LOG(HAL, Error) << "Failed to identify encoder";
			continue;
		}

		StreamConfiguration *streamConfiguration = &config_->at(cameraStream->index);
		Stream *stream = streamConfiguration->stream();
		FrameBuffer *buffer = request->findBuffer(stream);
		if (!buffer) {
			LOG(HAL, Error) << "Failed to find a source stream buffer";
			continue;
		}

		/*
		 * \todo Buffer mapping and compression should be moved to a
		 * separate thread.
		 */

		MappedCamera3Buffer mapped(*descriptor->buffers[i].buffer,
					   PROT_READ | PROT_WRITE);
		if (!mapped.isValid()) {
			LOG(HAL, Error) << "Failed to mmap android blob buffer";
			continue;
		}

		int jpeg_size = encoder->encode(buffer, mapped.maps()[0]);
		if (jpeg_size < 0) {
			LOG(HAL, Error) << "Failed to encode stream image";
			status = CAMERA3_BUFFER_STATUS_ERROR;
			continue;
		}

		/*
		 * Fill in the JPEG blob header.
		 *
		 * The mapped size of the buffer is being returned as
		 * substantially larger than the requested JPEG_MAX_SIZE
		 * (which is referenced from maxJpegBufferSize_). Utilise
		 * this static size to ensure the correct offset of the blob is
		 * determined.
		 *
		 * \todo Investigate if the buffer size mismatch is an issue or
		 * expected behaviour.
		 */
		uint8_t *resultPtr = mapped.maps()[0].data() +
				     maxJpegBufferSize_ -
				     sizeof(struct camera3_jpeg_blob);
		auto *blob = reinterpret_cast<struct camera3_jpeg_blob *>(resultPtr);
		blob->jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
		blob->jpeg_size = jpeg_size;

		/* Update the JPEG result Metadata. */
		resultMetadata->addEntry(ANDROID_JPEG_SIZE,
					 &jpeg_size, 1);

		const uint32_t jpeg_quality = 95;
		resultMetadata->addEntry(ANDROID_JPEG_QUALITY,
					 &jpeg_quality, 1);

		const uint32_t jpeg_orientation = 0;
		resultMetadata->addEntry(ANDROID_JPEG_ORIENTATION,
					 &jpeg_orientation, 1);
	}

	/* Prepare to call back the Android camera stack. */
	camera3_capture_result_t captureResult = {};
	captureResult.frame_number = descriptor->frameNumber;
	captureResult.num_output_buffers = descriptor->numBuffers;
	for (unsigned int i = 0; i < descriptor->numBuffers; ++i) {
		descriptor->buffers[i].acquire_fence = -1;
		descriptor->buffers[i].release_fence = -1;
		descriptor->buffers[i].status = status;
	}
	captureResult.output_buffers =
		const_cast<const camera3_stream_buffer_t *>(descriptor->buffers);


	if (status == CAMERA3_BUFFER_STATUS_OK) {
		notifyShutter(descriptor->frameNumber,
			      buffer->metadata().timestamp);

		captureResult.partial_result = 1;
		captureResult.result = resultMetadata->get();
	}

	if (status == CAMERA3_BUFFER_STATUS_ERROR || !captureResult.result) {
		/* \todo Improve error handling. In case we notify an error
		 * because the metadata generation fails, a shutter event has
		 * already been notified for this frame number before the error
		 * is here signalled. Make sure the error path plays well with
		 * the camera stack state machine.
		 */
		notifyError(descriptor->frameNumber,
			    descriptor->buffers[0].stream);
	}

	callbacks_->process_capture_result(callbacks_, &captureResult);

	delete descriptor;
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

void CameraDevice::notifyError(uint32_t frameNumber, camera3_stream_t *stream)
{
	camera3_notify_msg_t notify = {};

	/*
	 * \todo Report and identify the stream number or configuration to
	 * clarify the stream that failed.
	 */
	LOG(HAL, Error) << "Error occurred on frame " << frameNumber << " ("
			<< toPixelFormat(stream->format).toString() << ")";

	notify.type = CAMERA3_MSG_ERROR;
	notify.message.error.error_stream = stream;
	notify.message.error.frame_number = frameNumber;
	notify.message.error.error_code = CAMERA3_MSG_ERROR_REQUEST;

	callbacks_->notify(callbacks_, &notify);
}

/*
 * Produce a set of fixed result metadata.
 */
std::unique_ptr<CameraMetadata> CameraDevice::getResultMetadata(int frame_number,
								int64_t timestamp)
{
	/*
	 * \todo Keep this in sync with the actual number of entries.
	 * Currently: 18 entries, 62 bytes
	 */
	std::unique_ptr<CameraMetadata> resultMetadata =
		std::make_unique<CameraMetadata>(18, 62);
	if (!resultMetadata->isValid()) {
		LOG(HAL, Error) << "Failed to allocate static metadata";
		return nullptr;
	}

	const uint8_t ae_state = ANDROID_CONTROL_AE_STATE_CONVERGED;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_STATE, &ae_state, 1);

	const uint8_t ae_lock = ANDROID_CONTROL_AE_LOCK_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_AE_LOCK, &ae_lock, 1);

	uint8_t af_state = ANDROID_CONTROL_AF_STATE_INACTIVE;
	resultMetadata->addEntry(ANDROID_CONTROL_AF_STATE, &af_state, 1);

	const uint8_t awb_state = ANDROID_CONTROL_AWB_STATE_CONVERGED;
	resultMetadata->addEntry(ANDROID_CONTROL_AWB_STATE, &awb_state, 1);

	const uint8_t awb_lock = ANDROID_CONTROL_AWB_LOCK_OFF;
	resultMetadata->addEntry(ANDROID_CONTROL_AWB_LOCK, &awb_lock, 1);

	const uint8_t lens_state = ANDROID_LENS_STATE_STATIONARY;
	resultMetadata->addEntry(ANDROID_LENS_STATE, &lens_state, 1);

	int32_t sensorSizes[] = {
		0, 0, 2560, 1920,
	};
	resultMetadata->addEntry(ANDROID_SCALER_CROP_REGION, sensorSizes, 4);

	resultMetadata->addEntry(ANDROID_SENSOR_TIMESTAMP, &timestamp, 1);

	/* 33.3 msec */
	const int64_t rolling_shutter_skew = 33300000;
	resultMetadata->addEntry(ANDROID_SENSOR_ROLLING_SHUTTER_SKEW,
				 &rolling_shutter_skew, 1);

	/* 16.6 msec */
	const int64_t exposure_time = 16600000;
	resultMetadata->addEntry(ANDROID_SENSOR_EXPOSURE_TIME,
				 &exposure_time, 1);

	const uint8_t lens_shading_map_mode =
				ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_OFF;
	resultMetadata->addEntry(ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
				 &lens_shading_map_mode, 1);

	const uint8_t scene_flicker = ANDROID_STATISTICS_SCENE_FLICKER_NONE;
	resultMetadata->addEntry(ANDROID_STATISTICS_SCENE_FLICKER,
				 &scene_flicker, 1);

	/*
	 * Return the result metadata pack even is not valid: get() will return
	 * nullptr.
	 */
	if (!resultMetadata->isValid()) {
		LOG(HAL, Error) << "Failed to construct result metadata";
	}

	return resultMetadata;
}
