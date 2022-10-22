/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_capabilities.cpp - Camera static properties manager
 */

#include "camera_capabilities.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <type_traits>

#include <hardware/camera3.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/property_ids.h>

#include "libcamera/internal/formats.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

namespace {

/*
 * \var camera3Resolutions
 * \brief The list of image resolutions commonly supported by Android
 *
 * The following are defined as mandatory to be supported by the Android
 * Camera3 specification: (320x240), (640x480), (1280x720), (1920x1080).
 *
 * The following 4:3 resolutions are defined as optional, but commonly
 * supported by Android devices: (1280x960), (1600x1200).
 */
const std::vector<Size> camera3Resolutions = {
	{ 320, 240 },
	{ 640, 480 },
	{ 1280, 720 },
	{ 1280, 960 },
	{ 1600, 1200 },
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
	},
};

const std::map<camera_metadata_enum_android_info_supported_hardware_level, std::string>
hwLevelStrings = {
	{ ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED,  "LIMITED" },
	{ ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_FULL,     "FULL" },
	{ ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY,   "LEGACY" },
	{ ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_3,        "LEVEL_3" },
	{ ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL, "EXTERNAL" },
};

enum class ControlRange {
	Min,
	Def,
	Max,
};

/**
 * \brief Set Android metadata from libcamera ControlInfo or a default value
 * \tparam T Type of the control in libcamera
 * \tparam U Type of the metadata in Android
 * \param[in] metadata Android metadata pack to add the control value to
 * \param[in] tag Android metadata tag
 * \param[in] controlsInfo libcamera ControlInfoMap from which to find the control info
 * \param[in] control libcamera ControlId to find from \a controlsInfo
 * \param[in] controlRange Whether to use the min, def, or max value from the control info
 * \param[in] defaultValue The value to set in \a metadata if \a control is not found
 *
 * Set the Android metadata entry in \a metadata with tag \a tag based on the
 * control info found for the libcamera control \a control in the libcamera
 * ControlInfoMap \a controlsInfo. If no libcamera ControlInfo is found, then
 * the Android metadata entry is set to \a defaultValue.
 *
 * This function is for scalar values.
 */
template<typename T, typename U>
U setMetadata(CameraMetadata *metadata, uint32_t tag,
	      const ControlInfoMap &controlsInfo, const Control<T> &control,
	      enum ControlRange controlRange, const U defaultValue)
{
	U value = defaultValue;

	const auto &info = controlsInfo.find(&control);
	if (info != controlsInfo.end()) {
		switch (controlRange) {
		case ControlRange::Min:
			value = static_cast<U>(info->second.min().template get<T>());
			break;
		case ControlRange::Def:
			value = static_cast<U>(info->second.def().template get<T>());
			break;
		case ControlRange::Max:
			value = static_cast<U>(info->second.max().template get<T>());
			break;
		}
	}

	metadata->addEntry(tag, value);
	return value;
}

/**
 * \brief Set Android metadata from libcamera ControlInfo or a default value
 * \tparam T Type of the control in libcamera
 * \tparam U Type of the metadata in Android
 * \param[in] metadata Android metadata pack to add the control value to
 * \param[in] tag Android metadata tag
 * \param[in] controlsInfo libcamera ControlInfoMap from which to find the control info
 * \param[in] control libcamera ControlId to find from \a controlsInfo
 * \param[in] defaultVector The value to set in \a metadata if \a control is not found
 *
 * Set the Android metadata entry in \a metadata with tag \a tag based on the
 * control info found for the libcamera control \a control in the libcamera
 * ControlInfoMap \a controlsInfo. If no libcamera ControlInfo is found, then
 * the Android metadata entry is set to \a defaultVector.
 *
 * This function is for vector values.
 */
template<typename T, typename U>
std::vector<U> setMetadata(CameraMetadata *metadata, uint32_t tag,
			   const ControlInfoMap &controlsInfo,
			   const Control<T> &control,
			   const std::vector<U> &defaultVector)
{
	const auto &info = controlsInfo.find(&control);
	if (info == controlsInfo.end()) {
		metadata->addEntry(tag, defaultVector);
		return defaultVector;
	}

	std::vector<U> values(info->second.values().size());
	for (const auto &value : info->second.values())
		values.push_back(static_cast<U>(value.template get<T>()));
	metadata->addEntry(tag, values);

	return values;
}

} /* namespace */

bool CameraCapabilities::validateManualSensorCapability()
{
	const char *noMode = "Manual sensor capability unavailable: ";

	if (!staticMetadata_->entryContains<uint8_t>(ANDROID_CONTROL_AE_AVAILABLE_MODES,
						     ANDROID_CONTROL_AE_MODE_OFF)) {
		LOG(HAL, Info) << noMode << "missing AE mode off";
		return false;
	}

	if (!staticMetadata_->entryContains<uint8_t>(ANDROID_CONTROL_AE_LOCK_AVAILABLE,
						     ANDROID_CONTROL_AE_LOCK_AVAILABLE_TRUE)) {
		LOG(HAL, Info) << noMode << "missing AE lock";
		return false;
	}

	/*
	 * \todo Return true here after we satisfy all the requirements:
	 * https://developer.android.com/reference/android/hardware/camera2/CameraMetadata#REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR
	 * Manual frame duration control
	 *     android.sensor.frameDuration
	 *     android.sensor.info.maxFrameDuration
	 * Manual exposure control
	 *     android.sensor.exposureTime
	 *     android.sensor.info.exposureTimeRange
	 * Manual sensitivity control
	 *     android.sensor.sensitivity
	 *     android.sensor.info.sensitivityRange
	 * Manual lens control (if the lens is adjustable)
	 *     android.lens.*
	 * Manual flash control (if a flash unit is present)
	 *     android.flash.*
	 * Manual black level locking
	 *     android.blackLevel.lock
	 * Auto exposure lock
	 *     android.control.aeLock
	 */
	return false;
}

bool CameraCapabilities::validateManualPostProcessingCapability()
{
	const char *noMode = "Manual post processing capability unavailable: ";

	if (!staticMetadata_->entryContains<uint8_t>(ANDROID_CONTROL_AWB_AVAILABLE_MODES,
						     ANDROID_CONTROL_AWB_MODE_OFF)) {
		LOG(HAL, Info) << noMode << "missing AWB mode off";
		return false;
	}

	if (!staticMetadata_->entryContains<uint8_t>(ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
						     ANDROID_CONTROL_AWB_LOCK_AVAILABLE_TRUE)) {
		LOG(HAL, Info) << noMode << "missing AWB lock";
		return false;
	}

	/*
	 * \todo return true here after we satisfy all the requirements:
	 * https://developer.android.com/reference/android/hardware/camera2/CameraMetadata#REQUEST_AVAILABLE_CAPABILITIES_MANUAL_POST_PROCESSING
	 * Manual tonemap control
	 *     android.tonemap.curve
	 *     android.tonemap.mode
	 *     android.tonemap.maxCurvePoints
	 *     android.tonemap.gamma
	 *     android.tonemap.presetCurve
	 * Manual white balance control
	 *     android.colorCorrection.transform
	 *     android.colorCorrection.gains
	 * Manual lens shading map control
	 *     android.shading.mode
	 *     android.statistics.lensShadingMapMode
	 *     android.statistics.lensShadingMap
	 *     android.lens.info.shadingMapSize
	 * Manual aberration correction control (if aberration correction is supported)
	 *     android.colorCorrection.aberrationMode
	 *     android.colorCorrection.availableAberrationModes
	 * Auto white balance lock
	 *     android.control.awbLock
	 */
	return false;
}

bool CameraCapabilities::validateBurstCaptureCapability()
{
	camera_metadata_ro_entry_t entry;
	bool found;

	const char *noMode = "Burst capture capability unavailable: ";

	if (!staticMetadata_->entryContains<uint8_t>(ANDROID_CONTROL_AE_LOCK_AVAILABLE,
						     ANDROID_CONTROL_AE_LOCK_AVAILABLE_TRUE)) {
		LOG(HAL, Info) << noMode << "missing AE lock";
		return false;
	}

	if (!staticMetadata_->entryContains<uint8_t>(ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
						     ANDROID_CONTROL_AWB_LOCK_AVAILABLE_TRUE)) {
		LOG(HAL, Info) << noMode << "missing AWB lock";
		return false;
	}

	found = staticMetadata_->getEntry(ANDROID_SYNC_MAX_LATENCY, &entry);
	if (!found || *entry.data.i32 < 0 || 4 < *entry.data.i32) {
		LOG(HAL, Info)
			<< noMode << "max sync latency is "
			<< (found ? std::to_string(*entry.data.i32) : "not present");
		return false;
	}

	/*
	 * \todo return true here after we satisfy all the requirements
	 * https://developer.android.com/reference/android/hardware/camera2/CameraMetadata#REQUEST_AVAILABLE_CAPABILITIES_BURST_CAPTURE
	 */
	return false;
}

std::set<camera_metadata_enum_android_request_available_capabilities>
CameraCapabilities::computeCapabilities()
{
	std::set<camera_metadata_enum_android_request_available_capabilities>
		capabilities;

	capabilities.insert(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE);

	if (validateManualSensorCapability()) {
		capabilities.insert(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR);
		/* The requirements for READ_SENSOR_SETTINGS are a subset of MANUAL_SENSOR */
		capabilities.insert(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_READ_SENSOR_SETTINGS);
	}

	if (validateManualPostProcessingCapability())
		capabilities.insert(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_POST_PROCESSING);

	if (validateBurstCaptureCapability())
		capabilities.insert(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BURST_CAPTURE);

	if (rawStreamAvailable_)
		capabilities.insert(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_RAW);

	return capabilities;
}

void CameraCapabilities::computeHwLevel(
	const std::set<camera_metadata_enum_android_request_available_capabilities> &caps)
{
	const char *noFull = "Hardware level FULL unavailable: ";
	camera_metadata_ro_entry_t entry;
	bool found;

	camera_metadata_enum_android_info_supported_hardware_level
		hwLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_FULL;

	if (!caps.count(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR)) {
		LOG(HAL, Info) << noFull << "missing manual sensor";
		hwLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
	}

	if (!caps.count(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_POST_PROCESSING)) {
		LOG(HAL, Info) << noFull << "missing manual post processing";
		hwLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
	}

	if (!caps.count(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BURST_CAPTURE)) {
		LOG(HAL, Info) << noFull << "missing burst capture";
		hwLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
	}

	found = staticMetadata_->getEntry(ANDROID_SYNC_MAX_LATENCY, &entry);
	if (!found || *entry.data.i32 != 0) {
		LOG(HAL, Info) << noFull << "missing or invalid max sync latency";
		hwLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
	}

	hwLevel_ = hwLevel;
}

int CameraCapabilities::initialize(std::shared_ptr<Camera> camera,
				   int orientation, int facing)
{
	camera_ = camera;
	orientation_ = orientation;
	facing_ = facing;
	rawStreamAvailable_ = false;
	maxFrameDuration_ = 0;

	/* Acquire the camera and initialize available stream configurations. */
	int ret = camera_->acquire();
	if (ret) {
		LOG(HAL, Error) << "Failed to temporarily acquire the camera";
		return ret;
	}

	ret = initializeStreamConfigurations();
	if (ret) {
		camera_->release();
		return ret;
	}

	ret = initializeStaticMetadata();
	camera_->release();
	return ret;
}

std::vector<Size>
CameraCapabilities::initializeYUVResolutions(const PixelFormat &pixelFormat,
					     const std::vector<Size> &resolutions)
{
	std::vector<Size> supportedResolutions;
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StreamRole::Viewfinder });
	if (!cameraConfig) {
		LOG(HAL, Error) << "Failed to get supported YUV resolutions";
		return supportedResolutions;
	}

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

std::vector<Size>
CameraCapabilities::initializeRawResolutions(const PixelFormat &pixelFormat)
{
	std::vector<Size> supportedResolutions;
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StreamRole::Raw });
	if (!cameraConfig) {
		LOG(HAL, Error) << "Failed to get supported Raw resolutions";
		return supportedResolutions;
	}

	StreamConfiguration &cfg = cameraConfig->at(0);
	const StreamFormats &formats = cfg.formats();
	supportedResolutions = formats.sizes(pixelFormat);

	return supportedResolutions;
}

/*
 * Initialize the format conversion map to translate from Android format
 * identifier to libcamera pixel formats and fill in the list of supported
 * stream configurations to be reported to the Android camera framework through
 * the camera static metadata.
 */
int CameraCapabilities::initializeStreamConfigurations()
{
	/*
	 * Get the maximum output resolutions
	 * \todo Get this from the camera properties once defined
	 */
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StreamRole::StillCapture });
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
	LOG(HAL, Debug) << "Maximum supported resolution: " << maxRes;

	/*
	 * Build the list of supported image resolutions.
	 *
	 * The resolutions listed in camera3Resolution are supported, up to the
	 * camera maximum resolution.
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
					<< formats::MJPEG
					<< " (fixed mapping)";
			continue;
		}

		/*
		 * Test the libcamera formats that can produce images
		 * compatible with the format defined by Android.
		 */
		PixelFormat mappedFormat;
		for (const PixelFormat &pixelFormat : libcameraFormats) {

			LOG(HAL, Debug) << "Testing " << pixelFormat;

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
				<< mappedFormat;

		std::vector<Size> resolutions;
		const PixelFormatInfo &info = PixelFormatInfo::info(mappedFormat);
		switch (info.colourEncoding) {
		case PixelFormatInfo::ColourEncodingRAW:
			if (info.bitsPerPixel != 16)
				continue;

			rawStreamAvailable_ = true;
			resolutions = initializeRawResolutions(mappedFormat);
			break;

		case PixelFormatInfo::ColourEncodingYUV:
		case PixelFormatInfo::ColourEncodingRGB:
			/*
			 * We support enumerating RGB streams here to allow
			 * mapping IMPLEMENTATION_DEFINED format to RGB.
			 */
			resolutions = initializeYUVResolutions(mappedFormat,
							       cameraResolutions);
			break;
		}

		for (const Size &res : resolutions) {
			/*
			 * Configure the Camera with the collected format and
			 * resolution to get an updated list of controls.
			 *
			 * \todo Avoid the need to configure the camera when
			 * redesigning the configuration API.
			 */
			cfg.size = res;
			int ret = camera_->configure(cameraConfig.get());
			if (ret)
				return ret;

			const ControlInfoMap &controls = camera_->controls();
			const auto frameDurations = controls.find(
				&controls::FrameDurationLimits);
			if (frameDurations == controls.end()) {
				LOG(HAL, Error)
					<< "Camera does not report frame durations";
				return -EINVAL;
			}

			int64_t minFrameDuration = frameDurations->second.min().get<int64_t>() * 1000;
			int64_t maxFrameDuration = frameDurations->second.max().get<int64_t>() * 1000;

			/*
			 * Cap min frame duration to 30 FPS with 1% tolerance.
			 *
			 * 30 frames per second has been validated as the most
			 * opportune frame rate for quality tuning, and power
			 * vs performances budget on Intel IPU3-based
			 * Chromebooks.
			 *
			 * \todo This is a platform-specific decision that needs
			 * to be abstracted and delegated to the configuration
			 * file.
			 *
			 * \todo libcamera only allows to control frame duration
			 * through the per-request controls::FrameDuration
			 * control. If we cap the durations here, we should be
			 * capable of configuring the camera to operate at such
			 * duration without requiring to have the FrameDuration
			 * control to be specified for each Request. Defer this
			 * to the in-development configuration API rework.
			 */
			int64_t minFrameDurationCap = 1e9 / 30.0;
			if (minFrameDuration < minFrameDurationCap) {
				float tolerance =
					(minFrameDurationCap - minFrameDuration) * 100.0 / minFrameDurationCap;

				/*
				 * If the tolerance is less than 1%, do not cap
				 * the frame duration.
				 */
				if (tolerance > 1.0)
					minFrameDuration = minFrameDurationCap;
			}

			/*
			 * Calculate FPS as CTS does and adjust the minimum
			 * frame duration accordingly: see
			 * Camera2SurfaceViewTestCase.java:getSuitableFpsRangeForDuration()
			 */
			minFrameDuration =
				1e9 / static_cast<unsigned int>(floor(1e9 / minFrameDuration + 0.05f));

			streamConfigurations_.push_back({
				res, androidFormat, minFrameDuration, maxFrameDuration,
			});

			/*
			 * If the format is HAL_PIXEL_FORMAT_YCbCr_420_888
			 * from which JPEG is produced, add an entry for
			 * the JPEG stream.
			 *
			 * \todo Wire the JPEG encoder to query the supported
			 * sizes provided a list of formats it can encode.
			 *
			 * \todo Support JPEG streams produced by the camera
			 * natively.
			 *
			 * \todo HAL_PIXEL_FORMAT_BLOB is a 'stalling' format,
			 * its duration should take into account the time
			 * required for the YUV to JPEG encoding. For now
			 * use the same frame durations as collected for
			 * the YUV/RGB streams.
			 */
			if (androidFormat == HAL_PIXEL_FORMAT_YCbCr_420_888) {
				streamConfigurations_.push_back({
					res, HAL_PIXEL_FORMAT_BLOB,
					minFrameDuration, maxFrameDuration,
				});
				maxJpegSize = std::max(maxJpegSize, res);
			}

			maxFrameDuration_ = std::max(maxFrameDuration_,
						     maxFrameDuration);
		}

		/*
		 * \todo Calculate the maximum JPEG buffer size by asking the
		 * encoder giving the maximum frame size required.
		 */
		maxJpegBufferSize_ = maxJpegSize.width * maxJpegSize.height * 1.5;
	}

	LOG(HAL, Debug) << "Collected stream configuration map: ";
	for (const auto &entry : streamConfigurations_)
		LOG(HAL, Debug) << "{ " << entry.resolution << " - "
				<< utils::hex(entry.androidFormat) << " }";

	return 0;
}

int CameraCapabilities::initializeStaticMetadata()
{
	staticMetadata_ = std::make_unique<CameraMetadata>(64, 1024);
	if (!staticMetadata_->isValid()) {
		LOG(HAL, Error) << "Failed to allocate static metadata";
		staticMetadata_.reset();
		return -EINVAL;
	}

	/*
	 * Generate and apply a new configuration for the Viewfinder role to
	 * collect control limits and properties from a known state.
	 */
	std::unique_ptr<CameraConfiguration> cameraConfig =
		camera_->generateConfiguration({ StreamRole::Viewfinder });
	if (!cameraConfig) {
		LOG(HAL, Error) << "Failed to generate camera configuration";
		staticMetadata_.reset();
		return -ENODEV;
	}

	int ret = camera_->configure(cameraConfig.get());
	if (ret) {
		LOG(HAL, Error) << "Failed to initialize the camera state";
		staticMetadata_.reset();
		return ret;
	}

	const ControlInfoMap &controlsInfo = camera_->controls();
	const ControlList &properties = camera_->properties();

	availableCharacteristicsKeys_ = {
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

	availableRequestKeys_ = {
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

	availableResultKeys_ = {
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

	std::vector<int32_t> aeCompensationRange = {
		0, 0,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_COMPENSATION_RANGE,
				  aeCompensationRange);

	const camera_metadata_rational_t aeCompensationStep[] = {
		{ 0, 1 }
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_COMPENSATION_STEP,
				  aeCompensationStep);

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
	 * \todo Inspect the camera capabilities to report the available
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
		const Size &size = properties.get(properties::PixelArraySize).value_or(Size{});
		pixelArraySize[0] = size.width;
		pixelArraySize[1] = size.height;
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
					  pixelArraySize);
	}

	const auto &cellSize = properties.get<Size>(properties::UnitCellSize);
	if (cellSize) {
		std::array<float, 2> physicalSize{
			cellSize->width * pixelArraySize[0] / 1e6f,
			cellSize->height * pixelArraySize[1] / 1e6f
		};
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
					  physicalSize);
	}

	{
		const Span<const Rectangle> &rects =
			properties.get(properties::PixelArrayActiveAreas).value_or(Span<const Rectangle>{});
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
	const auto &filterArr = properties.get(properties::draft::ColorFilterArrangement);
	if (filterArr)
		staticMetadata_->addEntry(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
					  *filterArr);

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

	std::vector<int32_t> testPatternModes = {
		ANDROID_SENSOR_TEST_PATTERN_MODE_OFF
	};
	const auto &testPatternsInfo =
		controlsInfo.find(&controls::draft::TestPatternMode);
	if (testPatternsInfo != controlsInfo.end()) {
		const auto &values = testPatternsInfo->second.values();
		ASSERT(!values.empty());
		for (const auto &value : values) {
			switch (value.get<int32_t>()) {
			case controls::draft::TestPatternModeOff:
				/*
				 * ANDROID_SENSOR_TEST_PATTERN_MODE_OFF is
				 * already in testPatternModes.
				 */
				break;

			case controls::draft::TestPatternModeSolidColor:
				testPatternModes.push_back(
					ANDROID_SENSOR_TEST_PATTERN_MODE_SOLID_COLOR);
				break;

			case controls::draft::TestPatternModeColorBars:
				testPatternModes.push_back(
					ANDROID_SENSOR_TEST_PATTERN_MODE_COLOR_BARS);
				break;

			case controls::draft::TestPatternModeColorBarsFadeToGray:
				testPatternModes.push_back(
					ANDROID_SENSOR_TEST_PATTERN_MODE_COLOR_BARS_FADE_TO_GRAY);
				break;

			case controls::draft::TestPatternModePn9:
				testPatternModes.push_back(
					ANDROID_SENSOR_TEST_PATTERN_MODE_PN9);
				break;

			case controls::draft::TestPatternModeCustom1:
				/* We don't support this yet. */
				break;

			default:
				LOG(HAL, Error) << "Unknown test pattern mode: "
						<< value.get<int32_t>();
				continue;
			}
		}
	}
	staticMetadata_->addEntry(ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
				  testPatternModes);

	uint8_t timestampSource = ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
				  timestampSource);

	staticMetadata_->addEntry(ANDROID_SENSOR_INFO_MAX_FRAME_DURATION,
				  maxFrameDuration_);

	/* Statistics static metadata. */
	uint8_t faceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
	staticMetadata_->addEntry(ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
				  faceDetectMode);

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
	setMetadata(staticMetadata_.get(), ANDROID_SYNC_MAX_LATENCY,
		    controlsInfo, controls::draft::MaxLatency,
		    ControlRange::Def,
		    ANDROID_SYNC_MAX_LATENCY_UNKNOWN);

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
	std::vector<int64_t> minFrameDurations;
	int maxYUVFps = 0;
	Size maxYUVSize;

	availableStreamConfigurations.reserve(streamConfigurations_.size() * 4);
	minFrameDurations.reserve(streamConfigurations_.size() * 4);

	for (const auto &entry : streamConfigurations_) {
		/*
		 * Filter out YUV streams not capable of running at 30 FPS.
		 *
		 * This requirement comes from CTS RecordingTest failures most
		 * probably related to a requirement of the camcoder video
		 * recording profile. Inspecting the Intel IPU3 HAL
		 * implementation confirms this but no reference has been found
		 * in the metadata documentation.
		 */
		unsigned int fps =
			static_cast<unsigned int>(floor(1e9 / entry.minFrameDurationNsec));

		if (entry.androidFormat != HAL_PIXEL_FORMAT_BLOB && fps < 30)
			continue;

		/*
		 * Collect the FPS of the maximum YUV output size to populate
		 * AE_AVAILABLE_TARGET_FPS_RANGE
		 */
		if (entry.androidFormat == HAL_PIXEL_FORMAT_YCbCr_420_888 &&
		    entry.resolution > maxYUVSize) {
			maxYUVSize = entry.resolution;
			maxYUVFps = fps;
		}

		/* Stream configuration map. */
		availableStreamConfigurations.push_back(entry.androidFormat);
		availableStreamConfigurations.push_back(entry.resolution.width);
		availableStreamConfigurations.push_back(entry.resolution.height);
		availableStreamConfigurations.push_back(
			ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT);

		/* Per-stream durations. */
		minFrameDurations.push_back(entry.androidFormat);
		minFrameDurations.push_back(entry.resolution.width);
		minFrameDurations.push_back(entry.resolution.height);
		minFrameDurations.push_back(entry.minFrameDurationNsec);

		LOG(HAL, Debug)
			<< "Output Stream: " << utils::hex(entry.androidFormat)
			<< " (" << entry.resolution << ")["
			<< entry.minFrameDurationNsec << "]"
			<< "@" << fps;
	}
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
				  availableStreamConfigurations);

	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
				  minFrameDurations);

	/*
	 * Register to the camera service {min, max} and {max, max} with
	 * 'max' being the larger YUV stream maximum frame rate and 'min' being
	 * the globally minimum frame rate rounded to the next largest integer
	 * as the camera service expects the camera maximum frame duration to be
	 * smaller than 10^9 / minFps.
	 */
	int32_t minFps = std::ceil(1e9 / maxFrameDuration_);
	int32_t availableAeFpsTarget[] = {
		minFps, maxYUVFps, maxYUVFps, maxYUVFps,
	};
	staticMetadata_->addEntry(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
				  availableAeFpsTarget);

	std::vector<int64_t> availableStallDurations;
	for (const auto &entry : streamConfigurations_) {
		if (entry.androidFormat != HAL_PIXEL_FORMAT_BLOB)
			continue;

		availableStallDurations.push_back(entry.androidFormat);
		availableStallDurations.push_back(entry.resolution.width);
		availableStallDurations.push_back(entry.resolution.height);
		availableStallDurations.push_back(entry.minFrameDurationNsec);
	}
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
				  availableStallDurations);

	uint8_t croppingType = ANDROID_SCALER_CROPPING_TYPE_CENTER_ONLY;
	staticMetadata_->addEntry(ANDROID_SCALER_CROPPING_TYPE, croppingType);

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

	/* Number of { RAW, YUV, JPEG } supported output streams */
	int32_t numOutStreams[] = { rawStreamAvailable_, 2, 1 };
	staticMetadata_->addEntry(ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS,
				  numOutStreams);

	/* Check capabilities */
	capabilities_ = computeCapabilities();
	/* This *must* be uint8_t. */
	std::vector<uint8_t> capsVec(capabilities_.begin(), capabilities_.end());
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_CAPABILITIES, capsVec);

	computeHwLevel(capabilities_);
	staticMetadata_->addEntry(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL, hwLevel_);

	LOG(HAL, Info)
		<< "Hardware level: " << hwLevelStrings.find(hwLevel_)->second;

	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
				  std::vector<int32_t>(availableCharacteristicsKeys_.begin(),
						       availableCharacteristicsKeys_.end()));

	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
				  std::vector<int32_t>(availableRequestKeys_.begin(),
						       availableRequestKeys_.end()));

	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
				  std::vector<int32_t>(availableResultKeys_.begin(),
						       availableResultKeys_.end()));

	if (!staticMetadata_->isValid()) {
		LOG(HAL, Error) << "Failed to construct static metadata";
		staticMetadata_.reset();
		return -EINVAL;
	}

	if (staticMetadata_->resized()) {
		auto [entryCount, dataCount] = staticMetadata_->usage();
		LOG(HAL, Info)
			<< "Static metadata resized: " << entryCount
			<< " entries and " << dataCount << " bytes used";
	}

	return 0;
}

/* Translate Android format code to libcamera pixel format. */
PixelFormat CameraCapabilities::toPixelFormat(int format) const
{
	auto it = formatsMap_.find(format);
	if (it == formatsMap_.end()) {
		LOG(HAL, Error) << "Requested format " << utils::hex(format)
				<< " not supported";
		return PixelFormat();
	}

	return it->second;
}

std::unique_ptr<CameraMetadata> CameraCapabilities::requestTemplateManual() const
{
	if (!capabilities_.count(ANDROID_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR)) {
		LOG(HAL, Error) << "Manual template not supported";
		return nullptr;
	}

	std::unique_ptr<CameraMetadata> manualTemplate = requestTemplatePreview();
	if (!manualTemplate)
		return nullptr;

	return manualTemplate;
}

std::unique_ptr<CameraMetadata> CameraCapabilities::requestTemplatePreview() const
{
	/*
	 * Give initial hint of entries and number of bytes to be allocated.
	 * It is deliberate that the hint is slightly larger than required, to
	 * avoid resizing the container.
	 *
	 * CameraMetadata is capable of resizing the container on the fly, if
	 * adding a new entry will exceed its capacity.
	 */
	auto requestTemplate = std::make_unique<CameraMetadata>(22, 38);
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

	/*
	 * Get thumbnail sizes from static metadata and add the first non-zero
	 * size to the template.
	 */
	found = staticMetadata_->getEntry(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
					  &entry);
	ASSERT(found && entry.count >= 4);
	requestTemplate->addEntry(ANDROID_JPEG_THUMBNAIL_SIZE,
				  entry.data.i32 + 2, 2);

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

std::unique_ptr<CameraMetadata> CameraCapabilities::requestTemplateStill() const
{
	std::unique_ptr<CameraMetadata> stillTemplate = requestTemplatePreview();
	if (!stillTemplate)
		return nullptr;

	return stillTemplate;
}

std::unique_ptr<CameraMetadata> CameraCapabilities::requestTemplateVideo() const
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
