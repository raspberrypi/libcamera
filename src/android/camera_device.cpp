/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_device.cpp - libcamera Android Camera Device
 */

#include "camera_device.h"

#include "log.h"
#include "utils.h"

#include "camera_metadata.h"
#include "thread_rpc.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL);

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
}

CameraDevice::Camera3RequestDescriptor::~Camera3RequestDescriptor()
{
	delete[] buffers;
}

/*
 * \class CameraDevice
 *
 * The CameraDevice class wraps a libcamera::Camera instance, and implements
 * the camera_device_t interface by handling RPC requests received from its
 * associated CameraProxy.
 *
 * It translate parameters and operations from Camera HALv3 API to the libcamera
 * ones to provide static information for a Camera, create request templates
 * for it, process capture requests and then deliver capture results back
 * to the framework using the designated callbacks.
 */

CameraDevice::CameraDevice(unsigned int id, const std::shared_ptr<Camera> &camera)
	: running_(false), camera_(camera), staticMetadata_(nullptr)
{
	camera_->requestCompleted.connect(this, &CameraDevice::requestComplete);
}

CameraDevice::~CameraDevice()
{
	if (staticMetadata_)
		delete staticMetadata_;

	for (auto &it : requestTemplates_)
		delete it.second;
}

/*
 * Handle RPC request received from the associated proxy.
 */
void CameraDevice::call(ThreadRpc *rpc)
{
	switch (rpc->tag) {
	case ThreadRpc::ProcessCaptureRequest:
		processCaptureRequest(rpc->request);
		break;
	case ThreadRpc::Close:
		close();
		break;
	default:
		LOG(HAL, Error) << "Unknown RPC operation: " << rpc->tag;
	}

	rpc->notifyReception();
}

int CameraDevice::open()
{
	int ret = camera_->acquire();
	if (ret) {
		LOG(HAL, Error) << "Failed to acquire the camera";
		return ret;
	}

	return 0;
}

void CameraDevice::close()
{
	camera_->stop();

	camera_->freeBuffers();
	camera_->release();

	running_ = false;
}

void CameraDevice::setCallbacks(const camera3_callback_ops_t *callbacks)
{
	callbacks_ = callbacks;
}

/*
 * Return static information for the camera.
 */
camera_metadata_t *CameraDevice::getStaticMetadata()
{
	if (staticMetadata_)
		return staticMetadata_->get();

	/*
	 * The here reported metadata are enough to implement a basic capture
	 * example application, but a real camera implementation will require
	 * more.
	 */

	/*
	 * \todo Keep this in sync with the actual number of entries.
	 * Currently: 47 entries, 390 bytes
	 */
	staticMetadata_ = new CameraMetadata(50, 500);
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

	int32_t orientation = 0;
	staticMetadata_->addEntry(ANDROID_SENSOR_ORIENTATION,
				  &orientation, 1);

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

	uint8_t lensFacing = ANDROID_LENS_FACING_FRONT;
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

	std::vector<uint32_t> availableStreamFormats = {
		ANDROID_SCALER_AVAILABLE_FORMATS_BLOB,
		ANDROID_SCALER_AVAILABLE_FORMATS_YCbCr_420_888,
		ANDROID_SCALER_AVAILABLE_FORMATS_IMPLEMENTATION_DEFINED,
	};
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_FORMATS,
				  availableStreamFormats.data(),
				  availableStreamFormats.size());

	std::vector<uint32_t> availableStreamConfigurations = {
		ANDROID_SCALER_AVAILABLE_FORMATS_BLOB, 2560, 1920,
		ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
		ANDROID_SCALER_AVAILABLE_FORMATS_YCbCr_420_888, 2560, 1920,
		ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
		ANDROID_SCALER_AVAILABLE_FORMATS_IMPLEMENTATION_DEFINED, 2560, 1920,
		ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
	};
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
				  availableStreamConfigurations.data(),
				  availableStreamConfigurations.size());

	std::vector<int64_t> availableStallDurations = {
		ANDROID_SCALER_AVAILABLE_FORMATS_BLOB, 2560, 1920, 33333333,
	};
	staticMetadata_->addEntry(ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
				  availableStallDurations.data(),
				  availableStallDurations.size());

	std::vector<int64_t> minFrameDurations = {
		ANDROID_SCALER_AVAILABLE_FORMATS_BLOB, 2560, 1920, 33333333,
		ANDROID_SCALER_AVAILABLE_FORMATS_IMPLEMENTATION_DEFINED, 2560, 1920, 33333333,
		ANDROID_SCALER_AVAILABLE_FORMATS_YCbCr_420_888, 2560, 1920, 33333333,
	};
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

	std::vector<uint8_t> availableCapabilities = {
		ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE,
	};
	staticMetadata_->addEntry(ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
				  availableCapabilities.data(),
				  availableCapabilities.size());

	if (!staticMetadata_->isValid()) {
		LOG(HAL, Error) << "Failed to construct static metadata";
		delete staticMetadata_;
		staticMetadata_ = nullptr;
		return nullptr;
	}

	return staticMetadata_->get();
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

	/*
	 * \todo Keep this in sync with the actual number of entries.
	 * Currently: 12 entries, 15 bytes
	 */
	CameraMetadata *requestTemplate = new CameraMetadata(15, 20);
	if (!requestTemplate->isValid()) {
		LOG(HAL, Error) << "Failed to allocate template metadata";
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

	requestTemplate->addEntry(ANDROID_CONTROL_CAPTURE_INTENT,
				  &captureIntent, 1);

	if (!requestTemplate->isValid()) {
		LOG(HAL, Error) << "Failed to construct request template";
		delete requestTemplate;
		return nullptr;
	}

	requestTemplates_[type] = requestTemplate;
	return requestTemplate->get();
}

/*
 * Inspect the stream_list to produce a list of StreamConfiguration to
 * be use to configure the Camera.
 */
int CameraDevice::configureStreams(camera3_stream_configuration_t *stream_list)
{
	for (unsigned int i = 0; i < stream_list->num_streams; ++i) {
		camera3_stream_t *stream = stream_list->streams[i];

		LOG(HAL, Info) << "Stream #" << i
			       << ", direction: " << stream->stream_type
			       << ", width: " << stream->width
			       << ", height: " << stream->height
			       << ", format: " << std::hex << stream->format;
	}

	/* Hardcode viewfinder role, collecting sizes from the stream config. */
	if (stream_list->num_streams != 1) {
		LOG(HAL, Error) << "Only one stream supported";
		return -EINVAL;
	}

	StreamRoles roles = { StreamRole::Viewfinder };
	config_ = camera_->generateConfiguration(roles);
	if (!config_ || config_->empty()) {
		LOG(HAL, Error) << "Failed to generate camera configuration";
		return -EINVAL;
	}

	/* Only one stream is supported. */
	camera3_stream_t *camera3Stream = stream_list->streams[0];
	StreamConfiguration *streamConfiguration = &config_->at(0);
	streamConfiguration->size.width = camera3Stream->width;
	streamConfiguration->size.height = camera3Stream->height;
	streamConfiguration->memoryType = ExternalMemory;

	/*
	 * \todo We'll need to translate from Android defined pixel format codes
	 * to the libcamera image format codes. For now, do not change the
	 * format returned from Camera::generateConfiguration().
	 */

	switch (config_->validate()) {
	case CameraConfiguration::Valid:
		break;
	case CameraConfiguration::Adjusted:
		LOG(HAL, Info) << "Camera configuration adjusted";
		config_.reset();
		return -EINVAL;
	case CameraConfiguration::Invalid:
		LOG(HAL, Info) << "Camera configuration invalid";
		config_.reset();
		return -EINVAL;
	}

	camera3Stream->max_buffers = streamConfiguration->bufferCount;

	/*
	 * Once the CameraConfiguration has been adjusted/validated
	 * it can be applied to the camera.
	 */
	int ret = camera_->configure(config_.get());
	if (ret) {
		LOG(HAL, Error) << "Failed to configure camera '"
				<< camera_->name() << "'";
		return ret;
	}

	return 0;
}

int CameraDevice::processCaptureRequest(camera3_capture_request_t *camera3Request)
{
	StreamConfiguration *streamConfiguration = &config_->at(0);
	Stream *stream = streamConfiguration->stream();

	if (camera3Request->num_output_buffers != 1) {
		LOG(HAL, Error) << "Invalid number of output buffers: "
				<< camera3Request->num_output_buffers;
		return -EINVAL;
	}

	/* Start the camera if that's the first request we handle. */
	if (!running_) {
		int ret = camera_->allocateBuffers();
		if (ret) {
			LOG(HAL, Error) << "Failed to allocate buffers";
			return ret;
		}

		ret = camera_->start();
		if (ret) {
			LOG(HAL, Error) << "Failed to start camera";
			camera_->freeBuffers();
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
	for (unsigned int i = 0; i < descriptor->numBuffers; ++i) {
		/*
		 * Keep track of which stream the request belongs to and store
		 * the native buffer handles.
		 *
		 * \todo Currently we only support one capture buffer. Copy
		 * all of them to be ready once we'll support more.
		 */
		descriptor->buffers[i].stream = camera3Buffers[i].stream;
		descriptor->buffers[i].buffer = camera3Buffers[i].buffer;
	}

	/*
	 * Create a libcamera buffer using the dmabuf descriptors of the first
	 * and (currently) only supported request buffer.
	 */
	const buffer_handle_t camera3Handle = *camera3Buffers[0].buffer;
	std::array<int, 3> fds = {
		camera3Handle->data[0],
		camera3Handle->data[1],
		camera3Handle->data[2],
	};

	std::unique_ptr<Buffer> buffer = stream->createBuffer(fds);
	if (!buffer) {
		LOG(HAL, Error) << "Failed to create buffer";
		delete descriptor;
		return -EINVAL;
	}

	Request *request =
		camera_->createRequest(reinterpret_cast<uint64_t>(descriptor));
	request->addBuffer(std::move(buffer));

	int ret = camera_->queueRequest(request);
	if (ret) {
		LOG(HAL, Error) << "Failed to queue request";
		goto error;
	}

	return 0;

error:
	delete request;
	delete descriptor;

	return ret;
}

void CameraDevice::requestComplete(Request *request,
				   const std::map<Stream *, Buffer *> &buffers)
{
	Buffer *libcameraBuffer = buffers.begin()->second;
	camera3_buffer_status status = CAMERA3_BUFFER_STATUS_OK;
	std::unique_ptr<CameraMetadata> resultMetadata;

	if (request->status() != Request::RequestComplete) {
		LOG(HAL, Error) << "Request not succesfully completed: "
				<< request->status();
		status = CAMERA3_BUFFER_STATUS_ERROR;
	}

	/* Prepare to call back the Android camera stack. */
	Camera3RequestDescriptor *descriptor =
		reinterpret_cast<Camera3RequestDescriptor *>(request->cookie());

	camera3_capture_result_t captureResult = {};
	captureResult.frame_number = descriptor->frameNumber;
	captureResult.num_output_buffers = descriptor->numBuffers;
	for (unsigned int i = 0; i < descriptor->numBuffers; ++i) {
		/*
		 * \todo Currently we only support one capture buffer. Prepare
		 * all of them to be ready once we'll support more.
		 */
		descriptor->buffers[i].acquire_fence = -1;
		descriptor->buffers[i].release_fence = -1;
		descriptor->buffers[i].status = status;
	}
	captureResult.output_buffers =
		const_cast<const camera3_stream_buffer_t *>(descriptor->buffers);

	if (status == CAMERA3_BUFFER_STATUS_OK) {
		notifyShutter(descriptor->frameNumber,
			      libcameraBuffer->timestamp());

		captureResult.partial_result = 1;
		resultMetadata = getResultMetadata(descriptor->frameNumber,
						   libcameraBuffer->timestamp());
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
	 * Currently: 12 entries, 36 bytes
	 */
	std::unique_ptr<CameraMetadata> resultMetadata =
		utils::make_unique<CameraMetadata>(15, 50);
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
