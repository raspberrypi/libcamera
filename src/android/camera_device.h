/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_device.h - libcamera Android Camera Device
 */

#pragma once

#include <map>
#include <memory>
#include <queue>
#include <vector>

#include <hardware/camera3.h>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>
#include <libcamera/base/message.h>
#include <libcamera/base/mutex.h>

#include <libcamera/camera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "camera_capabilities.h"
#include "camera_metadata.h"
#include "camera_stream.h"
#include "jpeg/encoder.h"

class Camera3RequestDescriptor;
struct CameraConfigData;

class CameraDevice : protected libcamera::Loggable
{
public:
	static std::unique_ptr<CameraDevice> create(unsigned int id,
						    std::shared_ptr<libcamera::Camera> cam);
	~CameraDevice();

	int initialize(const CameraConfigData *cameraConfigData);

	int open(const hw_module_t *hardwareModule);
	void close();
	void flush();

	unsigned int id() const { return id_; }
	camera3_device_t *camera3Device() { return &camera3Device_; }
	const CameraCapabilities *capabilities() const { return &capabilities_; }
	const std::shared_ptr<libcamera::Camera> &camera() const { return camera_; }

	const std::string &maker() const { return maker_; }
	const std::string &model() const { return model_; }
	int facing() const { return facing_; }
	int orientation() const { return orientation_; }
	unsigned int maxJpegBufferSize() const;

	void setCallbacks(const camera3_callback_ops_t *callbacks);
	const camera_metadata_t *getStaticMetadata();
	const camera_metadata_t *constructDefaultRequestSettings(int type);
	int configureStreams(camera3_stream_configuration_t *stream_list);
	int processCaptureRequest(camera3_capture_request_t *request);
	void requestComplete(libcamera::Request *request);
	void streamProcessingComplete(Camera3RequestDescriptor::StreamBuffer *bufferStream,
				      Camera3RequestDescriptor::Status status);

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraDevice)

	CameraDevice(unsigned int id, std::shared_ptr<libcamera::Camera> camera);

	enum class State {
		Stopped,
		Flushing,
		Running,
	};

	void stop() LIBCAMERA_TSA_EXCLUDES(stateMutex_);

	std::unique_ptr<libcamera::FrameBuffer>
	createFrameBuffer(const buffer_handle_t camera3buffer,
			  libcamera::PixelFormat pixelFormat,
			  const libcamera::Size &size);
	void abortRequest(Camera3RequestDescriptor *descriptor) const;
	bool isValidRequest(camera3_capture_request_t *request) const;
	void notifyShutter(uint32_t frameNumber, uint64_t timestamp);
	void notifyError(uint32_t frameNumber, camera3_stream_t *stream,
			 camera3_error_msg_code code) const;
	int processControls(Camera3RequestDescriptor *descriptor);
	void completeDescriptor(Camera3RequestDescriptor *descriptor)
		LIBCAMERA_TSA_EXCLUDES(descriptorsMutex_);
	void sendCaptureResults() LIBCAMERA_TSA_REQUIRES(descriptorsMutex_);
	void setBufferStatus(Camera3RequestDescriptor::StreamBuffer &buffer,
			     Camera3RequestDescriptor::Status status);
	std::unique_ptr<CameraMetadata> getResultMetadata(
		const Camera3RequestDescriptor &descriptor) const;

	unsigned int id_;
	camera3_device_t camera3Device_;

	libcamera::Mutex stateMutex_; /* Protects access to the camera state. */
	State state_ LIBCAMERA_TSA_GUARDED_BY(stateMutex_);

	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;
	CameraCapabilities capabilities_;

	std::map<unsigned int, std::unique_ptr<CameraMetadata>> requestTemplates_;
	const camera3_callback_ops_t *callbacks_;

	std::vector<CameraStream> streams_;

	libcamera::Mutex descriptorsMutex_ LIBCAMERA_TSA_ACQUIRED_AFTER(stateMutex_);
	std::queue<std::unique_ptr<Camera3RequestDescriptor>> descriptors_
		LIBCAMERA_TSA_GUARDED_BY(descriptorsMutex_);

	std::string maker_;
	std::string model_;

	int facing_;
	int orientation_;

	CameraMetadata lastSettings_;
};
