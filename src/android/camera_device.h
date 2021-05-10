/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_device.h - libcamera Android Camera Device
 */
#ifndef __ANDROID_CAMERA_DEVICE_H__
#define __ANDROID_CAMERA_DEVICE_H__

#include <map>
#include <memory>
#include <mutex>
#include <tuple>
#include <vector>

#include <hardware/camera3.h>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/buffer.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/message.h"
#include "libcamera/internal/thread.h"

#include "camera_metadata.h"
#include "camera_stream.h"
#include "camera_worker.h"
#include "jpeg/encoder.h"

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

	unsigned int id() const { return id_; }
	camera3_device_t *camera3Device() { return &camera3Device_; }
	const std::shared_ptr<libcamera::Camera> &camera() const { return camera_; }
	libcamera::CameraConfiguration *cameraConfiguration() const
	{
		return config_.get();
	}

	const std::string &maker() const { return maker_; }
	const std::string &model() const { return model_; }
	int facing() const { return facing_; }
	int orientation() const { return orientation_; }
	unsigned int maxJpegBufferSize() const { return maxJpegBufferSize_; }

	void setCallbacks(const camera3_callback_ops_t *callbacks);
	const camera_metadata_t *getStaticMetadata();
	const camera_metadata_t *constructDefaultRequestSettings(int type);
	int configureStreams(camera3_stream_configuration_t *stream_list);
	int processCaptureRequest(camera3_capture_request_t *request);
	void requestComplete(libcamera::Request *request);

protected:
	std::string logPrefix() const override;

private:
	CameraDevice(unsigned int id, std::shared_ptr<libcamera::Camera> camera);

	struct Camera3RequestDescriptor {
		Camera3RequestDescriptor() = default;
		~Camera3RequestDescriptor() = default;
		Camera3RequestDescriptor(libcamera::Camera *camera,
					 const camera3_capture_request_t *camera3Request);
		Camera3RequestDescriptor &operator=(Camera3RequestDescriptor &&) = default;

		uint32_t frameNumber_ = 0;
		std::vector<camera3_stream_buffer_t> buffers_;
		std::vector<std::unique_ptr<libcamera::FrameBuffer>> frameBuffers_;
		CameraMetadata settings_;
		std::unique_ptr<CaptureRequest> request_;
	};

	struct Camera3StreamConfiguration {
		libcamera::Size resolution;
		int androidFormat;
	};

	enum class State {
		Stopped,
		Running,
	};

	void stop();

	int initializeStreamConfigurations();
	std::vector<libcamera::Size>
	getYUVResolutions(libcamera::CameraConfiguration *cameraConfig,
			  const libcamera::PixelFormat &pixelFormat,
			  const std::vector<libcamera::Size> &resolutions);
	std::vector<libcamera::Size>
	getRawResolutions(const libcamera::PixelFormat &pixelFormat);

	libcamera::FrameBuffer *createFrameBuffer(const buffer_handle_t camera3buffer);
	void notifyShutter(uint32_t frameNumber, uint64_t timestamp);
	void notifyError(uint32_t frameNumber, camera3_stream_t *stream,
			 camera3_error_msg_code code);
	std::unique_ptr<CameraMetadata> requestTemplatePreview();
	std::unique_ptr<CameraMetadata> requestTemplateVideo();
	libcamera::PixelFormat toPixelFormat(int format) const;
	int processControls(Camera3RequestDescriptor *descriptor);
	std::unique_ptr<CameraMetadata> getResultMetadata(
		const Camera3RequestDescriptor &descriptor) const;

	unsigned int id_;
	camera3_device_t camera3Device_;

	CameraWorker worker_;

	libcamera::Mutex stateMutex_; /* Protects access to the camera state. */
	State state_;

	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;

	std::unique_ptr<CameraMetadata> staticMetadata_;
	std::map<unsigned int, std::unique_ptr<CameraMetadata>> requestTemplates_;
	const camera3_callback_ops_t *callbacks_;

	std::vector<Camera3StreamConfiguration> streamConfigurations_;
	std::map<int, libcamera::PixelFormat> formatsMap_;
	std::vector<CameraStream> streams_;

	libcamera::Mutex descriptorsMutex_; /* Protects descriptors_. */
	std::map<uint64_t, Camera3RequestDescriptor> descriptors_;

	std::string maker_;
	std::string model_;

	int facing_;
	int orientation_;

	unsigned int maxJpegBufferSize_;

	CameraMetadata lastSettings_;
};

#endif /* __ANDROID_CAMERA_DEVICE_H__ */
