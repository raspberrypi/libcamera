/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_device.h - libcamera Android Camera Device
 */
#ifndef __ANDROID_CAMERA_DEVICE_H__
#define __ANDROID_CAMERA_DEVICE_H__

#include <memory>

#include <hardware/camera3.h>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/message.h"

class CameraMetadata;

class CameraDevice
{
public:
	CameraDevice(unsigned int id, const std::shared_ptr<libcamera::Camera> &camera);
	~CameraDevice();

	int open(const hw_module_t *hardwareModule);
	void close();

	unsigned int id() const { return id_; }
	camera3_device_t *camera3Device() { return &camera3Device_; }

	void setCallbacks(const camera3_callback_ops_t *callbacks);
	const camera_metadata_t *getStaticMetadata();
	const camera_metadata_t *constructDefaultRequestSettings(int type);
	int configureStreams(camera3_stream_configuration_t *stream_list);
	int processCaptureRequest(camera3_capture_request_t *request);
	void requestComplete(libcamera::Request *request);

private:
	struct Camera3RequestDescriptor {
		Camera3RequestDescriptor(unsigned int frameNumber,
					 unsigned int numBuffers);
		~Camera3RequestDescriptor();

		uint32_t frameNumber;
		uint32_t numBuffers;
		camera3_stream_buffer_t *buffers;
	};

	void notifyShutter(uint32_t frameNumber, uint64_t timestamp);
	void notifyError(uint32_t frameNumber, camera3_stream_t *stream);
	std::unique_ptr<CameraMetadata> getResultMetadata(int frame_number,
							  int64_t timestamp);

	unsigned int id_;
	camera3_device_t camera3Device_;

	bool running_;
	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;

	CameraMetadata *staticMetadata_;
	std::map<unsigned int, CameraMetadata *> requestTemplates_;
	const camera3_callback_ops_t *callbacks_;
};

#endif /* __ANDROID_CAMERA_DEVICE_H__ */
