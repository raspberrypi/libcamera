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
#include <libcamera/object.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "message.h"

#define METADATA_ASSERT(_r)		\
	do {				\
		if (!(_r)) break;	\
		LOG(HAL, Error) << "Error: " << __func__ << ":" << __LINE__; \
		return nullptr;		\
	} while(0);

class CameraDevice : public libcamera::Object
{
public:
	CameraDevice(unsigned int id, std::shared_ptr<libcamera::Camera> &camera);
	~CameraDevice();

	void message(libcamera::Message *message);

	int open();
	void close();
	void setCallbacks(const camera3_callback_ops_t *callbacks);
	camera_metadata_t *getStaticMetadata();
	const camera_metadata_t *constructDefaultRequestSettings(int type);
	int configureStreams(camera3_stream_configuration_t *stream_list);
	int processCaptureRequest(camera3_capture_request_t *request);
	void requestComplete(libcamera::Request *request,
			     const std::map<libcamera::Stream *, libcamera::Buffer *> &buffers);

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
	camera_metadata_t *getResultMetadata(int frame_number, int64_t timestamp);

	bool running_;
	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;

	camera_metadata_t *staticMetadata_;
	camera_metadata_t *requestTemplate_;
	const camera3_callback_ops_t *callbacks_;
};

#endif /* __ANDROID_CAMERA_DEVICE_H__ */
