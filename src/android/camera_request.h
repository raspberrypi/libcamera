/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2021, Google Inc.
 *
 * camera_request.h - libcamera Android Camera Request Descriptor
 */
#ifndef __ANDROID_CAMERA_REQUEST_H__
#define __ANDROID_CAMERA_REQUEST_H__

#include <memory>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/framebuffer.h>

#include <hardware/camera3.h>

#include "camera_metadata.h"
#include "camera_worker.h"

struct Camera3RequestDescriptor {
	enum class Status {
		Pending,
		Success,
		Error,
	};

	Camera3RequestDescriptor() = default;
	~Camera3RequestDescriptor() = default;
	Camera3RequestDescriptor(libcamera::Camera *camera,
				 const camera3_capture_request_t *camera3Request);
	Camera3RequestDescriptor &operator=(Camera3RequestDescriptor &&) = default;

	bool isPending() const { return status_ == Status::Pending; }

	uint32_t frameNumber_ = 0;
	std::vector<camera3_stream_buffer_t> buffers_;
	std::vector<std::unique_ptr<libcamera::FrameBuffer>> frameBuffers_;
	CameraMetadata settings_;
	std::unique_ptr<CaptureRequest> request_;

	camera3_capture_result_t captureResult_ = {};
	Status status_ = Status::Pending;
};

#endif /* __ANDROID_CAMERA_REQUEST_H__ */
