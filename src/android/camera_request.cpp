/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2021, Google Inc.
 *
 * camera_request.cpp - libcamera Android Camera Request Descriptor
 */

#include "camera_request.h"

using namespace libcamera;

/*
 * \class Camera3RequestDescriptor
 *
 * A utility class that groups information about a capture request to be later
 * reused at request complete time to notify the framework.
 */

Camera3RequestDescriptor::Camera3RequestDescriptor(
	Camera *camera, const camera3_capture_request_t *camera3Request)
{
	frameNumber_ = camera3Request->frame_number;

	/* Copy the camera3 request stream information for later access. */
	const uint32_t numBuffers = camera3Request->num_output_buffers;

	buffers_.resize(numBuffers);
	for (uint32_t i = 0; i < numBuffers; i++)
		buffers_[i].buffer = camera3Request->output_buffers[i];

	/* Clone the controls associated with the camera3 request. */
	settings_ = CameraMetadata(camera3Request->settings);

	/*
	 * Create the CaptureRequest, stored as a unique_ptr<> to tie its
	 * lifetime to the descriptor.
	 */
	request_ = std::make_unique<CaptureRequest>(camera,
						    reinterpret_cast<uint64_t>(this));
}

Camera3RequestDescriptor::~Camera3RequestDescriptor() = default;
