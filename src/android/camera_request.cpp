/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2021, Google Inc.
 *
 * camera_request.cpp - libcamera Android Camera Request Descriptor
 */

#include "camera_request.h"

#include <libcamera/base/span.h>

#include "camera_buffer.h"

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
	const Span<const camera3_stream_buffer_t> buffers{
		camera3Request->output_buffers,
		camera3Request->num_output_buffers
	};

	buffers_.reserve(buffers.size());

	for (const camera3_stream_buffer_t &buffer : buffers) {
		CameraStream *stream =
			static_cast<CameraStream *>(buffer.stream->priv);

		buffers_.emplace_back(stream, buffer, this);
	}

	/* Clone the controls associated with the camera3 request. */
	settings_ = CameraMetadata(camera3Request->settings);

	/*
	 * Create the CaptureRequest, stored as a unique_ptr<> to tie its
	 * lifetime to the descriptor.
	 */
	request_ = camera->createRequest(reinterpret_cast<uint64_t>(this));
}

Camera3RequestDescriptor::~Camera3RequestDescriptor() = default;

Camera3RequestDescriptor::StreamBuffer::StreamBuffer(
	CameraStream *cameraStream, const camera3_stream_buffer_t &buffer,
	Camera3RequestDescriptor *requestDescriptor)
	: stream(cameraStream), camera3Buffer(buffer.buffer),
	  fence(buffer.acquire_fence), request(requestDescriptor)
{
}

Camera3RequestDescriptor::StreamBuffer::~StreamBuffer() = default;

Camera3RequestDescriptor::StreamBuffer::StreamBuffer(StreamBuffer &&) = default;

Camera3RequestDescriptor::StreamBuffer &
Camera3RequestDescriptor::StreamBuffer::operator=(Camera3RequestDescriptor::StreamBuffer &&) = default;
