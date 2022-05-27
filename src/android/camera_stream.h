/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.h - Camera HAL stream
 */

#pragma once

#include <memory>
#include <queue>
#include <vector>

#include <hardware/camera3.h>

#include <libcamera/base/mutex.h>
#include <libcamera/base/thread.h>

#include <libcamera/camera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "camera_request.h"
#include "post_processor.h"

class CameraDevice;
class PlatformFrameBufferAllocator;

class CameraStream
{
public:
	/*
	 * Enumeration of CameraStream types.
	 *
	 * A camera stream associates an Android stream to a libcamera stream.
	 * This enumeration describes how the two streams are associated and how
	 * and where data produced from libcamera are delivered to the
	 * Android framework.
	 *
	 * Direct:
	 *
	 * The Android stream is directly mapped onto a libcamera stream: frames
	 * are delivered by the library directly in the memory location
	 * specified by the Android stream (buffer_handle_t->data) and provided
	 * to the framework as they are. The Android stream characteristics are
	 * directly translated to the libcamera stream configuration.
	 *
	 * +-----+                +-----+
	 * |  A  |                |  L  |
	 * +-----+                +-----+
	 *    |                      |
	 *    V                      V
	 * +-----+                +------+
	 * |  B  |<---------------|  FB  |
	 * +-----+                +------+
	 *
	 *
	 * Internal:
	 *
	 * Data for the Android stream is produced by processing a libcamera
	 * stream created by the HAL for that purpose. The libcamera stream
	 * needs to be supplied with intermediate buffers where the library
	 * delivers frames to be processed and then provided to the framework.
	 * The libcamera stream configuration is not a direct translation of the
	 * Android stream characteristics, but it describes the format and size
	 * required for the processing procedure to produce frames in the
	 * Android required format.
	 *
	 * +-----+                +-----+
	 * |  A  |                |  L  |
	 * +-----+                +-----+
	 *    |                      |
	 *    V                      V
	 * +-----+                +------+
	 * |  B  |                |  FB  |
	 * +-----+                +------+
	 *   ^                       |
	 *   |-------Processing------|
	 *
	 *
	 * Mapped:
	 *
	 * Data for the Android stream is produced by processing a libcamera
	 * stream associated with another CameraStream. Mapped camera streams do
	 * not need any memory to be reserved for them as they process data
	 * produced by libcamera for a different stream whose format and size
	 * are compatible with the processing procedure requirements to produce
	 * frames in the Android required format.
	 *
	 * +-----+      +-----+          +-----+
	 * |  A  |      |  A' |          |  L  |
	 * +-----+      +-----+          +-----+
	 *    |            |                |
	 *    V            V                V
	 * +-----+      +-----+          +------+
	 * |  B  |      |  B' |<---------|  FB  |
	 * +-----+      +-----+          +------+
	 *   ^              |
	 *   |--Processing--|
	 *
	 *
	 * --------------------------------------------------------------------
	 * A  = Android stream
	 * L  = libcamera stream
	 * B  = memory buffer
	 * FB = libcamera FrameBuffer
	 * "Processing" = Frame processing procedure (Encoding, scaling etc)
	 */
	enum class Type {
		Direct,
		Internal,
		Mapped,
	};
	CameraStream(CameraDevice *const cameraDevice,
		     libcamera::CameraConfiguration *config, Type type,
		     camera3_stream_t *camera3Stream,
		     CameraStream *const sourceStream,
		     unsigned int index);
	CameraStream(CameraStream &&other);
	~CameraStream();

	Type type() const { return type_; }
	camera3_stream_t *camera3Stream() const { return camera3Stream_; }
	const libcamera::StreamConfiguration &configuration() const;
	libcamera::Stream *stream() const;
	CameraStream *sourceStream() const { return sourceStream_; }

	int configure();
	int process(Camera3RequestDescriptor::StreamBuffer *streamBuffer);
	libcamera::FrameBuffer *getBuffer();
	void putBuffer(libcamera::FrameBuffer *buffer);
	void flush();

private:
	class PostProcessorWorker : public libcamera::Thread
	{
	public:
		enum class State {
			Stopped,
			Running,
			Flushing,
		};

		PostProcessorWorker(PostProcessor *postProcessor);
		~PostProcessorWorker();

		void start();
		void queueRequest(Camera3RequestDescriptor::StreamBuffer *request);
		void flush();

	protected:
		void run() override;

	private:
		PostProcessor *postProcessor_;

		libcamera::Mutex mutex_;
		libcamera::ConditionVariable cv_;

		std::queue<Camera3RequestDescriptor::StreamBuffer *> requests_
			LIBCAMERA_TSA_GUARDED_BY(mutex_);

		State state_ LIBCAMERA_TSA_GUARDED_BY(mutex_) = State::Stopped;
	};

	int waitFence(int fence);

	CameraDevice *const cameraDevice_;
	const libcamera::CameraConfiguration *config_;
	const Type type_;
	camera3_stream_t *camera3Stream_;
	CameraStream *const sourceStream_;
	const unsigned int index_;

	std::unique_ptr<PlatformFrameBufferAllocator> allocator_;
	std::vector<std::unique_ptr<libcamera::FrameBuffer>> allocatedBuffers_;
	std::vector<libcamera::FrameBuffer *> buffers_ LIBCAMERA_TSA_GUARDED_BY(mutex_);
	/*
	 * The class has to be MoveConstructible as instances are stored in
	 * an std::vector in CameraDevice.
	 */
	std::unique_ptr<libcamera::Mutex> mutex_;
	std::unique_ptr<PostProcessor> postProcessor_;

	std::unique_ptr<PostProcessorWorker> worker_;
};
