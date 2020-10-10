/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.h - Camera HAL stream
 */
#ifndef __ANDROID_CAMERA_STREAM_H__
#define __ANDROID_CAMERA_STREAM_H__

#include <memory>
#include <mutex>
#include <vector>

#include <hardware/camera3.h>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

class Encoder;
class CameraDevice;
class CameraMetadata;
class MappedCamera3Buffer;

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
	CameraStream(CameraDevice *cameraDevice, Type type,
		     camera3_stream_t *camera3Stream, unsigned int index);

	Type type() const { return type_; }
	const camera3_stream_t &camera3Stream() const { return *camera3Stream_; }
	const libcamera::StreamConfiguration &configuration() const;
	libcamera::Stream *stream() const;

	int configure();
	int process(const libcamera::FrameBuffer &source,
		    MappedCamera3Buffer *dest, CameraMetadata *metadata);
	libcamera::FrameBuffer *getBuffer();
	void putBuffer(libcamera::FrameBuffer *buffer);

private:
	CameraDevice *cameraDevice_;
	libcamera::CameraConfiguration *config_;
	Type type_;
	camera3_stream_t *camera3Stream_;
	unsigned int index_;

	std::unique_ptr<Encoder> encoder_;
	std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
	std::vector<libcamera::FrameBuffer *> buffers_;
	/*
	 * The class has to be MoveConstructible as instances are stored in
	 * an std::vector in CameraDevice.
	 */
	 std::unique_ptr<std::mutex> mutex_;
};

#endif /* __ANDROID_CAMERA_STREAM__ */
