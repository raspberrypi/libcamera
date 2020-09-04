/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.h - Camera HAL stream
 */
#ifndef __ANDROID_CAMERA_STREAM_H__
#define __ANDROID_CAMERA_STREAM_H__

#include <memory>

#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

class Encoder;

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
	CameraStream(libcamera::PixelFormat format, libcamera::Size size,
		     Type type, unsigned int index, Encoder *encoder = nullptr);

	const libcamera::PixelFormat &format() const { return format_; }
	const libcamera::Size &size() const { return size_; }
	Type type() const { return type_; }
	unsigned int index() const { return index_; }
	Encoder *encoder() const { return encoder_.get(); }

private:
	libcamera::PixelFormat format_;
	libcamera::Size size_;
	Type type_;
	/*
	 * The index of the libcamera StreamConfiguration as added during
	 * configureStreams(). A single libcamera Stream may be used to deliver
	 * one or more streams to the Android framework.
	 */
	unsigned int index_;
	std::unique_ptr<Encoder> encoder_;
};

#endif /* __ANDROID_CAMERA_STREAM__ */
