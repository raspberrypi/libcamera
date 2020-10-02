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
	CameraStream(libcamera::PixelFormat format, libcamera::Size size,
		     unsigned int index, Encoder *encoder = nullptr);

	const libcamera::PixelFormat &format() const { return format_; }
	const libcamera::Size &size() const { return size_; }
	unsigned int index() const { return index_; }
	Encoder *encoder() const { return encoder_.get(); }

private:
	libcamera::PixelFormat format_;
	libcamera::Size size_;
	/*
	 * The index of the libcamera StreamConfiguration as added during
	 * configureStreams(). A single libcamera Stream may be used to deliver
	 * one or more streams to the Android framework.
	 */
	unsigned int index_;
	std::unique_ptr<Encoder> encoder_;
};

#endif /* __ANDROID_CAMERA_STREAM__ */
