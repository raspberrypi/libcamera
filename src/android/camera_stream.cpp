/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.cpp - Camera HAL stream
 */

#include "camera_stream.h"

#include "jpeg/encoder.h"

using namespace libcamera;

CameraStream::CameraStream(PixelFormat format, Size size,
			   Type type, unsigned int index, Encoder *encoder)
	: format_(format), size_(size), type_(type), index_(index), encoder_(encoder)
{
}
