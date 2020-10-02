/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.cpp - Camera HAL stream
 */

#include "camera_stream.h"

#include "jpeg/encoder.h"
#include "jpeg/encoder_libjpeg.h"

using namespace libcamera;

CameraStream::CameraStream(PixelFormat format, Size size, Type type, unsigned int index)
	: format_(format), size_(size), type_(type), index_(index)
{
	if (type_ == Type::Internal || type_ == Type::Mapped)
		encoder_ = std::make_unique<EncoderLibJpeg>();
}

int CameraStream::configure(const libcamera::StreamConfiguration &cfg)
{
	if (encoder_)
		return encoder_->configure(cfg);

	return 0;
}
