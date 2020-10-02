/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.cpp - Camera HAL stream
 */

#include "camera_stream.h"

#include "camera_device.h"
#include "jpeg/encoder.h"
#include "jpeg/encoder_libjpeg.h"

using namespace libcamera;

CameraStream::CameraStream(CameraDevice *cameraDevice,
			   camera3_stream_t *camera3Stream,
			   const libcamera::StreamConfiguration &cfg,
			   Type type, unsigned int index)
	: cameraDevice_(cameraDevice), camera3Stream_(camera3Stream),
	  type_(type), index_(index)
{
	config_ = cameraDevice_->cameraConfiguration();

	format_ = cfg.pixelFormat;
	size_ = cfg.size;

	if (type_ == Type::Internal || type_ == Type::Mapped)
		encoder_ = std::make_unique<EncoderLibJpeg>();
}

int CameraStream::configure(const libcamera::StreamConfiguration &cfg)
{
	if (encoder_)
		return encoder_->configure(cfg);

	return 0;
}
