/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * post_processor_jpeg.h - JPEG Post Processor
 */
#ifndef __ANDROID_POST_PROCESSOR_JPEG_H__
#define __ANDROID_POST_PROCESSOR_JPEG_H__

#include "../post_processor.h"

#include <libcamera/geometry.h>

#include "libcamera/internal/buffer.h"

class Encoder;
class CameraDevice;

class PostProcessorJpeg : public PostProcessor
{
public:
	PostProcessorJpeg(CameraDevice *const device);

	int configure(const libcamera::StreamConfiguration &incfg,
		      const libcamera::StreamConfiguration &outcfg) override;
	int process(const libcamera::FrameBuffer &source,
		    libcamera::Span<uint8_t> destination,
		    CameraMetadata *metadata) override;

private:
	CameraDevice *const cameraDevice_;
	std::unique_ptr<Encoder> encoder_;
	libcamera::Size streamSize_;
};

#endif /* __ANDROID_POST_PROCESSOR_JPEG_H__ */
