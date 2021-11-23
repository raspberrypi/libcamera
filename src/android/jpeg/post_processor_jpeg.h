/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * post_processor_jpeg.h - JPEG Post Processor
 */

#pragma once

#include "../post_processor.h"
#include "encoder_libjpeg.h"
#include "thumbnailer.h"

#include <libcamera/geometry.h>

class CameraDevice;

class PostProcessorJpeg : public PostProcessor
{
public:
	PostProcessorJpeg(CameraDevice *const device);

	int configure(const libcamera::StreamConfiguration &incfg,
		      const libcamera::StreamConfiguration &outcfg) override;
	void process(Camera3RequestDescriptor::StreamBuffer *streamBuffer) override;

private:
	void generateThumbnail(const libcamera::FrameBuffer &source,
			       const libcamera::Size &targetSize,
			       unsigned int quality,
			       std::vector<unsigned char> *thumbnail);

	CameraDevice *const cameraDevice_;
	std::unique_ptr<Encoder> encoder_;
	libcamera::Size streamSize_;
	EncoderLibJpeg thumbnailEncoder_;
	Thumbnailer thumbnailer_;
};
