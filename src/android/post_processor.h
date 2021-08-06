/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * post_processor.h - CameraStream Post Processing Interface
 */
#ifndef __ANDROID_POST_PROCESSOR_H__
#define __ANDROID_POST_PROCESSOR_H__

#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>

#include "camera_buffer.h"

class CameraMetadata;

class PostProcessor
{
public:
	virtual ~PostProcessor() = default;

	virtual int configure(const libcamera::StreamConfiguration &inCfg,
			      const libcamera::StreamConfiguration &outCfg) = 0;
	virtual int process(const libcamera::FrameBuffer &source,
			    CameraBuffer *destination,
			    const CameraMetadata &requestMetadata,
			    CameraMetadata *resultMetadata) = 0;
};

#endif /* __ANDROID_POST_PROCESSOR_H__ */
