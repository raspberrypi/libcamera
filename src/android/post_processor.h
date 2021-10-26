/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * post_processor.h - CameraStream Post Processing Interface
 */
#ifndef __ANDROID_POST_PROCESSOR_H__
#define __ANDROID_POST_PROCESSOR_H__

#include <libcamera/base/signal.h>

#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>

#include "camera_buffer.h"
#include "camera_request.h"

class PostProcessor
{
public:
	enum class Status {
		Error,
		Success
	};

	virtual ~PostProcessor() = default;

	virtual int configure(const libcamera::StreamConfiguration &inCfg,
			      const libcamera::StreamConfiguration &outCfg) = 0;
	virtual int process(Camera3RequestDescriptor::StreamBuffer *streamBuffer) = 0;

	libcamera::Signal<Camera3RequestDescriptor::StreamBuffer *, Status> processComplete;
};

#endif /* __ANDROID_POST_PROCESSOR_H__ */
