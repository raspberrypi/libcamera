/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * capture.h - Cam capture
 */
#ifndef __CAM_CAPTURE_H__
#define __CAM_CAPTURE_H__

#include <memory>
#include <stdint.h>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "buffer_writer.h"
#include "event_loop.h"
#include "options.h"

class Capture
{
public:
	Capture(std::shared_ptr<libcamera::Camera> camera,
		libcamera::CameraConfiguration *config,
		EventLoop *loop);

	int run(const OptionsParser::Options &options);
private:
	int capture(libcamera::FrameBufferAllocator *allocator);

	int queueRequest(libcamera::Request *request);
	void requestComplete(libcamera::Request *request);
	void processRequest(libcamera::Request *request);

	std::shared_ptr<libcamera::Camera> camera_;
	libcamera::CameraConfiguration *config_;

	std::map<const libcamera::Stream *, std::string> streamName_;
	BufferWriter *writer_;
	uint64_t last_;

	EventLoop *loop_;
	unsigned int queueCount_;
	unsigned int captureCount_;
	unsigned int captureLimit_;
	bool printMetadata_;

	std::vector<std::unique_ptr<libcamera::Request>> requests_;
};

#endif /* __CAM_CAPTURE_H__ */
