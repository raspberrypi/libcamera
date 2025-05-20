/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020-2021, Google Inc.
 *
 * Simple capture helper
 */

#pragma once

#include <memory>
#include <optional>

#include <libcamera/libcamera.h>

#include "../common/event_loop.h"

class Capture
{
public:
	Capture(std::shared_ptr<libcamera::Camera> camera);
	~Capture();

	void configure(libcamera::Span<const libcamera::StreamRole> roles);
	void run(unsigned int captureLimit, std::optional<unsigned int> queueLimit = {});

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(Capture)

	void start();
	void stop();

	int queueRequest(libcamera::Request *request);
	void requestComplete(libcamera::Request *request);

	std::shared_ptr<libcamera::Camera> camera_;
	libcamera::FrameBufferAllocator allocator_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;
	std::vector<std::unique_ptr<libcamera::Request>> requests_;

	EventLoop *loop_ = nullptr;
	unsigned int captureLimit_ = 0;
	std::optional<unsigned int> queueLimit_;
	unsigned int captureCount_ = 0;
	unsigned int queueCount_ = 0;
};
