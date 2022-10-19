/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_session.h - Camera capture session
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

#include <libcamera/base/signal.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "../common/options.h"

class CaptureScript;
class FrameSink;

class CameraSession
{
public:
	CameraSession(libcamera::CameraManager *cm,
		      const std::string &cameraId, unsigned int cameraIndex,
		      const OptionsParser::Options &options);
	~CameraSession();

	bool isValid() const { return config_ != nullptr; }
	const OptionsParser::Options &options() { return options_; }

	libcamera::Camera *camera() { return camera_.get(); }
	libcamera::CameraConfiguration *config() { return config_.get(); }

	void listControls() const;
	void listProperties() const;
	void infoConfiguration() const;

	int start();
	void stop();

	libcamera::Signal<> captureDone;

private:
	int startCapture();

	int queueRequest(libcamera::Request *request);
	void requestComplete(libcamera::Request *request);
	void processRequest(libcamera::Request *request);
	void sinkRelease(libcamera::Request *request);

	const OptionsParser::Options &options_;
	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;

	std::unique_ptr<CaptureScript> script_;

	std::map<const libcamera::Stream *, std::string> streamNames_;
	std::unique_ptr<FrameSink> sink_;
	unsigned int cameraIndex_;

	uint64_t last_;

	unsigned int queueCount_;
	unsigned int captureCount_;
	unsigned int captureLimit_;
	bool printMetadata_;

	std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
	std::vector<std::unique_ptr<libcamera::Request>> requests_;
};
