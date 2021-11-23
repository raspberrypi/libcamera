/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * kms_sink.h - KMS Sink
 */

#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "drm.h"
#include "frame_sink.h"

class KMSSink : public FrameSink
{
public:
	KMSSink(const std::string &connectorName);

	void mapBuffer(libcamera::FrameBuffer *buffer) override;

	int configure(const libcamera::CameraConfiguration &config) override;
	int start() override;
	int stop() override;

	bool processRequest(libcamera::Request *request) override;

private:
	class Request
	{
	public:
		Request(DRM::AtomicRequest *drmRequest, libcamera::Request *camRequest)
			: drmRequest_(drmRequest), camRequest_(camRequest)
		{
		}

		std::unique_ptr<DRM::AtomicRequest> drmRequest_;
		libcamera::Request *camRequest_;
	};

	int configurePipeline(const libcamera::PixelFormat &format);
	void requestComplete(DRM::AtomicRequest *request);

	DRM::Device dev_;

	const DRM::Connector *connector_;
	const DRM::Crtc *crtc_;
	const DRM::Plane *plane_;
	const DRM::Mode *mode_;

	libcamera::PixelFormat format_;
	libcamera::Size size_;
	unsigned int stride_;

	std::map<libcamera::FrameBuffer *, std::unique_ptr<DRM::FrameBuffer>> buffers_;

	std::mutex lock_;
	std::unique_ptr<Request> pending_;
	std::unique_ptr<Request> queued_;
	std::unique_ptr<Request> active_;
};
