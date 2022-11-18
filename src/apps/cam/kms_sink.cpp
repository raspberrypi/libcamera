/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * kms_sink.cpp - KMS Sink
 */

#include "kms_sink.h"

#include <array>
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <limits.h>
#include <memory>
#include <stdint.h>
#include <string.h>

#include <libcamera/camera.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>

#include "drm.h"

KMSSink::KMSSink(const std::string &connectorName)
	: connector_(nullptr), crtc_(nullptr), plane_(nullptr), mode_(nullptr)
{
	int ret = dev_.init();
	if (ret < 0)
		return;

	/*
	 * Find the requested connector. If no specific connector is requested,
	 * pick the first connected connector or, if no connector is connected,
	 * the first connector with unknown status.
	 */
	for (const DRM::Connector &conn : dev_.connectors()) {
		if (!connectorName.empty()) {
			if (conn.name() != connectorName)
				continue;

			connector_ = &conn;
			break;
		}

		if (conn.status() == DRM::Connector::Connected) {
			connector_ = &conn;
			break;
		}

		if (!connector_ && conn.status() == DRM::Connector::Unknown)
			connector_ = &conn;
	}

	if (!connector_) {
		if (!connectorName.empty())
			std::cerr
				<< "Connector " << connectorName << " not found"
				<< std::endl;
		else
			std::cerr << "No connected connector found" << std::endl;
		return;
	}

	dev_.requestComplete.connect(this, &KMSSink::requestComplete);
}

void KMSSink::mapBuffer(libcamera::FrameBuffer *buffer)
{
	std::array<uint32_t, 4> strides = {};

	/* \todo Should libcamera report per-plane strides ? */
	unsigned int uvStrideMultiplier;

	switch (format_) {
	case libcamera::formats::NV24:
	case libcamera::formats::NV42:
		uvStrideMultiplier = 4;
		break;
	case libcamera::formats::YUV420:
	case libcamera::formats::YVU420:
	case libcamera::formats::YUV422:
		uvStrideMultiplier = 1;
		break;
	default:
		uvStrideMultiplier = 2;
		break;
	}

	strides[0] = stride_;
	for (unsigned int i = 1; i < buffer->planes().size(); ++i)
		strides[i] = stride_ * uvStrideMultiplier / 2;

	std::unique_ptr<DRM::FrameBuffer> drmBuffer =
		dev_.createFrameBuffer(*buffer, format_, size_, strides);
	if (!drmBuffer)
		return;

	buffers_.emplace(std::piecewise_construct,
			 std::forward_as_tuple(buffer),
			 std::forward_as_tuple(std::move(drmBuffer)));
}

int KMSSink::configure(const libcamera::CameraConfiguration &config)
{
	if (!connector_)
		return -EINVAL;

	crtc_ = nullptr;
	plane_ = nullptr;
	mode_ = nullptr;

	const libcamera::StreamConfiguration &cfg = config.at(0);

	/* Find the best mode for the stream size. */
	const std::vector<DRM::Mode> &modes = connector_->modes();

	unsigned int cfgArea = cfg.size.width * cfg.size.height;
	unsigned int bestDistance = UINT_MAX;

	for (const DRM::Mode &mode : modes) {
		unsigned int modeArea = mode.hdisplay * mode.vdisplay;
		unsigned int distance = modeArea > cfgArea ? modeArea - cfgArea
				      : cfgArea - modeArea;

		if (distance < bestDistance) {
			mode_ = &mode;
			bestDistance = distance;

			/*
			 * If the sizes match exactly, there will be no better
			 * match.
			 */
			if (distance == 0)
				break;
		}
	}

	if (!mode_) {
		std::cerr << "No modes\n";
		return -EINVAL;
	}

	int ret = configurePipeline(cfg.pixelFormat);
	if (ret < 0)
		return ret;

	size_ = cfg.size;
	stride_ = cfg.stride;

	/* Configure color space. */
	colorEncoding_ = std::nullopt;
	colorRange_ = std::nullopt;

	if (cfg.colorSpace->ycbcrEncoding == libcamera::ColorSpace::YcbcrEncoding::None)
		return 0;

	/*
	 * The encoding and range enums are defined in the kernel but not
	 * exposed in public headers.
	 */
	enum drm_color_encoding {
		DRM_COLOR_YCBCR_BT601,
		DRM_COLOR_YCBCR_BT709,
		DRM_COLOR_YCBCR_BT2020,
	};

	enum drm_color_range {
		DRM_COLOR_YCBCR_LIMITED_RANGE,
		DRM_COLOR_YCBCR_FULL_RANGE,
	};

	const DRM::Property *colorEncoding = plane_->property("COLOR_ENCODING");
	const DRM::Property *colorRange = plane_->property("COLOR_RANGE");

	if (colorEncoding) {
		drm_color_encoding encoding;

		switch (cfg.colorSpace->ycbcrEncoding) {
		case libcamera::ColorSpace::YcbcrEncoding::Rec601:
		default:
			encoding = DRM_COLOR_YCBCR_BT601;
			break;
		case libcamera::ColorSpace::YcbcrEncoding::Rec709:
			encoding = DRM_COLOR_YCBCR_BT709;
			break;
		case libcamera::ColorSpace::YcbcrEncoding::Rec2020:
			encoding = DRM_COLOR_YCBCR_BT2020;
			break;
		}

		for (const auto &[id, name] : colorEncoding->enums()) {
			if (id == encoding) {
				colorEncoding_ = encoding;
				break;
			}
		}
	}

	if (colorRange) {
		drm_color_range range;

		switch (cfg.colorSpace->range) {
		case libcamera::ColorSpace::Range::Limited:
		default:
			range = DRM_COLOR_YCBCR_LIMITED_RANGE;
			break;
		case libcamera::ColorSpace::Range::Full:
			range = DRM_COLOR_YCBCR_FULL_RANGE;
			break;
		}

		for (const auto &[id, name] : colorRange->enums()) {
			if (id == range) {
				colorRange_ = range;
				break;
			}
		}
	}

	if (!colorEncoding_ || !colorRange_)
		std::cerr << "Color space " << cfg.colorSpace->toString()
			  << " not supported by the display device."
			  << " Colors may be wrong." << std::endl;

	return 0;
}

int KMSSink::selectPipeline(const libcamera::PixelFormat &format)
{
	/*
	 * If the requested format has an alpha channel, also consider the X
	 * variant.
	 */
	libcamera::PixelFormat xFormat;

	switch (format) {
	case libcamera::formats::ABGR8888:
		xFormat = libcamera::formats::XBGR8888;
		break;
	case libcamera::formats::ARGB8888:
		xFormat = libcamera::formats::XRGB8888;
		break;
	case libcamera::formats::BGRA8888:
		xFormat = libcamera::formats::BGRX8888;
		break;
	case libcamera::formats::RGBA8888:
		xFormat = libcamera::formats::RGBX8888;
		break;
	}

	/*
	 * Find a CRTC and plane suitable for the request format and the
	 * connector at the end of the pipeline. Restrict the search to primary
	 * planes for now.
	 */
	for (const DRM::Encoder *encoder : connector_->encoders()) {
		for (const DRM::Crtc *crtc : encoder->possibleCrtcs()) {
			for (const DRM::Plane *plane : crtc->planes()) {
				if (plane->type() != DRM::Plane::TypePrimary)
					continue;

				if (plane->supportsFormat(format)) {
					crtc_ = crtc;
					plane_ = plane;
					format_ = format;
					return 0;
				}

				if (plane->supportsFormat(xFormat)) {
					crtc_ = crtc;
					plane_ = plane;
					format_ = xFormat;
					return 0;
				}
			}
		}
	}

	return -EPIPE;
}

int KMSSink::configurePipeline(const libcamera::PixelFormat &format)
{
	const int ret = selectPipeline(format);
	if (ret) {
		std::cerr
			<< "Unable to find display pipeline for format "
			<< format << std::endl;

		return ret;
	}

	std::cout
		<< "Using KMS plane " << plane_->id() << ", CRTC " << crtc_->id()
		<< ", connector " << connector_->name()
		<< " (" << connector_->id() << "), mode " << mode_->hdisplay
		<< "x" << mode_->vdisplay << "@" << mode_->vrefresh << std::endl;

	return 0;
}

int KMSSink::start()
{
	std::unique_ptr<DRM::AtomicRequest> request;

	int ret = FrameSink::start();
	if (ret < 0)
		return ret;

	/* Disable all CRTCs and planes to start from a known valid state. */
	request = std::make_unique<DRM::AtomicRequest>(&dev_);

	for (const DRM::Crtc &crtc : dev_.crtcs())
		request->addProperty(&crtc, "ACTIVE", 0);

	for (const DRM::Plane &plane : dev_.planes()) {
		request->addProperty(&plane, "CRTC_ID", 0);
		request->addProperty(&plane, "FB_ID", 0);
	}

	ret = request->commit(DRM::AtomicRequest::FlagAllowModeset);
	if (ret < 0) {
		std::cerr
			<< "Failed to disable CRTCs and planes: "
			<< strerror(-ret) << std::endl;
		return ret;
	}

	return 0;
}

int KMSSink::stop()
{
	/* Display pipeline. */
	DRM::AtomicRequest request(&dev_);

	request.addProperty(connector_, "CRTC_ID", 0);
	request.addProperty(crtc_, "ACTIVE", 0);
	request.addProperty(crtc_, "MODE_ID", 0);
	request.addProperty(plane_, "CRTC_ID", 0);
	request.addProperty(plane_, "FB_ID", 0);

	int ret = request.commit(DRM::AtomicRequest::FlagAllowModeset);
	if (ret < 0) {
		std::cerr
			<< "Failed to stop display pipeline: "
			<< strerror(-ret) << std::endl;
		return ret;
	}

	/* Free all buffers. */
	pending_.reset();
	queued_.reset();
	active_.reset();
	buffers_.clear();

	return FrameSink::stop();
}

bool KMSSink::testModeSet(DRM::FrameBuffer *drmBuffer,
			  const libcamera::Rectangle &src,
			  const libcamera::Rectangle &dst)
{
	DRM::AtomicRequest drmRequest{ &dev_ };

	drmRequest.addProperty(connector_, "CRTC_ID", crtc_->id());

	drmRequest.addProperty(crtc_, "ACTIVE", 1);
	drmRequest.addProperty(crtc_, "MODE_ID", mode_->toBlob(&dev_));

	drmRequest.addProperty(plane_, "CRTC_ID", crtc_->id());
	drmRequest.addProperty(plane_, "FB_ID", drmBuffer->id());
	drmRequest.addProperty(plane_, "SRC_X", src.x << 16);
	drmRequest.addProperty(plane_, "SRC_Y", src.y << 16);
	drmRequest.addProperty(plane_, "SRC_W", src.width << 16);
	drmRequest.addProperty(plane_, "SRC_H", src.height << 16);
	drmRequest.addProperty(plane_, "CRTC_X", dst.x);
	drmRequest.addProperty(plane_, "CRTC_Y", dst.y);
	drmRequest.addProperty(plane_, "CRTC_W", dst.width);
	drmRequest.addProperty(plane_, "CRTC_H", dst.height);

	return !drmRequest.commit(DRM::AtomicRequest::FlagAllowModeset |
				  DRM::AtomicRequest::FlagTestOnly);
}

bool KMSSink::setupComposition(DRM::FrameBuffer *drmBuffer)
{
	/*
	 * Test composition options, from most to least desirable, to select the
	 * best one.
	 */
	const libcamera::Rectangle framebuffer{ size_ };
	const libcamera::Rectangle display{ 0, 0, mode_->hdisplay, mode_->vdisplay };

	/* 1. Scale the frame buffer to full screen, preserving aspect ratio. */
	libcamera::Rectangle src = framebuffer;
	libcamera::Rectangle dst = display.size().boundedToAspectRatio(framebuffer.size())
						 .centeredTo(display.center());

	if (testModeSet(drmBuffer, src, dst)) {
		std::cout << "KMS: full-screen scaled output, square pixels"
			  << std::endl;
		src_ = src;
		dst_ = dst;
		return true;
	}

	/*
	 * 2. Scale the frame buffer to full screen, without preserving aspect
	 *    ratio.
	 */
	src = framebuffer;
	dst = display;

	if (testModeSet(drmBuffer, src, dst)) {
		std::cout << "KMS: full-screen scaled output, non-square pixels"
			  << std::endl;
		src_ = src;
		dst_ = dst;
		return true;
	}

	/* 3. Center the frame buffer on the display. */
	src = display.size().centeredTo(framebuffer.center()).boundedTo(framebuffer);
	dst = framebuffer.size().centeredTo(display.center()).boundedTo(display);

	if (testModeSet(drmBuffer, src, dst)) {
		std::cout << "KMS: centered output" << std::endl;
		src_ = src;
		dst_ = dst;
		return true;
	}

	/* 4. Align the frame buffer on the top-left of the display. */
	src = framebuffer.boundedTo(display);
	dst = display.boundedTo(framebuffer);

	if (testModeSet(drmBuffer, src, dst)) {
		std::cout << "KMS: top-left aligned output" << std::endl;
		src_ = src;
		dst_ = dst;
		return true;
	}

	return false;
}

bool KMSSink::processRequest(libcamera::Request *camRequest)
{
	/*
	 * Perform a very crude rate adaptation by simply dropping the request
	 * if the display queue is full.
	 */
	if (pending_)
		return true;

	libcamera::FrameBuffer *buffer = camRequest->buffers().begin()->second;
	auto iter = buffers_.find(buffer);
	if (iter == buffers_.end())
		return true;

	DRM::FrameBuffer *drmBuffer = iter->second.get();

	unsigned int flags = DRM::AtomicRequest::FlagAsync;
	std::unique_ptr<DRM::AtomicRequest> drmRequest =
		std::make_unique<DRM::AtomicRequest>(&dev_);
	drmRequest->addProperty(plane_, "FB_ID", drmBuffer->id());

	if (!active_ && !queued_) {
		/* Enable the display pipeline on the first frame. */
		if (!setupComposition(drmBuffer)) {
			std::cerr << "Failed to setup composition" << std::endl;
			return true;
		}

		drmRequest->addProperty(connector_, "CRTC_ID", crtc_->id());

		drmRequest->addProperty(crtc_, "ACTIVE", 1);
		drmRequest->addProperty(crtc_, "MODE_ID", mode_->toBlob(&dev_));

		drmRequest->addProperty(plane_, "CRTC_ID", crtc_->id());
		drmRequest->addProperty(plane_, "SRC_X", src_.x << 16);
		drmRequest->addProperty(plane_, "SRC_Y", src_.y << 16);
		drmRequest->addProperty(plane_, "SRC_W", src_.width << 16);
		drmRequest->addProperty(plane_, "SRC_H", src_.height << 16);
		drmRequest->addProperty(plane_, "CRTC_X", dst_.x);
		drmRequest->addProperty(plane_, "CRTC_Y", dst_.y);
		drmRequest->addProperty(plane_, "CRTC_W", dst_.width);
		drmRequest->addProperty(plane_, "CRTC_H", dst_.height);

		if (colorEncoding_)
			drmRequest->addProperty(plane_, "COLOR_ENCODING", *colorEncoding_);
		if (colorRange_)
			drmRequest->addProperty(plane_, "COLOR_RANGE", *colorRange_);

		flags |= DRM::AtomicRequest::FlagAllowModeset;
	}

	pending_ = std::make_unique<Request>(std::move(drmRequest), camRequest);

	std::lock_guard<std::mutex> lock(lock_);

	if (!queued_) {
		int ret = pending_->drmRequest_->commit(flags);
		if (ret < 0) {
			std::cerr
				<< "Failed to commit atomic request: "
				<< strerror(-ret) << std::endl;
			/* \todo Implement error handling */
		}

		queued_ = std::move(pending_);
	}

	return false;
}

void KMSSink::requestComplete([[maybe_unused]] DRM::AtomicRequest *request)
{
	std::lock_guard<std::mutex> lock(lock_);

	assert(queued_ && queued_->drmRequest_.get() == request);

	/* Complete the active request, if any. */
	if (active_)
		requestProcessed.emit(active_->camRequest_);

	/* The queued request becomes active. */
	active_ = std::move(queued_);

	/* Queue the pending request, if any. */
	if (pending_) {
		pending_->drmRequest_->commit(DRM::AtomicRequest::FlagAsync);
		queued_ = std::move(pending_);
	}
}
