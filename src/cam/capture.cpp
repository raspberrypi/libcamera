/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * capture.cpp - Cam capture
 */

#include <iomanip>
#include <iostream>
#include <limits.h>
#include <sstream>

#include "capture.h"
#include "main.h"

using namespace libcamera;

Capture::Capture(std::shared_ptr<Camera> camera, CameraConfiguration *config,
		 EventLoop *loop)
	: camera_(camera), config_(config), writer_(nullptr), last_(0), loop_(loop),
	  captureCount_(0), captureLimit_(0)
{
}

int Capture::run(const OptionsParser::Options &options)
{
	int ret;

	captureCount_ = 0;
	captureLimit_ = options[OptCapture].toInteger();

	if (!camera_) {
		std::cout << "Can't capture without a camera" << std::endl;
		return -ENODEV;
	}

	ret = camera_->configure(config_);
	if (ret < 0) {
		std::cout << "Failed to configure camera" << std::endl;
		return ret;
	}

	streamName_.clear();
	for (unsigned int index = 0; index < config_->size(); ++index) {
		StreamConfiguration &cfg = config_->at(index);
		streamName_[cfg.stream()] = "stream" + std::to_string(index);
	}

	camera_->requestCompleted.connect(this, &Capture::requestComplete);

	if (options.isSet(OptFile)) {
		if (!options[OptFile].toString().empty())
			writer_ = new BufferWriter(options[OptFile]);
		else
			writer_ = new BufferWriter();
	}


	FrameBufferAllocator *allocator = new FrameBufferAllocator(camera_);

	ret = capture(allocator);

	if (options.isSet(OptFile)) {
		delete writer_;
		writer_ = nullptr;
	}

	requests_.clear();

	delete allocator;

	return ret;
}

int Capture::capture(FrameBufferAllocator *allocator)
{
	int ret;

	/* Identify the stream with the least number of buffers. */
	unsigned int nbuffers = UINT_MAX;
	for (StreamConfiguration &cfg : *config_) {
		ret = allocator->allocate(cfg.stream());
		if (ret < 0) {
			std::cerr << "Can't allocate buffers" << std::endl;
			return -ENOMEM;
		}

		unsigned int allocated = allocator->buffers(cfg.stream()).size();
		nbuffers = std::min(nbuffers, allocated);
	}

	/*
	 * TODO: make cam tool smarter to support still capture by for
	 * example pushing a button. For now run all streams all the time.
	 */

	for (unsigned int i = 0; i < nbuffers; i++) {
		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			return -ENOMEM;
		}

		for (StreamConfiguration &cfg : *config_) {
			Stream *stream = cfg.stream();
			const std::vector<std::unique_ptr<FrameBuffer>> &buffers =
				allocator->buffers(stream);
			const std::unique_ptr<FrameBuffer> &buffer = buffers[i];

			ret = request->addBuffer(stream, buffer.get());
			if (ret < 0) {
				std::cerr << "Can't set buffer for request"
					  << std::endl;
				return ret;
			}

			if (writer_)
				writer_->mapBuffer(buffer.get());
		}

		requests_.push_back(std::move(request));
	}

	ret = camera_->start();
	if (ret) {
		std::cout << "Failed to start capture" << std::endl;
		return ret;
	}

	for (std::unique_ptr<Request> &request : requests_) {
		ret = camera_->queueRequest(request.get());
		if (ret < 0) {
			std::cerr << "Can't queue request" << std::endl;
			camera_->stop();
			return ret;
		}
	}

	if (captureLimit_)
		std::cout << "Capture " << captureLimit_ << " frames" << std::endl;
	else
		std::cout << "Capture until user interrupts by SIGINT" << std::endl;

	ret = loop_->exec();
	if (ret)
		std::cout << "Failed to run capture loop" << std::endl;

	ret = camera_->stop();
	if (ret)
		std::cout << "Failed to stop capture" << std::endl;

	return ret;
}

void Capture::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	/*
	 * Defer processing of the completed request to the event loop, to avoid
	 * blocking the camera manager thread.
	 */
	loop_->callLater([=]() { processRequest(request); });
}

void Capture::processRequest(Request *request)
{
	const Request::BufferMap &buffers = request->buffers();

	/*
	 * Compute the frame rate. The timestamp is arbitrarily retrieved from
	 * the first buffer, as all buffers should have matching timestamps.
	 */
	uint64_t ts = buffers.begin()->second->metadata().timestamp;
	double fps = ts - last_;
	fps = last_ != 0 && fps ? 1000000000.0 / fps : 0.0;
	last_ = ts;

	std::stringstream info;
	info << ts / 1000000000 << "."
	     << std::setw(6) << std::setfill('0') << ts / 1000 % 1000000
	     << " (" << std::fixed << std::setprecision(2) << fps << " fps)";

	for (auto it = buffers.begin(); it != buffers.end(); ++it) {
		const Stream *stream = it->first;
		FrameBuffer *buffer = it->second;
		const std::string &name = streamName_[stream];

		const FrameMetadata &metadata = buffer->metadata();

		info << " " << name
		     << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence
		     << " bytesused: ";

		unsigned int nplane = 0;
		for (const FrameMetadata::Plane &plane : metadata.planes) {
			info << plane.bytesused;
			if (++nplane < metadata.planes.size())
				info << "/";
		}

		if (writer_)
			writer_->write(buffer, name);
	}

	std::cout << info.str() << std::endl;

	captureCount_++;
	if (captureLimit_ && captureCount_ >= captureLimit_) {
		loop_->exit(0);
		return;
	}

	request->reuse(Request::ReuseBuffers);
	camera_->queueRequest(request);
}
