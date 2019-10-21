/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * capture.cpp - Cam capture
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <sstream>

#include "capture.h"
#include "main.h"

using namespace libcamera;

Capture::Capture(Camera *camera, CameraConfiguration *config)
	: camera_(camera), config_(config), writer_(nullptr)
{
}

int Capture::run(EventLoop *loop, const OptionsParser::Options &options)
{
	int ret;

	if (!camera_) {
		std::cout << "Can't capture without a camera" << std::endl;
		return -ENODEV;
	}

	streamName_.clear();
	for (unsigned int index = 0; index < config_->size(); ++index) {
		StreamConfiguration &cfg = config_->at(index);
		streamName_[cfg.stream()] = "stream" + std::to_string(index);
	}

	ret = camera_->configure(config_);
	if (ret < 0) {
		std::cout << "Failed to configure camera" << std::endl;
		return ret;
	}

	ret = camera_->allocateBuffers();
	if (ret) {
		std::cerr << "Failed to allocate buffers" << std::endl;
		return ret;
	}

	camera_->requestCompleted.connect(this, &Capture::requestComplete);

	if (options.isSet(OptFile)) {
		if (!options[OptFile].toString().empty())
			writer_ = new BufferWriter(options[OptFile]);
		else
			writer_ = new BufferWriter();
	}

	ret = capture(loop);

	if (options.isSet(OptFile)) {
		delete writer_;
		writer_ = nullptr;
	}

	camera_->freeBuffers();

	return ret;
}

int Capture::capture(EventLoop *loop)
{
	int ret;

	/* Identify the stream with the least number of buffers. */
	unsigned int nbuffers = UINT_MAX;
	for (StreamConfiguration &cfg : *config_)
		nbuffers = std::min(nbuffers, cfg.bufferCount);

	/*
	 * TODO: make cam tool smarter to support still capture by for
	 * example pushing a button. For now run all streams all the time.
	 */

	std::vector<Request *> requests;
	for (unsigned int i = 0; i < nbuffers; i++) {
		Request *request = camera_->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			return -ENOMEM;
		}

		for (StreamConfiguration &cfg : *config_) {
			Stream *stream = cfg.stream();
			std::unique_ptr<Buffer> buffer = stream->createBuffer(i);

			ret = request->addBuffer(std::move(buffer));
			if (ret < 0) {
				std::cerr << "Can't set buffer for request"
					  << std::endl;
				return ret;
			}
		}

		requests.push_back(request);
	}

	ret = camera_->start();
	if (ret) {
		std::cout << "Failed to start capture" << std::endl;
		return ret;
	}

	for (Request *request : requests) {
		ret = camera_->queueRequest(request);
		if (ret < 0) {
			std::cerr << "Can't queue request" << std::endl;
			camera_->stop();
			return ret;
		}
	}

	std::cout << "Capture until user interrupts by SIGINT" << std::endl;
	ret = loop->exec();
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

	const std::map<Stream *, Buffer *> &buffers = request->buffers();

	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	double fps = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_).count();
	fps = last_ != std::chrono::steady_clock::time_point() && fps
	    ? 1000.0 / fps : 0.0;
	last_ = now;

	std::stringstream info;
	info << "fps: " << std::fixed << std::setprecision(2) << fps;

	for (auto it = buffers.begin(); it != buffers.end(); ++it) {
		Stream *stream = it->first;
		Buffer *buffer = it->second;
		const std::string &name = streamName_[stream];

		info << " " << name
		     << " (" << buffer->index() << ")"
		     << " seq: " << std::setw(6) << std::setfill('0') << buffer->sequence()
		     << " bytesused: " << buffer->bytesused();

		if (writer_)
			writer_->write(buffer, name);
	}

	std::cout << info.str() << std::endl;

	/*
	 * Create a new request and populate it with one buffer for each
	 * stream.
	 */
	request = camera_->createRequest();
	if (!request) {
		std::cerr << "Can't create request" << std::endl;
		return;
	}

	for (auto it = buffers.begin(); it != buffers.end(); ++it) {
		Stream *stream = it->first;
		Buffer *buffer = it->second;
		unsigned int index = buffer->index();

		std::unique_ptr<Buffer> newBuffer = stream->createBuffer(index);
		if (!newBuffer) {
			std::cerr << "Can't create buffer " << index << std::endl;
			return;
		}

		request->addBuffer(std::move(newBuffer));
	}

	camera_->queueRequest(request);
}
