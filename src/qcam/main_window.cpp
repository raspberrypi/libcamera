/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.cpp - qcam - Main application window
 */

#include <iomanip>
#include <iostream>
#include <string>
#include <sys/mman.h>

#include <QCoreApplication>
#include <QInputDialog>
#include <QTimer>

#include <libcamera/camera_manager.h>
#include <libcamera/version.h>

#include "main_window.h"
#include "viewfinder.h"

using namespace libcamera;

MainWindow::MainWindow(CameraManager *cm, const OptionsParser::Options &options)
	: options_(options), isCapturing_(false)
{
	int ret;

	title_ = "QCam " + QString::fromStdString(CameraManager::version());
	setWindowTitle(title_);
	connect(&titleTimer_, SIGNAL(timeout()), this, SLOT(updateTitle()));

	viewfinder_ = new ViewFinder(this);
	setCentralWidget(viewfinder_);
	viewfinder_->setFixedSize(500, 500);
	adjustSize();

	ret = openCamera(cm);
	if (!ret)
		ret = startCapture();

	if (ret < 0)
		QTimer::singleShot(0, QCoreApplication::instance(),
				   &QCoreApplication::quit);
}

MainWindow::~MainWindow()
{
	if (camera_) {
		stopCapture();
		camera_->release();
		camera_.reset();
	}
}

void MainWindow::updateTitle()
{
	unsigned int duration = frameRateInterval_.elapsed();
	unsigned int frames = framesCaptured_ - previousFrames_;
	double fps = frames * 1000.0 / duration;

	/* Restart counters. */
	frameRateInterval_.start();
	previousFrames_ = framesCaptured_;

	setWindowTitle(title_ + " : " + QString::number(fps, 'f', 2) + " fps");
}

std::string MainWindow::chooseCamera(CameraManager *cm)
{
	QStringList cameras;
	bool result;

	if (cm->cameras().size() == 1)
		return cm->cameras()[0]->name();

	for (const std::shared_ptr<Camera> &cam : cm->cameras())
		cameras.append(QString::fromStdString(cam->name()));

	QString name = QInputDialog::getItem(this, "Select Camera",
					     "Camera:", cameras, 0,
					     false, &result);
	if (!result)
		return std::string();

	return name.toStdString();
}

int MainWindow::openCamera(CameraManager *cm)
{
	std::string cameraName;

	if (options_.isSet(OptCamera))
		cameraName = static_cast<std::string>(options_[OptCamera]);
	else
		cameraName = chooseCamera(cm);

	if (cameraName == "")
		return -EINVAL;

	camera_ = cm->get(cameraName);
	if (!camera_) {
		std::cout << "Camera " << cameraName << " not found"
			  << std::endl;
		return -ENODEV;
	}

	if (camera_->acquire()) {
		std::cout << "Failed to acquire camera" << std::endl;
		camera_.reset();
		return -EBUSY;
	}

	std::cout << "Using camera " << camera_->name() << std::endl;

	camera_->requestCompleted.connect(this, &MainWindow::requestComplete);

	return 0;
}

int MainWindow::startCapture()
{
	int ret;

	config_ = camera_->generateConfiguration({ StreamRole::VideoRecording });

	StreamConfiguration &cfg = config_->at(0);
	if (options_.isSet(OptSize)) {
		const std::vector<OptionValue> &sizeOptions =
			options_[OptSize].toArray();

		/* Set desired stream size if requested. */
		for (const auto &value : sizeOptions) {
			KeyValueParser::Options opt = value.toKeyValues();

			if (opt.isSet("width"))
				cfg.size.width = opt["width"];

			if (opt.isSet("height"))
				cfg.size.height = opt["height"];
		}
	}

	CameraConfiguration::Status validation = config_->validate();
	if (validation == CameraConfiguration::Invalid) {
		std::cerr << "Failed to create valid camera configuration";
		return -EINVAL;
	}

	if (validation == CameraConfiguration::Adjusted) {
		std::cout << "Stream size adjusted to "
			  << cfg.size.toString() << std::endl;
	}

	ret = camera_->configure(config_.get());
	if (ret < 0) {
		std::cout << "Failed to configure camera" << std::endl;
		return ret;
	}

	Stream *stream = cfg.stream();
	ret = viewfinder_->setFormat(cfg.pixelFormat, cfg.size.width,
				     cfg.size.height);
	if (ret < 0) {
		std::cout << "Failed to set viewfinder format" << std::endl;
		return ret;
	}

	adjustSize();

	ret = camera_->allocateBuffers();
	if (ret) {
		std::cerr << "Failed to allocate buffers"
			  << std::endl;
		return ret;
	}

	std::vector<Request *> requests;
	for (unsigned int i = 0; i < cfg.bufferCount; ++i) {
		Request *request = camera_->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			ret = -ENOMEM;
			goto error;
		}

		std::unique_ptr<Buffer> buffer = stream->createBuffer(i);
		if (!buffer) {
			std::cerr << "Can't create buffer " << i << std::endl;
			goto error;
		}

		ret = request->addBuffer(std::move(buffer));
		if (ret < 0) {
			std::cerr << "Can't set buffer for request" << std::endl;
			goto error;
		}

		requests.push_back(request);
	}

	titleTimer_.start(2000);
	frameRateInterval_.start();
	previousFrames_ = 0;
	framesCaptured_ = 0;
	lastBufferTime_ = 0;

	ret = camera_->start();
	if (ret) {
		std::cout << "Failed to start capture" << std::endl;
		goto error;
	}

	for (Request *request : requests) {
		ret = camera_->queueRequest(request);
		if (ret < 0) {
			std::cerr << "Can't queue request" << std::endl;
			goto error;
		}
	}

	isCapturing_ = true;
	return 0;

error:
	for (Request *request : requests)
		delete request;

	camera_->freeBuffers();
	return ret;
}

void MainWindow::stopCapture()
{
	if (!isCapturing_)
		return;

	int ret = camera_->stop();
	if (ret)
		std::cout << "Failed to stop capture" << std::endl;

	camera_->freeBuffers();
	isCapturing_ = false;

	config_.reset();

	titleTimer_.stop();
	setWindowTitle(title_);
}

void MainWindow::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	const std::map<Stream *, Buffer *> &buffers = request->buffers();

	framesCaptured_++;

	Buffer *buffer = buffers.begin()->second;

	double fps = buffer->timestamp() - lastBufferTime_;
	fps = lastBufferTime_ && fps ? 1000000000.0 / fps : 0.0;
	lastBufferTime_ = buffer->timestamp();

	std::cout << "seq: " << std::setw(6) << std::setfill('0') << buffer->sequence()
		  << " bytesused: " << buffer->bytesused()
		  << " timestamp: " << buffer->timestamp()
		  << " fps: " << std::fixed << std::setprecision(2) << fps
		  << std::endl;

	display(buffer);

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
			std::cerr << "Can't create buffer" << std::endl;
			return;
		}

		request->addBuffer(std::move(newBuffer));
	}

	camera_->queueRequest(request);
}

int MainWindow::display(Buffer *buffer)
{
	if (buffer->mem()->planes().size() != 1)
		return -EINVAL;

	/* \todo Once the FrameBuffer is done cache mapped memory. */
	const FrameBuffer::Plane &plane = buffer->mem()->planes().front();
	void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
			    plane.fd.fd(), 0);

	unsigned char *raw = static_cast<unsigned char *>(memory);
	viewfinder_->display(raw, buffer->bytesused());

	munmap(memory, plane.length);

	return 0;
}
