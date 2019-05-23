/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <iostream>
#include <signal.h>
#include <string.h>

#include <libcamera/libcamera.h>

#include "capture.h"
#include "event_loop.h"
#include "main.h"
#include "options.h"

using namespace libcamera;

class CamApp
{
public:
	CamApp();

	static CamApp *instance();

	int init(int argc, char **argv);
	void cleanup();

	int exec();
	void quit();

private:
	int parseOptions(int argc, char *argv[]);
	int run();

	static CamApp *app_;
	OptionsParser::Options options_;
	CameraManager *cm_;
	std::shared_ptr<Camera> camera_;
	EventLoop *loop_;
};

CamApp *CamApp::app_ = nullptr;

CamApp::CamApp()
	: cm_(nullptr), camera_(nullptr), loop_(nullptr)
{
	CamApp::app_ = this;
}

CamApp *CamApp::instance()
{
	return CamApp::app_;
}

int CamApp::init(int argc, char **argv)
{
	int ret;

	ret = parseOptions(argc, argv);
	if (ret < 0)
		return ret == -EINTR ? 0 : ret;

	cm_ = CameraManager::instance();

	ret = cm_->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return ret;
	}

	if (options_.isSet(OptCamera)) {
		camera_ = cm_->get(options_[OptCamera]);
		if (!camera_) {
			std::cout << "Camera "
				  << std::string(options_[OptCamera])
				  << " not found" << std::endl;
			cm_->stop();
			return -ENODEV;
		}

		if (camera_->acquire()) {
			std::cout << "Failed to acquire camera" << std::endl;
			camera_.reset();
			cm_->stop();
			return -EINVAL;
		}

		std::cout << "Using camera " << camera_->name() << std::endl;
	}

	loop_ = new EventLoop(cm_->eventDispatcher());

	return 0;
}

void CamApp::cleanup()
{
	delete loop_;
	loop_ = nullptr;

	if (camera_) {
		camera_->release();
		camera_.reset();
	}

	cm_->stop();
}

int CamApp::exec()
{
	int ret;

	ret = run();
	cleanup();

	return ret;
}

void CamApp::quit()
{
	if (loop_)
		loop_->exit();
}

int CamApp::parseOptions(int argc, char *argv[])
{
	KeyValueParser streamKeyValue;
	streamKeyValue.addOption("role", OptionString,
				 "Role for the stream (viewfinder, video, still)",
				 ArgumentRequired);
	streamKeyValue.addOption("width", OptionInteger, "Width in pixels",
				 ArgumentRequired);
	streamKeyValue.addOption("height", OptionInteger, "Height in pixels",
				 ArgumentRequired);
	streamKeyValue.addOption("pixelformat", OptionInteger, "Pixel format",
				 ArgumentRequired);

	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptCapture, OptionNone,
			 "Capture until interrupted by user", "capture");
	parser.addOption(OptFile, OptionString,
			 "Write captured frames to disk\n"
			 "The first '#' character in the file name is expanded to the stream name and frame sequence number.\n"
			 "The default file name is 'frame-#.bin'.",
			 "file", ArgumentOptional, "filename");
	parser.addOption(OptStream, &streamKeyValue,
			 "Set configuration of a camera stream", "stream", true);
	parser.addOption(OptHelp, OptionNone, "Display this help message",
			 "help");
	parser.addOption(OptList, OptionNone, "List all cameras", "list");

	options_ = parser.parse(argc, argv);
	if (!options_.valid())
		return -EINVAL;

	if (options_.empty() || options_.isSet(OptHelp)) {
		parser.usage();
		return options_.empty() ? -EINVAL : -EINTR;
	}

	return 0;
}

int CamApp::run()
{
	if (options_.isSet(OptList)) {
		std::cout << "Available cameras:" << std::endl;
		for (const std::shared_ptr<Camera> &cam : cm_->cameras())
			std::cout << "- " << cam->name() << std::endl;
	}

	if (options_.isSet(OptCapture)) {
		Capture capture(camera_.get());
		return capture.run(loop_, options_);
	}

	return 0;
}

void signalHandler(int signal)
{
	std::cout << "Exiting" << std::endl;
	CamApp::instance()->quit();
}

int main(int argc, char **argv)
{
	CamApp app;

	if (app.init(argc, argv))
		return EXIT_FAILURE;

	struct sigaction sa = {};
	sa.sa_handler = &signalHandler;
	sigaction(SIGINT, &sa, nullptr);

	if (app.exec())
		return EXIT_FAILURE;

	return 0;
}
