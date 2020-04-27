/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <iomanip>
#include <iostream>
#include <signal.h>
#include <string.h>

#include <libcamera/libcamera.h>
#include <libcamera/property_ids.h>

#include "capture.h"
#include "event_loop.h"
#include "main.h"
#include "options.h"
#include "stream_options.h"

using namespace libcamera;

class CamApp
{
public:
	CamApp();
	~CamApp();

	static CamApp *instance();

	int init(int argc, char **argv);
	void cleanup();

	int exec();
	void quit();

private:
	int parseOptions(int argc, char *argv[]);
	int prepareConfig();
	int listControls();
	int listProperties();
	int infoConfiguration();
	int run();

	static CamApp *app_;
	OptionsParser::Options options_;
	CameraManager *cm_;
	std::shared_ptr<Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;
	EventLoop *loop_;
};

CamApp *CamApp::app_ = nullptr;

CamApp::CamApp()
	: cm_(nullptr), camera_(nullptr), config_(nullptr), loop_(nullptr)
{
	CamApp::app_ = this;
}

CamApp::~CamApp()
{
	delete cm_;
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
		return ret;

	cm_ = new CameraManager();

	ret = cm_->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return ret;
	}

	if (options_.isSet(OptCamera)) {
		const std::string &cameraName = options_[OptCamera];
		char *endptr;
		unsigned long index = strtoul(cameraName.c_str(), &endptr, 10);
		if (*endptr == '\0' && index > 0 && index <= cm_->cameras().size())
			camera_ = cm_->cameras()[index - 1];
		else
			camera_ = cm_->get(cameraName);

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

		ret = prepareConfig();
		if (ret)
			return ret;
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

	config_.reset();

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
	StreamKeyValueParser streamKeyValue;

	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on, by name or by index", "camera",
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
	parser.addOption(OptInfo, OptionNone,
			 "Display information about stream(s)", "info");
	parser.addOption(OptList, OptionNone, "List all cameras", "list");
	parser.addOption(OptListControls, OptionNone, "List cameras controls",
			 "list-controls");
	parser.addOption(OptListProperties, OptionNone, "List cameras properties",
			 "list-properties");

	options_ = parser.parse(argc, argv);
	if (!options_.valid())
		return -EINVAL;

	if (options_.empty() || options_.isSet(OptHelp)) {
		parser.usage();
		return options_.empty() ? -EINVAL : -EINTR;
	}

	return 0;
}

int CamApp::prepareConfig()
{
	StreamRoles roles = StreamKeyValueParser::roles(options_[OptStream]);

	config_ = camera_->generateConfiguration(roles);
	if (!config_ || config_->size() != roles.size()) {
		std::cerr << "Failed to get default stream configuration"
			  << std::endl;
		return -EINVAL;
	}

	/* Apply configuration if explicitly requested. */
	if (StreamKeyValueParser::updateConfiguration(config_.get(),
						      options_[OptStream])) {
		std::cerr << "Failed to update configuration" << std::endl;
		return -EINVAL;
	}

	switch (config_->validate()) {
	case CameraConfiguration::Valid:
		break;
	case CameraConfiguration::Adjusted:
		std::cout << "Camera configuration adjusted" << std::endl;
		break;
	case CameraConfiguration::Invalid:
		std::cout << "Camera configuration invalid" << std::endl;
		config_.reset();
		return -EINVAL;
	}

	return 0;
}

int CamApp::listControls()
{
	if (!camera_) {
		std::cout << "Cannot list controls without a camera"
			  << std::endl;
		return -EINVAL;
	}

	for (const auto &ctrl : camera_->controls()) {
		const ControlId *id = ctrl.first;
		const ControlInfo &info = ctrl.second;

		std::cout << "Control: " << id->name() << ": "
			  << info.toString() << std::endl;
	}

	return 0;
}

int CamApp::listProperties()
{
	if (!camera_) {
		std::cout << "Cannot list properties without a camera"
			  << std::endl;
		return -EINVAL;
	}

	for (const auto &prop : camera_->properties()) {
		const ControlId *id = properties::properties.at(prop.first);
		const ControlValue &value = prop.second;

		std::cout << "Property: " << id->name() << " = "
			  << value.toString() << std::endl;
	}

	return 0;
}

int CamApp::infoConfiguration()
{
	if (!config_) {
		std::cout << "Cannot print stream information without a camera"
			  << std::endl;
		return -EINVAL;
	}

	unsigned int index = 0;
	for (const StreamConfiguration &cfg : *config_) {
		std::cout << index << ": " << cfg.toString() << std::endl;

		const StreamFormats &formats = cfg.formats();
		for (PixelFormat pixelformat : formats.pixelformats()) {
			std::cout << " * Pixelformat: "
				  << pixelformat.toString() << " "
				  << formats.range(pixelformat).toString()
				  << std::endl;

			for (const Size &size : formats.sizes(pixelformat))
				std::cout << "  - " << size.toString()
					  << std::endl;
		}

		index++;
	}

	return 0;
}

int CamApp::run()
{
	int ret;

	if (options_.isSet(OptList)) {
		std::cout << "Available cameras:" << std::endl;

		unsigned int index = 1;
		for (const std::shared_ptr<Camera> &cam : cm_->cameras()) {
			std::cout << index << ": " << cam->name() << std::endl;
			index++;
		}
	}

	if (options_.isSet(OptListControls)) {
		ret = listControls();
		if (ret)
			return ret;
	}

	if (options_.isSet(OptListProperties)) {
		ret = listProperties();
		if (ret)
			return ret;
	}

	if (options_.isSet(OptInfo)) {
		ret = infoConfiguration();
		if (ret)
			return ret;
	}

	if (options_.isSet(OptCapture)) {
		Capture capture(camera_, config_.get());
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
	int ret;

	ret = app.init(argc, argv);
	if (ret)
		return ret == -EINTR ? 0 : EXIT_FAILURE;

	struct sigaction sa = {};
	sa.sa_handler = &signalHandler;
	sigaction(SIGINT, &sa, nullptr);

	if (app.exec())
		return EXIT_FAILURE;

	return 0;
}
