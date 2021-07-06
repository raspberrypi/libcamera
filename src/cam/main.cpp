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

#include "camera_session.h"
#include "event_loop.h"
#include "main.h"
#include "options.h"
#include "stream_options.h"

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
	void cameraAdded(std::shared_ptr<Camera> cam);
	void cameraRemoved(std::shared_ptr<Camera> cam);
	void captureDone();
	int parseOptions(int argc, char *argv[]);
	int listControls();
	int listProperties();
	int infoConfiguration();
	int run();

	static std::string cameraName(const Camera *camera);

	static CamApp *app_;
	OptionsParser::Options options_;

	std::unique_ptr<CameraManager> cm_;
	std::unique_ptr<CameraSession> session_;

	EventLoop loop_;
};

CamApp *CamApp::app_ = nullptr;

CamApp::CamApp()
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
		return ret;

	cm_ = std::make_unique<CameraManager>();

	ret = cm_->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return ret;
	}

	if (options_.isSet(OptCamera)) {
		session_ = std::make_unique<CameraSession>(cm_.get(), options_);
		if (!session_->isValid()) {
			std::cout << "Failed to create camera session" << std::endl;
			cleanup();
			return -EINVAL;
		}

		std::cout << "Using camera " << session_->camera()->id()
			  << std::endl;

		session_->captureDone.connect(this, &CamApp::captureDone);
	}

	if (options_.isSet(OptMonitor)) {
		cm_->cameraAdded.connect(this, &CamApp::cameraAdded);
		cm_->cameraRemoved.connect(this, &CamApp::cameraRemoved);
		std::cout << "Monitoring new hotplug and unplug events" << std::endl;
	}

	return 0;
}

void CamApp::cleanup()
{
	session_.reset();

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
	loop_.exit();
}

int CamApp::parseOptions(int argc, char *argv[])
{
	StreamKeyValueParser streamKeyValue;

	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on, by id or by index", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptCapture, OptionInteger,
			 "Capture until interrupted by user or until <count> frames captured",
			 "capture", ArgumentOptional, "count");
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
	parser.addOption(OptMonitor, OptionNone,
			 "Monitor for hotplug and unplug camera events",
			 "monitor");
	parser.addOption(OptStrictFormats, OptionNone,
			 "Do not allow requested stream format(s) to be adjusted",
			 "strict-formats");
	parser.addOption(OptMetadata, OptionNone,
			 "Print the metadata for completed requests",
			 "metadata");

	options_ = parser.parse(argc, argv);
	if (!options_.valid())
		return -EINVAL;

	if (options_.empty() || options_.isSet(OptHelp)) {
		parser.usage();
		return options_.empty() ? -EINVAL : -EINTR;
	}

	return 0;
}

int CamApp::listControls()
{
	if (!session_) {
		std::cout << "Cannot list controls without a camera"
			  << std::endl;
		return -EINVAL;
	}

	for (const auto &ctrl : session_->camera()->controls()) {
		const ControlId *id = ctrl.first;
		const ControlInfo &info = ctrl.second;

		std::cout << "Control: " << id->name() << ": "
			  << info.toString() << std::endl;
	}

	return 0;
}

int CamApp::listProperties()
{
	if (!session_) {
		std::cout << "Cannot list properties without a camera"
			  << std::endl;
		return -EINVAL;
	}

	for (const auto &prop : session_->camera()->properties()) {
		const ControlId *id = properties::properties.at(prop.first);
		const ControlValue &value = prop.second;

		std::cout << "Property: " << id->name() << " = "
			  << value.toString() << std::endl;
	}

	return 0;
}

int CamApp::infoConfiguration()
{
	if (!session_) {
		std::cout << "Cannot print stream information without a camera"
			  << std::endl;
		return -EINVAL;
	}

	unsigned int index = 0;
	for (const StreamConfiguration &cfg : *session_->config()) {
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

void CamApp::cameraAdded(std::shared_ptr<Camera> cam)
{
	std::cout << "Camera Added: " << cam->id() << std::endl;
}

void CamApp::cameraRemoved(std::shared_ptr<Camera> cam)
{
	std::cout << "Camera Removed: " << cam->id() << std::endl;
}

void CamApp::captureDone()
{
	EventLoop::instance()->exit(0);
}

int CamApp::run()
{
	int ret;

	if (options_.isSet(OptList)) {
		std::cout << "Available cameras:" << std::endl;

		unsigned int index = 1;
		for (const std::shared_ptr<Camera> &cam : cm_->cameras()) {
			std::cout << index << ": " << cameraName(cam.get()) << std::endl;
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
		if (!session_) {
			std::cout << "Can't capture without a camera" << std::endl;
			return -ENODEV;
		}

		ret = session_->start(options_);
		if (ret) {
			std::cout << "Failed to start camera session" << std::endl;
			return ret;
		}

		loop_.exec();

		session_->stop();
		return 0;
	}

	if (options_.isSet(OptMonitor)) {
		std::cout << "Press Ctrl-C to interrupt" << std::endl;
		loop_.exec();
	}

	return 0;
}

std::string CamApp::cameraName(const Camera *camera)
{
	const ControlList &props = camera->properties();
	bool addModel = true;
	std::string name;

	/*
	 * Construct the name from the camera location, model and ID. The model
	 * is only used if the location isn't present or is set to External.
	 */
	if (props.contains(properties::Location)) {
		switch (props.get(properties::Location)) {
		case properties::CameraLocationFront:
			addModel = false;
			name = "Internal front camera ";
			break;
		case properties::CameraLocationBack:
			addModel = false;
			name = "Internal back camera ";
			break;
		case properties::CameraLocationExternal:
			name = "External camera ";
			break;
		}
	}

	if (addModel && props.contains(properties::Model)) {
		/*
		 * If the camera location is not availble use the camera model
		 * to build the camera name.
		 */
		name = "'" + props.get(properties::Model) + "' ";
	}

	name += "(" + camera->id() + ")";

	return name;
}

void signalHandler([[maybe_unused]] int signal)
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
