/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <atomic>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <string.h>

#include <libcamera/libcamera.h>
#include <libcamera/property_ids.h>

#include "../common/event_loop.h"
#include "../common/options.h"
#include "../common/stream_options.h"

#include "camera_session.h"
#include "main.h"

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
	int run();

	static std::string cameraName(const Camera *camera);

	static CamApp *app_;
	OptionsParser::Options options_;

	std::unique_ptr<CameraManager> cm_;

	std::atomic_uint loopUsers_;
	EventLoop loop_;
};

CamApp *CamApp::app_ = nullptr;

CamApp::CamApp()
	: loopUsers_(0)
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

	return 0;
}

void CamApp::cleanup()
{
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
			 ArgumentRequired, "camera", true);
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

	/* Sub-options of OptCamera: */
	parser.addOption(OptCapture, OptionInteger,
			 "Capture until interrupted by user or until <count> frames captured",
			 "capture", ArgumentOptional, "count", false,
			 OptCamera);
#ifdef HAVE_KMS
	parser.addOption(OptDisplay, OptionString,
			 "Display viewfinder through DRM/KMS on specified connector",
			 "display", ArgumentOptional, "connector", false,
			 OptCamera);
#endif
	parser.addOption(OptFile, OptionString,
			 "Write captured frames to disk\n"
			 "If the file name ends with a '/', it sets the directory in which\n"
			 "to write files, using the default file name. Otherwise it sets the\n"
			 "full file path and name. The first '#' character in the file name\n"
			 "is expanded to the camera index, stream name and frame sequence number.\n"
#ifdef HAVE_TIFF
			 "If the file name ends with '.dng', then the frame will be written to\n"
			 "the output file(s) in DNG format.\n"
#endif
			 "The default file name is 'frame-#.bin'.",
			 "file", ArgumentOptional, "filename", false,
			 OptCamera);
#ifdef HAVE_SDL
	parser.addOption(OptSDL, OptionNone, "Display viewfinder through SDL",
			 "sdl", ArgumentNone, "", false, OptCamera);
#endif
	parser.addOption(OptStream, &streamKeyValue,
			 "Set configuration of a camera stream", "stream", true,
			 OptCamera);
	parser.addOption(OptStrictFormats, OptionNone,
			 "Do not allow requested stream format(s) to be adjusted",
			 "strict-formats", ArgumentNone, nullptr, false,
			 OptCamera);
	parser.addOption(OptMetadata, OptionNone,
			 "Print the metadata for completed requests",
			 "metadata", ArgumentNone, nullptr, false,
			 OptCamera);
	parser.addOption(OptCaptureScript, OptionString,
			 "Load a capture session configuration script from a file",
			 "script", ArgumentRequired, "script", false,
			 OptCamera);

	options_ = parser.parse(argc, argv);
	if (!options_.valid())
		return -EINVAL;

	if (options_.empty() || options_.isSet(OptHelp)) {
		parser.usage();
		return options_.empty() ? -EINVAL : -EINTR;
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
	if (--loopUsers_ == 0)
		EventLoop::instance()->exit(0);
}

int CamApp::run()
{
	int ret;

	/* 1. List all cameras. */
	if (options_.isSet(OptList)) {
		std::cout << "Available cameras:" << std::endl;

		unsigned int index = 1;
		for (const std::shared_ptr<Camera> &cam : cm_->cameras()) {
			std::cout << index << ": " << cameraName(cam.get()) << std::endl;
			index++;
		}
	}

	/* 2. Create the camera sessions. */
	std::vector<std::unique_ptr<CameraSession>> sessions;

	if (options_.isSet(OptCamera)) {
		unsigned int index = 0;

		for (const OptionValue &camera : options_[OptCamera].toArray()) {
			std::unique_ptr<CameraSession> session =
				std::make_unique<CameraSession>(cm_.get(),
								camera.toString(),
								index,
								camera.children());
			if (!session->isValid()) {
				std::cout << "Failed to create camera session" << std::endl;
				return -EINVAL;
			}

			std::cout << "Using camera " << session->camera()->id()
				  << " as cam" << index << std::endl;

			session->captureDone.connect(this, &CamApp::captureDone);

			sessions.push_back(std::move(session));
			index++;
		}
	}

	/* 3. Print camera information. */
	if (options_.isSet(OptListControls) ||
	    options_.isSet(OptListProperties) ||
	    options_.isSet(OptInfo)) {
		for (const auto &session : sessions) {
			if (options_.isSet(OptListControls))
				session->listControls();
			if (options_.isSet(OptListProperties))
				session->listProperties();
			if (options_.isSet(OptInfo))
				session->infoConfiguration();
		}
	}

	/* 4. Start capture. */
	for (const auto &session : sessions) {
		if (!session->options().isSet(OptCapture))
			continue;

		ret = session->start();
		if (ret) {
			std::cout << "Failed to start camera session" << std::endl;
			return ret;
		}

		loopUsers_++;
	}

	/* 5. Enable hotplug monitoring. */
	if (options_.isSet(OptMonitor)) {
		std::cout << "Monitoring new hotplug and unplug events" << std::endl;
		std::cout << "Press Ctrl-C to interrupt" << std::endl;

		cm_->cameraAdded.connect(this, &CamApp::cameraAdded);
		cm_->cameraRemoved.connect(this, &CamApp::cameraRemoved);

		loopUsers_++;
	}

	if (loopUsers_)
		loop_.exec();

	/* 6. Stop capture. */
	for (const auto &session : sessions) {
		if (!session->options().isSet(OptCapture))
			continue;

		session->stop();
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
	const auto &location = props.get(properties::Location);
	if (location) {
		switch (*location) {
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

	if (addModel) {
		/*
		 * If the camera location is not availble use the camera model
		 * to build the camera name.
		 */
		const auto &model = props.get(properties::Model);
		if (model)
			name = "'" + *model + "' ";
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
