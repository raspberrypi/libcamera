/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <iostream>
#include <map>
#include <signal.h>
#include <string.h>

#include <libcamera/libcamera.h>

#include "event_loop.h"
#include "options.h"

using namespace libcamera;

OptionsParser::Options options;

enum {
	OptCamera = 'c',
	OptFormat = 'f',
	OptHelp = 'h',
	OptList = 'l',
};

EventLoop *loop;

void signalHandler(int signal)
{
	std::cout << "Exiting" << std::endl;
	loop->exit();
}

static int parseOptions(int argc, char *argv[])
{
	KeyValueParser formatKeyValue;
	formatKeyValue.addOption("width", OptionInteger, "Width in pixels",
				 ArgumentRequired);
	formatKeyValue.addOption("height", OptionInteger, "Height in pixels",
				 ArgumentRequired);
	formatKeyValue.addOption("pixelformat", OptionInteger, "Pixel format",
				 ArgumentRequired);

	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptFormat, &formatKeyValue,
			 "Set format of the camera's first stream", "format");
	parser.addOption(OptHelp, OptionNone, "Display this help message",
			 "help");
	parser.addOption(OptList, OptionNone, "List all cameras", "list");

	options = parser.parse(argc, argv);
	if (!options.valid())
		return -EINVAL;

	if (argc == 1 || options.isSet(OptHelp)) {
		parser.usage();
		return 1;
	}

	return 0;
}

bool configureStreams(Camera *camera, std::vector<Stream *> &streams)
{
	KeyValueParser::Options format = options[OptFormat];

	if (streams.size() != 1) {
		std::cout << "Camera has " << streams.size()
			  << " streams, I only know how to work with 1"
			  << std::endl;
		return false;
	}
	Stream *id = streams.front();

	std::map<Stream *, StreamConfiguration> config =
		camera->streamConfiguration(streams);

	if (format.isSet("width"))
		config[id].width = format["width"];

	if (format.isSet("height"))
		config[id].height = format["height"];

	/* TODO: Translate 4CC string to ID. */
	if (format.isSet("pixelformat"))
		config[id].pixelFormat = format["pixelformat"];

	if (camera->configureStreams(config))
		return false;

	return true;
}

int main(int argc, char **argv)
{
	int ret;

	ret = parseOptions(argc, argv);
	if (ret < 0)
		return EXIT_FAILURE;

	CameraManager *cm = CameraManager::instance();
	std::shared_ptr<Camera> camera;
	std::vector<Stream *> streams;

	ret = cm->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return EXIT_FAILURE;
	}

	loop = new EventLoop(cm->eventDispatcher());

	struct sigaction sa = {};
	sa.sa_handler = &signalHandler;
	sigaction(SIGINT, &sa, nullptr);

	if (options.isSet(OptList)) {
		std::cout << "Available cameras:" << std::endl;
		for (const std::shared_ptr<Camera> &cam : cm->cameras())
			std::cout << "- " << cam->name() << std::endl;
	}

	if (options.isSet(OptCamera)) {
		camera = cm->get(options[OptCamera]);
		if (!camera) {
			std::cout << "Camera " << options[OptCamera]
				  << " not found" << std::endl;
			goto out;
		}

		streams = camera->streams();

		if (camera->acquire()) {
			std::cout << "Failed to acquire camera" << std::endl;
			goto out;
		}

		std::cout << "Using camera " << camera->name() << std::endl;
	}

	if (options.isSet(OptFormat)) {
		if (!camera) {
			std::cout << "Can't configure stream, no camera selected"
				  << std::endl;
			goto out_camera;
		}

		if (!configureStreams(camera.get(), streams)) {
			std::cout << "Failed to configure camera" << std::endl;
			goto out_camera;
		}
	}

	ret = loop->exec();

out_camera:
	if (camera) {
		camera->release();
		camera.reset();
	}
out:
	delete loop;

	cm->stop();

	return ret;
}
