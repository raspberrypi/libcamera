/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <iostream>
#include <map>
#include <string.h>

#include <libcamera/libcamera.h>

#include "options.h"

using namespace libcamera;

OptionsParser::Options options;

enum {
	OptCamera = 'c',
	OptHelp = 'h',
	OptList = 'l',
};

static int parseOptions(int argc, char *argv[])
{
	OptionsParser parser;

	parser.addOption(OptCamera, "Specify which camera to operate on",
			 "camera", OptionsParser::ArgumentRequired,
			 "camera");
	parser.addOption(OptHelp, "Display this help message", "help");
	parser.addOption(OptList, "List all cameras", "list");

	options = std::move(parser.parse(argc, argv));
	if (!options.valid())
		return -EINVAL;

	if (argc == 1 || options.isSet(OptHelp)) {
		parser.usage();
		return 1;
	}

	return 0;
}

int main(int argc, char **argv)
{
	int ret;

	ret = parseOptions(argc, argv);
	if (ret < 0)
		return EXIT_FAILURE;

	CameraManager *cm = CameraManager::instance();

	ret = cm->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return EXIT_FAILURE;
	}

	if (options.isSet(OptList)) {
		std::cout << "Available cameras:" << std::endl;
		for (const std::shared_ptr<Camera> &camera : cm->cameras())
			std::cout << "- " << camera->name() << std::endl;
	}

	if (options.isSet(OptCamera)) {
		std::shared_ptr<Camera> cam = cm->get(options[OptCamera]);

		if (cam) {
			std::cout << "Using camera " << cam->name() << std::endl;
		} else {
			std::cout << "Camera " << options[OptCamera]
				  << " not found" << std::endl;
		}
	}

	cm->stop();

	return 0;
}
