/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 * Copyright (C) 2021, Collabora Ltd.
 *
 * main.cpp - lc-compliance - The libcamera compliance tool
 */

#include <iomanip>
#include <iostream>
#include <string.h>

#include <gtest/gtest.h>

#include <libcamera/libcamera.h>

#include "environment.h"
#include "../cam/options.h"

using namespace libcamera;

enum {
	OptCamera = 'c',
	OptHelp = 'h',
};

/*
 * Make asserts act like exceptions, otherwise they only fail (or skip) the
 * current function. From gtest documentation:
 * https://google.github.io/googletest/advanced.html#asserting-on-subroutines-with-an-exception
 */
class ThrowListener : public testing::EmptyTestEventListener
{
	void OnTestPartResult(const testing::TestPartResult &result) override
	{
		if (result.type() == testing::TestPartResult::kFatalFailure ||
		    result.type() == testing::TestPartResult::kSkip)
			throw testing::AssertionException(result);
	}
};

static void listCameras(CameraManager *cm)
{
	for (const std::shared_ptr<Camera> &cam : cm->cameras())
		std::cout << "- " << cam.get()->id() << std::endl;
}

static int initCamera(CameraManager *cm, OptionsParser::Options options)
{
	std::shared_ptr<Camera> camera;

	int ret = cm->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return ret;
	}

	if (!options.isSet(OptCamera)) {
		std::cout << "No camera specified, available cameras:" << std::endl;
		listCameras(cm);
		return -ENODEV;
	}

	const std::string &cameraId = options[OptCamera];
	camera = cm->get(cameraId);
	if (!camera) {
		std::cout << "Camera " << cameraId << " not found, available cameras:" << std::endl;
		listCameras(cm);
		return -ENODEV;
	}

	Environment::get()->setup(cm, cameraId);

	std::cout << "Using camera " << cameraId << std::endl;

	return 0;
}

static int parseOptions(int argc, char **argv, OptionsParser::Options *options)
{
	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on, by id", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptHelp, OptionNone, "Display this help message",
			 "help");

	*options = parser.parse(argc, argv);
	if (!options->valid())
		return -EINVAL;

	if (options->isSet(OptHelp)) {
		parser.usage();
		return -EINTR;
	}

	return 0;
}

int main(int argc, char **argv)
{
	OptionsParser::Options options;
	int ret = parseOptions(argc, argv, &options);
	if (ret == -EINTR)
		return EXIT_SUCCESS;
	if (ret < 0)
		return EXIT_FAILURE;

	std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();

	ret = initCamera(cm.get(), options);
	if (ret)
		return ret;

	testing::UnitTest::GetInstance()->listeners().Append(new ThrowListener);

	ret = RUN_ALL_TESTS();

	cm->stop();

	return ret;
}
