/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam-ctl a tool to interact with the library
 */

#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <string.h>

#include <libcamera/libcamera.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

using namespace std;
using namespace libcamera;

enum Option {
	OptCamera = 'c',
	OptHelp = 'h',
	OptList = 'l',
	OptLast = 0,
};

struct OptionInfo {
	Option id;
	const char *name;
	const char *arguments;
	const char *description;
};

static struct OptionInfo option_info[] = {
	{ OptCamera, "camera", "<camera>", "Specify which camera to operate on" },
	{ OptHelp, "help", nullptr, "Display this help message" },
	{ OptList, "list", nullptr, "List all cameras" },
	{ OptLast, nullptr, nullptr, nullptr },
};

std::map<Option, std::string> options;

void usage()
{
	struct OptionInfo *info;

	cout << "Options:" << endl;
	for (info = option_info; info->id != OptLast; info++) {
		string arg(info->name);

		if (info->arguments)
			arg += string(" ") + info->arguments;

		cout << "  -" << static_cast<char>(info->id) << " --" <<
			setw(20) << left << arg << " - " <<
			info->description << endl;
	}
}

int parseOptions(int argc, char **argv)
{
	char short_options[ARRAY_SIZE(option_info) * 2 + 1];
	struct option long_options[ARRAY_SIZE(option_info)];
	struct OptionInfo *info;
	unsigned ids = 0, idl = 0;

	memset(short_options, 0, sizeof(short_options));
	memset(long_options, 0, sizeof(long_options));

	for (info = option_info; info->id != OptLast; info++) {
		short_options[ids++] = info->id;
		if (info->arguments)
			short_options[ids++] = ':';

		long_options[idl].name = info->name;
		long_options[idl].has_arg =
			info->arguments ? required_argument : no_argument;
		long_options[idl].flag = 0;
		long_options[idl].val = info->id;
		idl++;
	}

	while (true) {
		int c = getopt_long(argc, argv, short_options, long_options, nullptr);

		if (c == -1)
			break;

		if (!isalpha(c))
			return EXIT_FAILURE;

		options[static_cast<Option>(c)] = optarg ? string(optarg) : "";
	}

	return 0;
}

bool optSet(Option opt)
{
	return options.count(opt) != 0;
}

int main(int argc, char **argv)
{
	int ret;

	ret = parseOptions(argc, argv);
	if (ret == EXIT_FAILURE)
		return ret;

	if (argc == 1 || optSet(OptHelp)) {
		usage();
		return 0;
	}

	CameraManager *cm = CameraManager::instance();

	ret = cm->start();
	if (ret) {
		cout << "Failed to start camera manager: " << strerror(-ret) << endl;
		return EXIT_FAILURE;
	}

	if (optSet(OptList)) {
		cout << "Available cameras:" << endl;
		for (const std::shared_ptr<Camera> &camera : cm->cameras())
			cout << "- " << camera->name() << endl;
	}

	if (optSet(OptCamera)) {
		std::shared_ptr<Camera> cam = cm->get(options[OptCamera]);

		if (cam) {
			cout << "Using camera " << cam->name() << endl;
		} else {
			cout << "Camera " << options[OptCamera] << " not found" << endl;
		}
	}

	cm->stop();

	return 0;
}
