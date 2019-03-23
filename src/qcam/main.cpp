/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <iostream>
#include <signal.h>
#include <string.h>

#include <QApplication>

#include <libcamera/camera_manager.h>

#include "main_window.h"
#include "../cam/options.h"
#include "qt_event_dispatcher.h"

void signalHandler(int signal)
{
	std::cout << "Exiting" << std::endl;
	qApp->quit();
}

OptionsParser::Options parseOptions(int argc, char *argv[])
{
	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptHelp, OptionNone, "Display this help message",
			 "help");

	OptionsParser::Options options = parser.parse(argc, argv);
	if (options.isSet(OptHelp))
		parser.usage();

	return options;
}

int main(int argc, char **argv)
{
	QApplication app(argc, argv);
	int ret;

	OptionsParser::Options options = parseOptions(argc, argv);
	if (!options.valid())
		return EXIT_FAILURE;
	if (options.isSet(OptHelp))
		return 0;

	struct sigaction sa = {};
	sa.sa_handler = &signalHandler;
	sigaction(SIGINT, &sa, nullptr);

	std::unique_ptr<EventDispatcher> dispatcher(new QtEventDispatcher());
	CameraManager *cm = CameraManager::instance();
	cm->setEventDispatcher(std::move(dispatcher));

	ret = cm->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return EXIT_FAILURE;
	}

	MainWindow *mainWindow = new MainWindow(options);
	mainWindow->show();
	ret = app.exec();
	delete mainWindow;

	cm->stop();
	return ret;
}
