/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <signal.h>
#include <string.h>

#include <QApplication>
#include <QtDebug>

#include <libcamera/camera_manager.h>

#include "../cam/options.h"
#include "../cam/stream_options.h"
#include "main_window.h"

void signalHandler(int signal)
{
	qInfo() << "Exiting";
	qApp->quit();
}

OptionsParser::Options parseOptions(int argc, char *argv[])
{
	StreamKeyValueParser streamKeyValue;

	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptHelp, OptionNone, "Display this help message",
			 "help");
	parser.addOption(OptStream, &streamKeyValue,
			 "Set configuration of a camera stream", "stream", true);

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

	CameraManager *cm = new CameraManager();

	ret = cm->start();
	if (ret) {
		qInfo() << "Failed to start camera manager:"
			<< strerror(-ret);
		return EXIT_FAILURE;
	}

	MainWindow *mainWindow = new MainWindow(cm, options);
	mainWindow->show();
	ret = app.exec();
	delete mainWindow;

	cm->stop();
	delete cm;

	return ret;
}
