/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.h - qcam - Main application window
 */
#ifndef __QCAM_MAIN_WINDOW_H__
#define __QCAM_MAIN_WINDOW_H__

#include <map>
#include <memory>

#include <QMainWindow>

#include <libcamera/camera.h>
#include <libcamera/stream.h>

#include "../cam/options.h"

using namespace libcamera;

class ViewFinder;

enum {
	OptCamera = 'c',
	OptHelp = 'h',
};

class MainWindow : public QMainWindow
{
public:
	MainWindow(const OptionsParser::Options &options);
	~MainWindow();

private:
	int openCamera();

	int startCapture();
	void stopCapture();

	void requestComplete(Request *request,
			     const std::map<Stream *, Buffer *> &buffers);
	int display(Buffer *buffer);

	const OptionsParser::Options &options_;

	std::shared_ptr<Camera> camera_;
	bool isCapturing_;
	std::unique_ptr<CameraConfiguration> config_;

	ViewFinder *viewfinder_;
};

#endif /* __QCAM_MAIN_WINDOW__ */
