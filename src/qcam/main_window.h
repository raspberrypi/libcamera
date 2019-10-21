/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.h - qcam - Main application window
 */
#ifndef __QCAM_MAIN_WINDOW_H__
#define __QCAM_MAIN_WINDOW_H__

#include <memory>

#include <QElapsedTimer>
#include <QMainWindow>
#include <QObject>
#include <QTimer>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/stream.h>

#include "../cam/options.h"

using namespace libcamera;

class ViewFinder;

enum {
	OptCamera = 'c',
	OptHelp = 'h',
	OptSize = 's',
};

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(CameraManager *cm, const OptionsParser::Options &options);
	~MainWindow();

private Q_SLOTS:
	void updateTitle();

private:
	std::string chooseCamera(CameraManager *cm);
	int openCamera(CameraManager *cm);

	int startCapture();
	void stopCapture();

	void requestComplete(Request *request);
	int display(Buffer *buffer);

	QString title_;
	QTimer titleTimer_;

	const OptionsParser::Options &options_;

	std::shared_ptr<Camera> camera_;
	bool isCapturing_;
	std::unique_ptr<CameraConfiguration> config_;

	uint64_t lastBufferTime_;

	QElapsedTimer frameRateInterval_;
	uint32_t previousFrames_;
	uint32_t framesCaptured_;

	ViewFinder *viewfinder_;
};

#endif /* __QCAM_MAIN_WINDOW__ */
