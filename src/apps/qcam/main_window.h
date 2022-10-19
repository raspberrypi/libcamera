/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.h - qcam - Main application window
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <QElapsedTimer>
#include <QIcon>
#include <QMainWindow>
#include <QMutex>
#include <QObject>
#include <QPushButton>
#include <QQueue>
#include <QTimer>

#include "../common/stream_options.h"

#include "viewfinder.h"

class QAction;

class CameraSelectorDialog;
class Image;
class HotplugEvent;

enum {
	OptCamera = 'c',
	OptHelp = 'h',
	OptRenderer = 'r',
	OptStream = 's',
	OptVerbose = 'v',
};

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(libcamera::CameraManager *cm,
		   const OptionsParser::Options &options);
	~MainWindow();

	bool event(QEvent *e) override;

private Q_SLOTS:
	void quit();
	void updateTitle();

	void switchCamera();
	void toggleCapture(bool start);

	void saveImageAs();
	void captureRaw();
	void processRaw(libcamera::FrameBuffer *buffer,
			const libcamera::ControlList &metadata);

	void renderComplete(libcamera::FrameBuffer *buffer);

private:
	int createToolbars();

	std::string chooseCamera();
	int openCamera();

	int startCapture();
	void stopCapture();

	void addCamera(std::shared_ptr<libcamera::Camera> camera);
	void removeCamera(std::shared_ptr<libcamera::Camera> camera);

	int queueRequest(libcamera::Request *request);
	void requestComplete(libcamera::Request *request);
	void processCapture();
	void processHotplug(HotplugEvent *e);
	void processViewfinder(libcamera::FrameBuffer *buffer);

	/* UI elements */
	QToolBar *toolbar_;
	QAction *startStopAction_;
	QPushButton *cameraSelectButton_;
	QAction *saveRaw_;
	ViewFinder *viewfinder_;

	QIcon iconPlay_;
	QIcon iconStop_;

	QString title_;
	QTimer titleTimer_;

	CameraSelectorDialog *cameraSelectorDialog_;

	/* Options */
	const OptionsParser::Options &options_;

	/* Camera manager, camera, configuration and buffers */
	libcamera::CameraManager *cm_;
	std::shared_ptr<libcamera::Camera> camera_;
	libcamera::FrameBufferAllocator *allocator_;

	std::unique_ptr<libcamera::CameraConfiguration> config_;
	std::map<libcamera::FrameBuffer *, std::unique_ptr<Image>> mappedBuffers_;

	/* Capture state, buffers queue and statistics */
	bool isCapturing_;
	bool captureRaw_;
	libcamera::Stream *vfStream_;
	libcamera::Stream *rawStream_;
	std::map<const libcamera::Stream *, QQueue<libcamera::FrameBuffer *>> freeBuffers_;
	QQueue<libcamera::Request *> doneQueue_;
	QQueue<libcamera::Request *> freeQueue_;
	QMutex mutex_; /* Protects freeBuffers_, doneQueue_, and freeQueue_ */

	uint64_t lastBufferTime_;
	QElapsedTimer frameRateInterval_;
	uint32_t previousFrames_;
	uint32_t framesCaptured_;

	std::vector<std::unique_ptr<libcamera::Request>> requests_;
};
