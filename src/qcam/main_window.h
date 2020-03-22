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
#include <QIcon>
#include <QMainWindow>
#include <QMutex>
#include <QObject>
#include <QQueue>
#include <QTimer>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>

#include "../cam/options.h"
#include "viewfinder.h"

using namespace libcamera;

class QAction;

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

	bool event(QEvent *e) override;

private Q_SLOTS:
	void quit();
	void updateTitle();

	void switchCamera(int index);
	void toggleCapture(bool start);

	void saveImageAs();

private:
	int createToolbars();

	std::string chooseCamera();
	int openCamera();

	int startCapture();
	void stopCapture();

	void requestComplete(Request *request);
	void processCapture();
	int display(FrameBuffer *buffer);
	void queueRequest(FrameBuffer *buffer);

	/* UI elements */
	QToolBar *toolbar_;
	QAction *startStopAction_;
	ViewFinder *viewfinder_;

	QIcon iconPlay_;
	QIcon iconStop_;

	QString title_;
	QTimer titleTimer_;

	/* Options */
	const OptionsParser::Options &options_;

	/* Camera manager, camera, configuration and buffers */
	CameraManager *cm_;
	std::shared_ptr<Camera> camera_;
	FrameBufferAllocator *allocator_;

	std::unique_ptr<CameraConfiguration> config_;
	std::map<FrameBuffer *, MappedBuffer> mappedBuffers_;

	/* Capture state, buffers queue and statistics */
	bool isCapturing_;
	QQueue<FrameBuffer *> doneQueue_;
	QMutex mutex_;	/* Protects doneQueue_ */

	uint64_t lastBufferTime_;
	QElapsedTimer frameRateInterval_;
	uint32_t previousFrames_;
	uint32_t framesCaptured_;
};

#endif /* __QCAM_MAIN_WINDOW__ */
