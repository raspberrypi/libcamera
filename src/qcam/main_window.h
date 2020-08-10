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
#include <libcamera/controls.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>

#include "../cam/stream_options.h"
#include "viewfinder.h"

using namespace libcamera;

class QAction;
class QComboBox;

class HotplugEvent;

enum {
	OptCamera = 'c',
	OptHelp = 'h',
	OptStream = 's',
};

class CaptureRequest
{
public:
	CaptureRequest()
	{
	}

	CaptureRequest(const Request::BufferMap &buffers,
		       const ControlList &metadata)
		: buffers_(buffers), metadata_(metadata)
	{
	}

	Request::BufferMap buffers_;
	ControlList metadata_;
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
	void captureRaw();
	void processRaw(FrameBuffer *buffer, const ControlList &metadata);

	void queueRequest(FrameBuffer *buffer);

private:
	int createToolbars();

	std::string chooseCamera();
	int openCamera();

	int startCapture();
	void stopCapture();

	void addCamera(std::shared_ptr<Camera> camera);
	void removeCamera(std::shared_ptr<Camera> camera);

	void requestComplete(Request *request);
	void processCapture();
	void processHotplug(HotplugEvent *e);
	void processViewfinder(FrameBuffer *buffer);

	/* UI elements */
	QToolBar *toolbar_;
	QAction *startStopAction_;
	QComboBox *cameraCombo_;
	QAction *saveRaw_;
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
	bool captureRaw_;
	Stream *vfStream_;
	Stream *rawStream_;
	std::map<const Stream *, QQueue<FrameBuffer *>> freeBuffers_;
	QQueue<CaptureRequest> doneQueue_;
	QMutex mutex_; /* Protects freeBuffers_ and doneQueue_ */

	uint64_t lastBufferTime_;
	QElapsedTimer frameRateInterval_;
	uint32_t previousFrames_;
	uint32_t framesCaptured_;
};

#endif /* __QCAM_MAIN_WINDOW__ */
