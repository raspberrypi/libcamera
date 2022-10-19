/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.cpp - qcam - Main application window
 */

#include "main_window.h"

#include <assert.h>
#include <iomanip>
#include <string>

#include <libcamera/camera_manager.h>
#include <libcamera/version.h>

#include <QCoreApplication>
#include <QFileDialog>
#include <QImage>
#include <QImageWriter>
#include <QMutexLocker>
#include <QStandardPaths>
#include <QStringList>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QtDebug>

#include "../common/dng_writer.h"
#include "../common/image.h"

#include "cam_select_dialog.h"
#ifndef QT_NO_OPENGL
#include "viewfinder_gl.h"
#endif
#include "viewfinder_qt.h"

using namespace libcamera;

#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
/*
 * Qt::fixed was introduced in v5.14, and ::fixed deprecated in v5.15. Allow
 * usage of Qt::fixed unconditionally.
 */
namespace Qt {
constexpr auto fixed = ::fixed;
} /* namespace Qt */
#endif

/**
 * \brief Custom QEvent to signal capture completion
 */
class CaptureEvent : public QEvent
{
public:
	CaptureEvent()
		: QEvent(type())
	{
	}

	static Type type()
	{
		static int type = QEvent::registerEventType();
		return static_cast<Type>(type);
	}
};

/**
 * \brief Custom QEvent to signal hotplug or unplug
 */
class HotplugEvent : public QEvent
{
public:
	enum PlugEvent {
		HotPlug,
		HotUnplug
	};

	HotplugEvent(std::shared_ptr<Camera> camera, PlugEvent event)
		: QEvent(type()), camera_(std::move(camera)), plugEvent_(event)
	{
	}

	static Type type()
	{
		static int type = QEvent::registerEventType();
		return static_cast<Type>(type);
	}

	PlugEvent hotplugEvent() const { return plugEvent_; }
	Camera *camera() const { return camera_.get(); }

private:
	std::shared_ptr<Camera> camera_;
	PlugEvent plugEvent_;
};

MainWindow::MainWindow(CameraManager *cm, const OptionsParser::Options &options)
	: saveRaw_(nullptr), options_(options), cm_(cm), allocator_(nullptr),
	  isCapturing_(false), captureRaw_(false)
{
	int ret;

	/*
	 * Initialize the UI: Create the toolbar, set the window title and
	 * create the viewfinder widget.
	 */
	createToolbars();

	title_ = "QCam " + QString::fromStdString(CameraManager::version());
	setWindowTitle(title_);
	connect(&titleTimer_, SIGNAL(timeout()), this, SLOT(updateTitle()));

	/* Renderer type Qt or GLES, select Qt by default. */
	std::string renderType = "qt";
	if (options_.isSet(OptRenderer))
		renderType = options_[OptRenderer].toString();

	if (renderType == "qt") {
		ViewFinderQt *viewfinder = new ViewFinderQt(this);
		connect(viewfinder, &ViewFinderQt::renderComplete,
			this, &MainWindow::renderComplete);
		viewfinder_ = viewfinder;
		setCentralWidget(viewfinder);
#ifndef QT_NO_OPENGL
	} else if (renderType == "gles") {
		ViewFinderGL *viewfinder = new ViewFinderGL(this);
		connect(viewfinder, &ViewFinderGL::renderComplete,
			this, &MainWindow::renderComplete);
		viewfinder_ = viewfinder;
		setCentralWidget(viewfinder);
#endif
	} else {
		qWarning() << "Invalid render type"
			   << QString::fromStdString(renderType);
		quit();
		return;
	}

	adjustSize();

	/* Hotplug/unplug support */
	cm_->cameraAdded.connect(this, &MainWindow::addCamera);
	cm_->cameraRemoved.connect(this, &MainWindow::removeCamera);

	cameraSelectorDialog_ = new CameraSelectorDialog(cm_, this);

	/* Open the camera and start capture. */
	ret = openCamera();
	if (ret < 0) {
		quit();
		return;
	}

	startStopAction_->setChecked(true);
}

MainWindow::~MainWindow()
{
	if (camera_) {
		stopCapture();
		camera_->release();
		camera_.reset();
	}
}

bool MainWindow::event(QEvent *e)
{
	if (e->type() == CaptureEvent::type()) {
		processCapture();
		return true;
	} else if (e->type() == HotplugEvent::type()) {
		processHotplug(static_cast<HotplugEvent *>(e));
		return true;
	}

	return QMainWindow::event(e);
}

int MainWindow::createToolbars()
{
	QAction *action;

	toolbar_ = addToolBar("Main");

	/* Disable right click context menu. */
	toolbar_->setContextMenuPolicy(Qt::PreventContextMenu);

	/* Quit action. */
	action = toolbar_->addAction(QIcon::fromTheme("application-exit",
						      QIcon(":x-circle.svg")),
				     "Quit");
	action->setShortcut(Qt::CTRL | Qt::Key_Q);
	connect(action, &QAction::triggered, this, &MainWindow::quit);

	/* Camera selector. */
	cameraSelectButton_ = new QPushButton;
	connect(cameraSelectButton_, &QPushButton::clicked,
		this, &MainWindow::switchCamera);

	toolbar_->addWidget(cameraSelectButton_);

	toolbar_->addSeparator();

	/* Start/Stop action. */
	iconPlay_ = QIcon::fromTheme("media-playback-start",
				     QIcon(":play-circle.svg"));
	iconStop_ = QIcon::fromTheme("media-playback-stop",
				     QIcon(":stop-circle.svg"));

	action = toolbar_->addAction(iconPlay_, "Start Capture");
	action->setCheckable(true);
	action->setShortcut(Qt::Key_Space);
	connect(action, &QAction::toggled, this, &MainWindow::toggleCapture);
	startStopAction_ = action;

	/* Save As... action. */
	action = toolbar_->addAction(QIcon::fromTheme("document-save-as",
						      QIcon(":save.svg")),
				     "Save As...");
	action->setShortcut(QKeySequence::SaveAs);
	connect(action, &QAction::triggered, this, &MainWindow::saveImageAs);

#ifdef HAVE_DNG
	/* Save Raw action. */
	action = toolbar_->addAction(QIcon::fromTheme("camera-photo",
						      QIcon(":aperture.svg")),
				     "Save Raw");
	action->setEnabled(false);
	connect(action, &QAction::triggered, this, &MainWindow::captureRaw);
	saveRaw_ = action;
#endif

	return 0;
}

void MainWindow::quit()
{
	QTimer::singleShot(0, QCoreApplication::instance(),
			   &QCoreApplication::quit);
}

void MainWindow::updateTitle()
{
	/* Calculate the average frame rate over the last period. */
	unsigned int duration = frameRateInterval_.elapsed();
	unsigned int frames = framesCaptured_ - previousFrames_;
	double fps = frames * 1000.0 / duration;

	/* Restart counters. */
	frameRateInterval_.start();
	previousFrames_ = framesCaptured_;

	setWindowTitle(title_ + " : " + QString::number(fps, 'f', 2) + " fps");
}

/* -----------------------------------------------------------------------------
 * Camera Selection
 */

void MainWindow::switchCamera()
{
	/* Get and acquire the new camera. */
	std::string newCameraId = chooseCamera();

	if (newCameraId.empty())
		return;

	if (camera_ && newCameraId == camera_->id())
		return;

	const std::shared_ptr<Camera> &cam = cm_->get(newCameraId);

	if (cam->acquire()) {
		qInfo() << "Failed to acquire camera" << cam->id().c_str();
		return;
	}

	qInfo() << "Switching to camera" << cam->id().c_str();

	/*
	 * Stop the capture session, release the current camera, replace it with
	 * the new camera and start a new capture session.
	 */
	startStopAction_->setChecked(false);

	if (camera_)
		camera_->release();

	camera_ = cam;

	startStopAction_->setChecked(true);

	/* Display the current cameraId in the toolbar .*/
	cameraSelectButton_->setText(QString::fromStdString(newCameraId));
}

std::string MainWindow::chooseCamera()
{
	if (cameraSelectorDialog_->exec() != QDialog::Accepted)
		return std::string();

	return cameraSelectorDialog_->getCameraId();
}

int MainWindow::openCamera()
{
	std::string cameraName;

	/*
	 * Use the camera specified on the command line, if any, or display the
	 * camera selection dialog box otherwise.
	 */
	if (options_.isSet(OptCamera))
		cameraName = static_cast<std::string>(options_[OptCamera]);
	else
		cameraName = chooseCamera();

	if (cameraName == "")
		return -EINVAL;

	/* Get and acquire the camera. */
	camera_ = cm_->get(cameraName);
	if (!camera_) {
		qInfo() << "Camera" << cameraName.c_str() << "not found";
		return -ENODEV;
	}

	if (camera_->acquire()) {
		qInfo() << "Failed to acquire camera";
		camera_.reset();
		return -EBUSY;
	}

	/* Set the camera switch button with the currently selected Camera id. */
	cameraSelectButton_->setText(QString::fromStdString(cameraName));

	return 0;
}

/* -----------------------------------------------------------------------------
 * Capture Start & Stop
 */

void MainWindow::toggleCapture(bool start)
{
	if (start) {
		startCapture();
		startStopAction_->setIcon(iconStop_);
		startStopAction_->setText("Stop Capture");
	} else {
		stopCapture();
		startStopAction_->setIcon(iconPlay_);
		startStopAction_->setText("Start Capture");
	}
}

/**
 * \brief Start capture with the current camera
 *
 * This function shall not be called directly, use toggleCapture() instead.
 */
int MainWindow::startCapture()
{
	StreamRoles roles = StreamKeyValueParser::roles(options_[OptStream]);
	int ret;

	/* Verify roles are supported. */
	switch (roles.size()) {
	case 1:
		if (roles[0] != StreamRole::Viewfinder) {
			qWarning() << "Only viewfinder supported for single stream";
			return -EINVAL;
		}
		break;
	case 2:
		if (roles[0] != StreamRole::Viewfinder ||
		    roles[1] != StreamRole::Raw) {
			qWarning() << "Only viewfinder + raw supported for dual streams";
			return -EINVAL;
		}
		break;
	default:
		if (roles.size() != 1) {
			qWarning() << "Unsupported stream configuration";
			return -EINVAL;
		}
		break;
	}

	/* Configure the camera. */
	config_ = camera_->generateConfiguration(roles);
	if (!config_) {
		qWarning() << "Failed to generate configuration from roles";
		return -EINVAL;
	}

	StreamConfiguration &vfConfig = config_->at(0);

	/* Use a format supported by the viewfinder if available. */
	std::vector<PixelFormat> formats = vfConfig.formats().pixelformats();
	for (const PixelFormat &format : viewfinder_->nativeFormats()) {
		auto match = std::find_if(formats.begin(), formats.end(),
					  [&](const PixelFormat &f) {
						  return f == format;
					  });
		if (match != formats.end()) {
			vfConfig.pixelFormat = format;
			break;
		}
	}

	/* Allow user to override configuration. */
	if (StreamKeyValueParser::updateConfiguration(config_.get(),
						      options_[OptStream])) {
		qWarning() << "Failed to update configuration";
		return -EINVAL;
	}

	CameraConfiguration::Status validation = config_->validate();
	if (validation == CameraConfiguration::Invalid) {
		qWarning() << "Failed to create valid camera configuration";
		return -EINVAL;
	}

	if (validation == CameraConfiguration::Adjusted)
		qInfo() << "Stream configuration adjusted to "
			<< vfConfig.toString().c_str();

	ret = camera_->configure(config_.get());
	if (ret < 0) {
		qInfo() << "Failed to configure camera";
		return ret;
	}

	/* Store stream allocation. */
	vfStream_ = config_->at(0).stream();
	if (config_->size() == 2)
		rawStream_ = config_->at(1).stream();
	else
		rawStream_ = nullptr;

	/*
	 * Configure the viewfinder. If no color space is reported, default to
	 * sYCC.
	 */
	ret = viewfinder_->setFormat(vfConfig.pixelFormat,
				     QSize(vfConfig.size.width, vfConfig.size.height),
				     vfConfig.colorSpace.value_or(ColorSpace::Sycc),
				     vfConfig.stride);
	if (ret < 0) {
		qInfo() << "Failed to set viewfinder format";
		return ret;
	}

	adjustSize();

	/* Configure the raw capture button. */
	if (saveRaw_)
		saveRaw_->setEnabled(config_->size() == 2);

	/* Allocate and map buffers. */
	allocator_ = new FrameBufferAllocator(camera_);
	for (StreamConfiguration &config : *config_) {
		Stream *stream = config.stream();

		ret = allocator_->allocate(stream);
		if (ret < 0) {
			qWarning() << "Failed to allocate capture buffers";
			goto error;
		}

		for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
			/* Map memory buffers and cache the mappings. */
			std::unique_ptr<Image> image =
				Image::fromFrameBuffer(buffer.get(), Image::MapMode::ReadOnly);
			assert(image != nullptr);
			mappedBuffers_[buffer.get()] = std::move(image);

			/* Store buffers on the free list. */
			freeBuffers_[stream].enqueue(buffer.get());
		}
	}

	/* Create requests and fill them with buffers from the viewfinder. */
	while (!freeBuffers_[vfStream_].isEmpty()) {
		FrameBuffer *buffer = freeBuffers_[vfStream_].dequeue();

		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request) {
			qWarning() << "Can't create request";
			ret = -ENOMEM;
			goto error;
		}

		ret = request->addBuffer(vfStream_, buffer);
		if (ret < 0) {
			qWarning() << "Can't set buffer for request";
			goto error;
		}

		requests_.push_back(std::move(request));
	}

	/* Start the title timer and the camera. */
	titleTimer_.start(2000);
	frameRateInterval_.start();
	previousFrames_ = 0;
	framesCaptured_ = 0;
	lastBufferTime_ = 0;

	ret = camera_->start();
	if (ret) {
		qInfo() << "Failed to start capture";
		goto error;
	}

	camera_->requestCompleted.connect(this, &MainWindow::requestComplete);

	/* Queue all requests. */
	for (std::unique_ptr<Request> &request : requests_) {
		ret = queueRequest(request.get());
		if (ret < 0) {
			qWarning() << "Can't queue request";
			goto error_disconnect;
		}
	}

	isCapturing_ = true;

	return 0;

error_disconnect:
	camera_->requestCompleted.disconnect(this);
	camera_->stop();

error:
	requests_.clear();

	mappedBuffers_.clear();

	freeBuffers_.clear();

	delete allocator_;
	allocator_ = nullptr;

	return ret;
}

/**
 * \brief Stop ongoing capture
 *
 * This function may be called directly when tearing down the MainWindow. Use
 * toggleCapture() instead in all other cases.
 */
void MainWindow::stopCapture()
{
	if (!isCapturing_)
		return;

	viewfinder_->stop();
	if (saveRaw_)
		saveRaw_->setEnabled(false);
	captureRaw_ = false;

	int ret = camera_->stop();
	if (ret)
		qInfo() << "Failed to stop capture";

	camera_->requestCompleted.disconnect(this);

	mappedBuffers_.clear();

	requests_.clear();
	freeQueue_.clear();

	delete allocator_;

	isCapturing_ = false;

	config_.reset();

	/*
	 * A CaptureEvent may have been posted before we stopped the camera,
	 * but not processed yet. Clear the queue of done buffers to avoid
	 * racing with the event handler.
	 */
	freeBuffers_.clear();
	doneQueue_.clear();

	titleTimer_.stop();
	setWindowTitle(title_);
}

/* -----------------------------------------------------------------------------
 * Camera hotplugging support
 */

void MainWindow::processHotplug(HotplugEvent *e)
{
	Camera *camera = e->camera();
	QString cameraId = QString::fromStdString(camera->id());
	HotplugEvent::PlugEvent event = e->hotplugEvent();

	if (event == HotplugEvent::HotPlug) {
		cameraSelectorDialog_->addCamera(cameraId);
	} else if (event == HotplugEvent::HotUnplug) {
		/* Check if the currently-streaming camera is removed. */
		if (camera == camera_.get()) {
			toggleCapture(false);
			camera_->release();
			camera_.reset();
		}

		cameraSelectorDialog_->removeCamera(cameraId);
	}
}

void MainWindow::addCamera(std::shared_ptr<Camera> camera)
{
	qInfo() << "Adding new camera:" << camera->id().c_str();
	QCoreApplication::postEvent(this,
				    new HotplugEvent(std::move(camera),
						     HotplugEvent::HotPlug));
}

void MainWindow::removeCamera(std::shared_ptr<Camera> camera)
{
	qInfo() << "Removing camera:" << camera->id().c_str();
	QCoreApplication::postEvent(this,
				    new HotplugEvent(std::move(camera),
						     HotplugEvent::HotUnplug));
}

/* -----------------------------------------------------------------------------
 * Image Save
 */

void MainWindow::saveImageAs()
{
	QImage image = viewfinder_->getCurrentImage();
	QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);

	QString filename = QFileDialog::getSaveFileName(this, "Save Image", defaultPath,
							"Image Files (*.png *.jpg *.jpeg)");
	if (filename.isEmpty())
		return;

	QImageWriter writer(filename);
	writer.setQuality(95);
	writer.write(image);
}

void MainWindow::captureRaw()
{
	captureRaw_ = true;
}

void MainWindow::processRaw(FrameBuffer *buffer,
			    [[maybe_unused]] const ControlList &metadata)
{
#ifdef HAVE_DNG
	QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
	QString filename = QFileDialog::getSaveFileName(this, "Save DNG", defaultPath,
							"DNG Files (*.dng)");

	if (!filename.isEmpty()) {
		uint8_t *memory = mappedBuffers_[buffer]->data(0).data();
		DNGWriter::write(filename.toStdString().c_str(), camera_.get(),
				 rawStream_->configuration(), metadata, buffer,
				 memory);
	}
#endif

	{
		QMutexLocker locker(&mutex_);
		freeBuffers_[rawStream_].enqueue(buffer);
	}
}

/* -----------------------------------------------------------------------------
 * Request Completion Handling
 */

void MainWindow::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	/*
	 * We're running in the libcamera thread context, expensive operations
	 * are not allowed. Add the buffer to the done queue and post a
	 * CaptureEvent for the application thread to handle.
	 */
	{
		QMutexLocker locker(&mutex_);
		doneQueue_.enqueue(request);
	}

	QCoreApplication::postEvent(this, new CaptureEvent);
}

void MainWindow::processCapture()
{
	/*
	 * Retrieve the next buffer from the done queue. The queue may be empty
	 * if stopCapture() has been called while a CaptureEvent was posted but
	 * not processed yet. Return immediately in that case.
	 */
	Request *request;
	{
		QMutexLocker locker(&mutex_);
		if (doneQueue_.isEmpty())
			return;

		request = doneQueue_.dequeue();
	}

	/* Process buffers. */
	if (request->buffers().count(vfStream_))
		processViewfinder(request->buffers().at(vfStream_));

	if (request->buffers().count(rawStream_))
		processRaw(request->buffers().at(rawStream_), request->metadata());

	request->reuse();
	QMutexLocker locker(&mutex_);
	freeQueue_.enqueue(request);
}

void MainWindow::processViewfinder(FrameBuffer *buffer)
{
	framesCaptured_++;

	const FrameMetadata &metadata = buffer->metadata();

	double fps = metadata.timestamp - lastBufferTime_;
	fps = lastBufferTime_ && fps ? 1000000000.0 / fps : 0.0;
	lastBufferTime_ = metadata.timestamp;

	QStringList bytesused;
	for (const FrameMetadata::Plane &plane : metadata.planes())
		bytesused << QString::number(plane.bytesused);

	qDebug().noquote()
		<< QString("seq: %1").arg(metadata.sequence, 6, 10, QLatin1Char('0'))
		<< "bytesused: {" << bytesused.join(", ")
		<< "} timestamp:" << metadata.timestamp
		<< "fps:" << Qt::fixed << qSetRealNumberPrecision(2) << fps;

	/* Render the frame on the viewfinder. */
	viewfinder_->render(buffer, mappedBuffers_[buffer].get());
}

void MainWindow::renderComplete(FrameBuffer *buffer)
{
	Request *request;
	{
		QMutexLocker locker(&mutex_);
		if (freeQueue_.isEmpty())
			return;

		request = freeQueue_.dequeue();
	}

	request->addBuffer(vfStream_, buffer);

	if (captureRaw_) {
		FrameBuffer *rawBuffer = nullptr;

		{
			QMutexLocker locker(&mutex_);
			if (!freeBuffers_[rawStream_].isEmpty())
				rawBuffer = freeBuffers_[rawStream_].dequeue();
		}

		if (rawBuffer) {
			request->addBuffer(rawStream_, rawBuffer);
			captureRaw_ = false;
		} else {
			qWarning() << "No free buffer available for RAW capture";
		}
	}
	queueRequest(request);
}

int MainWindow::queueRequest(Request *request)
{
	return camera_->queueRequest(request);
}
