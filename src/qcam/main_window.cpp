/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.cpp - qcam - Main application window
 */

#include "main_window.h"

#include <iomanip>
#include <string>
#include <sys/mman.h>

#include <QComboBox>
#include <QCoreApplication>
#include <QFileDialog>
#include <QImage>
#include <QImageWriter>
#include <QInputDialog>
#include <QMutexLocker>
#include <QStandardPaths>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QtDebug>

#include <libcamera/camera_manager.h>
#include <libcamera/version.h>

using namespace libcamera;

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

MainWindow::MainWindow(CameraManager *cm, const OptionsParser::Options &options)
	: options_(options), cm_(cm), allocator_(nullptr), isCapturing_(false)
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

	viewfinder_ = new ViewFinder(this);
	connect(viewfinder_, &ViewFinder::renderComplete,
		this, &MainWindow::queueRequest);
	setCentralWidget(viewfinder_);
	adjustSize();

	/* Open the camera and start capture. */
	ret = openCamera();
	if (ret < 0)
		quit();

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
	QComboBox *cameraCombo = new QComboBox();
	connect(cameraCombo, QOverload<int>::of(&QComboBox::activated),
		this, &MainWindow::switchCamera);

	for (const std::shared_ptr<Camera> &cam : cm_->cameras())
		cameraCombo->addItem(QString::fromStdString(cam->name()));

	toolbar_->addWidget(cameraCombo);

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

void MainWindow::switchCamera(int index)
{
	/* Get and acquire the new camera. */
	const auto &cameras = cm_->cameras();
	if (static_cast<unsigned int>(index) >= cameras.size())
		return;

	const std::shared_ptr<Camera> &cam = cameras[index];

	if (cam->acquire()) {
		qInfo() << "Failed to acquire camera" << cam->name().c_str();
		return;
	}

	qInfo() << "Switching to camera" << cam->name().c_str();

	/*
	 * Stop the capture session, release the current camera, replace it with
	 * the new camera and start a new capture session.
	 */
	startStopAction_->setChecked(false);

	camera_->release();
	camera_ = cam;

	startStopAction_->setChecked(true);
}

std::string MainWindow::chooseCamera()
{
	QStringList cameras;
	bool result;

	/* If only one camera is available, use it automatically. */
	if (cm_->cameras().size() == 1)
		return cm_->cameras()[0]->name();

	/* Present a dialog box to pick a camera. */
	for (const std::shared_ptr<Camera> &cam : cm_->cameras())
		cameras.append(QString::fromStdString(cam->name()));

	QString name = QInputDialog::getItem(this, "Select Camera",
					     "Camera:", cameras, 0,
					     false, &result);
	if (!result)
		return std::string();

	return name.toStdString();
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
	int ret;

	/* Configure the camera. */
	config_ = camera_->generateConfiguration({ StreamRole::Viewfinder });

	StreamConfiguration &cfg = config_->at(0);

	if (options_.isSet(OptSize)) {
		const std::vector<OptionValue> &sizeOptions =
			options_[OptSize].toArray();

		/* Set desired stream size if requested. */
		for (const auto &value : sizeOptions) {
			KeyValueParser::Options opt = value.toKeyValues();

			if (opt.isSet("width"))
				cfg.size.width = opt["width"];

			if (opt.isSet("height"))
				cfg.size.height = opt["height"];
		}
	}

	/* Use a format supported by the viewfinder if available. */
	std::vector<PixelFormat> formats = cfg.formats().pixelformats();
	for (const PixelFormat &format : viewfinder_->nativeFormats()) {
		auto match = std::find_if(formats.begin(), formats.end(),
					  [&](const PixelFormat &f) {
						  return f == format;
					  });
		if (match != formats.end()) {
			cfg.pixelFormat = format;
			break;
		}
	}

	CameraConfiguration::Status validation = config_->validate();
	if (validation == CameraConfiguration::Invalid) {
		qWarning() << "Failed to create valid camera configuration";
		return -EINVAL;
	}

	if (validation == CameraConfiguration::Adjusted)
		qInfo() << "Stream configuration adjusted to "
			<< cfg.toString().c_str();

	ret = camera_->configure(config_.get());
	if (ret < 0) {
		qInfo() << "Failed to configure camera";
		return ret;
	}

	/* Configure the viewfinder. */
	Stream *stream = cfg.stream();
	ret = viewfinder_->setFormat(cfg.pixelFormat,
				     QSize(cfg.size.width, cfg.size.height));
	if (ret < 0) {
		qInfo() << "Failed to set viewfinder format";
		return ret;
	}

	adjustSize();

	/* Allocate buffers and requests. */
	allocator_ = new FrameBufferAllocator(camera_);
	ret = allocator_->allocate(stream);
	if (ret < 0) {
		qWarning() << "Failed to allocate capture buffers";
		return ret;
	}

	std::vector<Request *> requests;
	for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
		Request *request = camera_->createRequest();
		if (!request) {
			qWarning() << "Can't create request";
			ret = -ENOMEM;
			goto error;
		}

		ret = request->addBuffer(stream, buffer.get());
		if (ret < 0) {
			qWarning() << "Can't set buffer for request";
			goto error;
		}

		requests.push_back(request);

		/* Map memory buffers and cache the mappings. */
		const FrameBuffer::Plane &plane = buffer->planes().front();
		void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
				    plane.fd.fd(), 0);
		mappedBuffers_[buffer.get()] = { memory, plane.length };
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
	for (Request *request : requests) {
		ret = camera_->queueRequest(request);
		if (ret < 0) {
			qWarning() << "Can't queue request";
			goto error_disconnect;
		}
	}

	isCapturing_ = true;

	return 0;

error_disconnect:
	camera_->requestCompleted.disconnect(this, &MainWindow::requestComplete);
	camera_->stop();

error:
	for (Request *request : requests)
		delete request;

	for (auto &iter : mappedBuffers_) {
		const MappedBuffer &buffer = iter.second;
		munmap(buffer.memory, buffer.size);
	}
	mappedBuffers_.clear();

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

	int ret = camera_->stop();
	if (ret)
		qInfo() << "Failed to stop capture";

	camera_->requestCompleted.disconnect(this, &MainWindow::requestComplete);

	for (auto &iter : mappedBuffers_) {
		const MappedBuffer &buffer = iter.second;
		munmap(buffer.memory, buffer.size);
	}
	mappedBuffers_.clear();

	delete allocator_;

	isCapturing_ = false;

	config_.reset();

	/*
	 * A CaptureEvent may have been posted before we stopped the camera,
	 * but not processed yet. Clear the queue of done buffers to avoid
	 * racing with the event handler.
	 */
	doneQueue_.clear();

	titleTimer_.stop();
	setWindowTitle(title_);
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
	const std::map<Stream *, FrameBuffer *> &buffers = request->buffers();
	FrameBuffer *buffer = buffers.begin()->second;

	{
		QMutexLocker locker(&mutex_);
		doneQueue_.enqueue(buffer);
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
	FrameBuffer *buffer;

	{
		QMutexLocker locker(&mutex_);
		if (doneQueue_.isEmpty())
			return;

		buffer = doneQueue_.dequeue();
	}

	framesCaptured_++;

	const FrameMetadata &metadata = buffer->metadata();

	double fps = metadata.timestamp - lastBufferTime_;
	fps = lastBufferTime_ && fps ? 1000000000.0 / fps : 0.0;
	lastBufferTime_ = metadata.timestamp;

	qInfo() << "seq:" << qSetFieldWidth(6) << qSetPadChar('0')
		<< metadata.sequence << reset
		<< "bytesused:" << metadata.planes[0].bytesused
		<< "timestamp:" << metadata.timestamp
		<< "fps:" << fixed << qSetRealNumberPrecision(2) << fps;

	/* Render the frame on the viewfinder. */
	viewfinder_->render(buffer, &mappedBuffers_[buffer]);
}

void MainWindow::queueRequest(FrameBuffer *buffer)
{
	Request *request = camera_->createRequest();
	if (!request) {
		qWarning() << "Can't create request";
		return;
	}

	Stream *stream = config_->at(0).stream();
	request->addBuffer(stream, buffer);

	camera_->queueRequest(request);
}
