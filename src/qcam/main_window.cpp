/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.cpp - qcam - Main application window
 */

#include "main_window.h"

#include <iomanip>
#include <iostream>
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

#include <libcamera/camera_manager.h>
#include <libcamera/version.h>

#include "viewfinder.h"

using namespace libcamera;

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

	createToolbars();

	title_ = "QCam " + QString::fromStdString(CameraManager::version());
	setWindowTitle(title_);
	connect(&titleTimer_, SIGNAL(timeout()), this, SLOT(updateTitle()));

	viewfinder_ = new ViewFinder(this);
	setCentralWidget(viewfinder_);
	adjustSize();

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

	action = toolbar_->addAction(QIcon::fromTheme("application-exit",
						      QIcon(":x-circle.svg")),
				     "Quit");
	action->setShortcut(Qt::CTRL | Qt::Key_Q);
	connect(action, &QAction::triggered, this, &MainWindow::quit);

	/* Camera selection. */
	QComboBox *cameraCombo = new QComboBox();
	connect(cameraCombo, QOverload<int>::of(&QComboBox::activated),
		this, &MainWindow::switchCamera);

	for (const std::shared_ptr<Camera> &cam : cm_->cameras())
		cameraCombo->addItem(QString::fromStdString(cam->name()));

	toolbar_->addWidget(cameraCombo);

	toolbar_->addSeparator();

	iconPlay_ = QIcon::fromTheme("media-playback-start",
				     QIcon(":play-circle.svg"));
	iconStop_ = QIcon::fromTheme("media-playback-stop",
				     QIcon(":stop-circle.svg"));

	action = toolbar_->addAction(iconPlay_, "Start Capture");
	action->setCheckable(true);
	action->setShortcut(Qt::Key_Space);
	connect(action, &QAction::toggled, this, &MainWindow::toggleCapture);
	startStopAction_ = action;

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
	unsigned int duration = frameRateInterval_.elapsed();
	unsigned int frames = framesCaptured_ - previousFrames_;
	double fps = frames * 1000.0 / duration;

	/* Restart counters. */
	frameRateInterval_.start();
	previousFrames_ = framesCaptured_;

	setWindowTitle(title_ + " : " + QString::number(fps, 'f', 2) + " fps");
}

void MainWindow::switchCamera(int index)
{
	const auto &cameras = cm_->cameras();
	if (static_cast<unsigned int>(index) >= cameras.size())
		return;

	const std::shared_ptr<Camera> &cam = cameras[index];

	if (cam->acquire()) {
		std::cout << "Failed to acquire camera " << cam->name() << std::endl;
		return;
	}

	std::cout << "Switching to camera " << cam->name() << std::endl;

	startStopAction_->setChecked(false);

	camera_->release();
	camera_ = cam;

	startStopAction_->setChecked(true);
}

std::string MainWindow::chooseCamera()
{
	QStringList cameras;
	bool result;

	if (cm_->cameras().size() == 1)
		return cm_->cameras()[0]->name();

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

	if (options_.isSet(OptCamera))
		cameraName = static_cast<std::string>(options_[OptCamera]);
	else
		cameraName = chooseCamera();

	if (cameraName == "")
		return -EINVAL;

	camera_ = cm_->get(cameraName);
	if (!camera_) {
		std::cout << "Camera " << cameraName << " not found"
			  << std::endl;
		return -ENODEV;
	}

	if (camera_->acquire()) {
		std::cout << "Failed to acquire camera" << std::endl;
		camera_.reset();
		return -EBUSY;
	}

	std::cout << "Using camera " << camera_->name() << std::endl;

	return 0;
}

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

int MainWindow::startCapture()
{
	int ret;

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

	CameraConfiguration::Status validation = config_->validate();
	if (validation == CameraConfiguration::Invalid) {
		std::cerr << "Failed to create valid camera configuration";
		return -EINVAL;
	}

	if (validation == CameraConfiguration::Adjusted) {
		std::cout << "Stream size adjusted to "
			  << cfg.size.toString() << std::endl;
	}

	ret = camera_->configure(config_.get());
	if (ret < 0) {
		std::cout << "Failed to configure camera" << std::endl;
		return ret;
	}

	Stream *stream = cfg.stream();
	ret = viewfinder_->setFormat(cfg.pixelFormat,
				     QSize(cfg.size.width, cfg.size.height));
	if (ret < 0) {
		std::cout << "Failed to set viewfinder format" << std::endl;
		return ret;
	}

	adjustSize();

	allocator_ = new FrameBufferAllocator(camera_);
	ret = allocator_->allocate(stream);
	if (ret < 0) {
		std::cerr << "Failed to allocate capture buffers" << std::endl;
		return ret;
	}

	std::vector<Request *> requests;
	for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
		Request *request = camera_->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			ret = -ENOMEM;
			goto error;
		}

		ret = request->addBuffer(stream, buffer.get());
		if (ret < 0) {
			std::cerr << "Can't set buffer for request" << std::endl;
			goto error;
		}

		requests.push_back(request);

		/* Map memory buffers and cache the mappings. */
		const FrameBuffer::Plane &plane = buffer->planes().front();
		void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
				    plane.fd.fd(), 0);
		mappedBuffers_[plane.fd.fd()] =
			std::make_pair(memory, plane.length);
	}

	titleTimer_.start(2000);
	frameRateInterval_.start();
	previousFrames_ = 0;
	framesCaptured_ = 0;
	lastBufferTime_ = 0;

	ret = camera_->start();
	if (ret) {
		std::cout << "Failed to start capture" << std::endl;
		goto error;
	}

	camera_->requestCompleted.connect(this, &MainWindow::requestComplete);

	for (Request *request : requests) {
		ret = camera_->queueRequest(request);
		if (ret < 0) {
			std::cerr << "Can't queue request" << std::endl;
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
		void *memory = iter.second.first;
		unsigned int length = iter.second.second;
		munmap(memory, length);
	}
	mappedBuffers_.clear();

	delete allocator_;
	allocator_ = nullptr;

	return ret;
}

void MainWindow::stopCapture()
{
	if (!isCapturing_)
		return;

	int ret = camera_->stop();
	if (ret)
		std::cout << "Failed to stop capture" << std::endl;

	camera_->requestCompleted.disconnect(this, &MainWindow::requestComplete);

	for (auto &iter : mappedBuffers_) {
		void *memory = iter.second.first;
		unsigned int length = iter.second.second;
		munmap(memory, length);
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

void MainWindow::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

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

	std::cout << "seq: " << std::setw(6) << std::setfill('0') << metadata.sequence
		  << " bytesused: " << metadata.planes[0].bytesused
		  << " timestamp: " << metadata.timestamp
		  << " fps: " << std::fixed << std::setprecision(2) << fps
		  << std::endl;

	display(buffer);

	queueRequest(buffer);
}

int MainWindow::display(FrameBuffer *buffer)
{
	if (buffer->planes().size() != 1)
		return -EINVAL;

	const FrameBuffer::Plane &plane = buffer->planes().front();
	void *memory = mappedBuffers_[plane.fd.fd()].first;
	unsigned char *raw = static_cast<unsigned char *>(memory);
	viewfinder_->display(raw, buffer->metadata().planes[0].bytesused);

	return 0;
}

void MainWindow::queueRequest(FrameBuffer *buffer)
{
	Request *request = camera_->createRequest();
	if (!request) {
		std::cerr << "Can't create request" << std::endl;
		return;
	}

	Stream *stream = config_->at(0).stream();
	request->addBuffer(stream, buffer);

	camera_->queueRequest(request);
}
