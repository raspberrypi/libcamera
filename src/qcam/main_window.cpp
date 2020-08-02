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

#include "dng_writer.h"

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

	viewfinder_ = new ViewFinder(this);
	connect(viewfinder_, &ViewFinder::renderComplete,
		this, &MainWindow::queueRequest);
	setCentralWidget(viewfinder_);
	adjustSize();

	/* Hotplug/unplug support */
	cm_->cameraAdded.connect(this, &MainWindow::addCamera);
	cm_->cameraRemoved.connect(this, &MainWindow::removeCamera);

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
	cameraCombo_ = new QComboBox();
	connect(cameraCombo_, QOverload<int>::of(&QComboBox::activated),
		this, &MainWindow::switchCamera);

	for (const std::shared_ptr<Camera> &cam : cm_->cameras())
		cameraCombo_->addItem(QString::fromStdString(cam->id()));

	toolbar_->addWidget(cameraCombo_);

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

void MainWindow::switchCamera(int index)
{
	/* Get and acquire the new camera. */
	const auto &cameras = cm_->cameras();
	if (static_cast<unsigned int>(index) >= cameras.size())
		return;

	const std::shared_ptr<Camera> &cam = cameras[index];

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
		return cm_->cameras()[0]->id();

	/* Present a dialog box to pick a camera. */
	for (const std::shared_ptr<Camera> &cam : cm_->cameras())
		cameras.append(QString::fromStdString(cam->id()));

	QString id = QInputDialog::getItem(this, "Select Camera",
					   "Camera:", cameras, 0,
					   false, &result);
	if (!result)
		return std::string();

	return id.toStdString();
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

	/* Set the combo-box entry with the currently selected Camera. */
	cameraCombo_->setCurrentText(QString::fromStdString(cameraName));

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
	std::vector<Request *> requests;
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
		    roles[1] != StreamRole::StillCaptureRaw) {
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

	/* Configure the viewfinder. */
	ret = viewfinder_->setFormat(vfConfig.pixelFormat,
				     QSize(vfConfig.size.width, vfConfig.size.height));
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
			const FrameBuffer::Plane &plane = buffer->planes().front();
			void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
					    plane.fd.fd(), 0);
			mappedBuffers_[buffer.get()] = { memory, plane.length };

			/* Store buffers on the free list. */
			freeBuffers_[stream].enqueue(buffer.get());
		}
	}

	/* Create requests and fill them with buffers from the viewfinder. */
	while (!freeBuffers_[vfStream_].isEmpty()) {
		FrameBuffer *buffer = freeBuffers_[vfStream_].dequeue();

		Request *request = camera_->createRequest();
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

		requests.push_back(request);
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
	HotplugEvent::PlugEvent event = e->hotplugEvent();

	if (event == HotplugEvent::HotPlug) {
		cameraCombo_->addItem(QString::fromStdString(camera->id()));
	} else if (event == HotplugEvent::HotUnplug) {
		/* Check if the currently-streaming camera is removed. */
		if (camera == camera_.get()) {
			toggleCapture(false);
			camera_->release();
			camera_.reset();
			cameraCombo_->setCurrentIndex(0);
		}

		int camIndex = cameraCombo_->findText(QString::fromStdString(camera->id()));
		cameraCombo_->removeItem(camIndex);
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

void MainWindow::processRaw(FrameBuffer *buffer, const ControlList &metadata)
{
#ifdef HAVE_DNG
	QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
	QString filename = QFileDialog::getSaveFileName(this, "Save DNG", defaultPath,
							"DNG Files (*.dng)");

	if (!filename.isEmpty()) {
		const MappedBuffer &mapped = mappedBuffers_[buffer];
		DNGWriter::write(filename.toStdString().c_str(), camera_.get(),
				 rawStream_->configuration(), metadata, buffer,
				 mapped.memory);
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
		doneQueue_.enqueue({ request->buffers(), request->metadata() });
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
	CaptureRequest request;

	{
		QMutexLocker locker(&mutex_);
		if (doneQueue_.isEmpty())
			return;

		request = doneQueue_.dequeue();
	}

	/* Process buffers. */
	if (request.buffers_.count(vfStream_))
		processViewfinder(request.buffers_[vfStream_]);

	if (request.buffers_.count(rawStream_))
		processRaw(request.buffers_[rawStream_], request.metadata_);
}

void MainWindow::processViewfinder(FrameBuffer *buffer)
{
	framesCaptured_++;

	const FrameMetadata &metadata = buffer->metadata();

	double fps = metadata.timestamp - lastBufferTime_;
	fps = lastBufferTime_ && fps ? 1000000000.0 / fps : 0.0;
	lastBufferTime_ = metadata.timestamp;

	qInfo().noquote()
		<< QString("seq: %1").arg(metadata.sequence, 6, 10, QLatin1Char('0'))
		<< "bytesused:" << metadata.planes[0].bytesused
		<< "timestamp:" << metadata.timestamp
		<< "fps:" << Qt::fixed << qSetRealNumberPrecision(2) << fps;

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

	request->addBuffer(vfStream_, buffer);

	if (captureRaw_) {
		FrameBuffer *buffer = nullptr;

		{
			QMutexLocker locker(&mutex_);
			if (!freeBuffers_[rawStream_].isEmpty())
				buffer = freeBuffers_[rawStream_].dequeue();
		}

		if (buffer) {
			request->addBuffer(rawStream_, buffer);
			captureRaw_ = false;
		} else {
			qWarning() << "No free buffer available for RAW capture";
		}
	}

	camera_->queueRequest(request);
}
