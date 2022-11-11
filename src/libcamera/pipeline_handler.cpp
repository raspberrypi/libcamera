/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * pipeline_handler.cpp - Pipeline handler infrastructure
 */

#include "libcamera/internal/pipeline_handler.h"

#include <chrono>
#include <sys/sysmacros.h>

#include <libcamera/base/log.h>
#include <libcamera/base/mutex.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/tracepoints.h"

/**
 * \file pipeline_handler.h
 * \brief Create pipelines and cameras from a set of media devices
 *
 * Each pipeline supported by libcamera needs to be backed by a pipeline
 * handler implementation that operate on a set of media devices. The pipeline
 * handler is responsible for matching the media devices it requires with the
 * devices present in the system, and once all those devices can be acquired,
 * create corresponding Camera instances.
 *
 * Every subclass of PipelineHandler shall be registered with libcamera using
 * the REGISTER_PIPELINE_HANDLER() macro.
 */

using namespace std::chrono_literals;

namespace libcamera {

LOG_DEFINE_CATEGORY(Pipeline)

/**
 * \class PipelineHandler
 * \brief Create and manage cameras based on a set of media devices
 *
 * The PipelineHandler matches the media devices provided by a DeviceEnumerator
 * with the pipelines it supports and creates corresponding Camera devices.
 *
 * Pipeline handler instances are reference-counted through std::shared_ptr<>.
 * They implement std::enable_shared_from_this<> in order to create new
 * std::shared_ptr<> in code paths originating from member functions of the
 * PipelineHandler class where only the 'this' pointer is available.
 */

/**
 * \brief Construct a PipelineHandler instance
 * \param[in] manager The camera manager
 *
 * In order to honour the std::enable_shared_from_this<> contract,
 * PipelineHandler instances shall never be constructed manually, but always
 * through the PipelineHandlerFactoryBase::create() function.
 */
PipelineHandler::PipelineHandler(CameraManager *manager)
	: manager_(manager), useCount_(0)
{
}

PipelineHandler::~PipelineHandler()
{
	for (std::shared_ptr<MediaDevice> media : mediaDevices_)
		media->release();
}

/**
 * \fn PipelineHandler::match(DeviceEnumerator *enumerator)
 * \brief Match media devices and create camera instances
 * \param[in] enumerator The enumerator providing all media devices found in the
 * system
 *
 * This function is the main entry point of the pipeline handler. It is called
 * by the camera manager with the \a enumerator passed as an argument. It shall
 * acquire from the \a enumerator all the media devices it needs for a single
 * pipeline, create one or multiple Camera instances and register them with the
 * camera manager.
 *
 * If all media devices needed by the pipeline handler are found, they must all
 * be acquired by a call to MediaDevice::acquire(). This function shall then
 * create the corresponding Camera instances, store them internally, and return
 * true. Otherwise it shall not acquire any media device (or shall release all
 * the media devices is has acquired by calling MediaDevice::release()) and
 * return false.
 *
 * If multiple instances of a pipeline are available in the system, the
 * PipelineHandler class will be instantiated once per instance, and its match()
 * function called for every instance. Each call shall acquire media devices for
 * one pipeline instance, until all compatible media devices are exhausted.
 *
 * If this function returns true, a new instance of the pipeline handler will
 * be created and its match() function called.
 *
 * \context This function is called from the CameraManager thread.
 *
 * \return true if media devices have been acquired and camera instances
 * created, or false otherwise
 */

/**
 * \brief Search and acquire a MediaDevice matching a device pattern
 * \param[in] enumerator Enumerator containing all media devices in the system
 * \param[in] dm Device match pattern
 *
 * Search the device \a enumerator for an available media device matching the
 * device match pattern \a dm. Matching media device that have previously been
 * acquired by MediaDevice::acquire() are not considered. If a match is found,
 * the media device is acquired and returned. The caller shall not release the
 * device explicitly, it will be automatically released when the pipeline
 * handler is destroyed.
 *
 * \context This function shall be called from the CameraManager thread.
 *
 * \return A pointer to the matching MediaDevice, or nullptr if no match is found
 */
MediaDevice *PipelineHandler::acquireMediaDevice(DeviceEnumerator *enumerator,
						 const DeviceMatch &dm)
{
	std::shared_ptr<MediaDevice> media = enumerator->search(dm);
	if (!media)
		return nullptr;

	if (!media->acquire())
		return nullptr;

	mediaDevices_.push_back(media);

	return media.get();
}

/**
 * \brief Acquire exclusive access to the pipeline handler for the process
 *
 * This function locks all the media devices used by the pipeline to ensure
 * that no other process can access them concurrently.
 *
 * Access to a pipeline handler may be acquired recursively from within the
 * same process. Every successful acquire() call shall be matched with a
 * release() call. This allows concurrent access to the same pipeline handler
 * from different cameras within the same process.
 *
 * Pipeline handlers shall not call this function directly as the Camera class
 * handles access internally.
 *
 * \context This function is \threadsafe.
 *
 * \return True if the pipeline handler was acquired, false if another process
 * has already acquired it
 * \sa release()
 */
bool PipelineHandler::acquire()
{
	MutexLocker locker(lock_);

	if (useCount_) {
		++useCount_;
		return true;
	}

	for (std::shared_ptr<MediaDevice> &media : mediaDevices_) {
		if (!media->lock()) {
			unlockMediaDevices();
			return false;
		}
	}

	++useCount_;
	return true;
}

/**
 * \brief Release exclusive access to the pipeline handler
 * \param[in] camera The camera for which to release data
 *
 * This function releases access to the pipeline handler previously acquired by
 * a call to acquire(). Every release() call shall match a previous successful
 * acquire() call. Calling this function on a pipeline handler that hasn't been
 * acquired results in undefined behaviour.
 *
 * Pipeline handlers shall not call this function directly as the Camera class
 * handles access internally.
 *
 * \context This function is \threadsafe.
 *
 * \sa acquire()
 */
void PipelineHandler::release(Camera *camera)
{
	MutexLocker locker(lock_);

	ASSERT(useCount_);

	unlockMediaDevices();

	releaseDevice(camera);

	--useCount_;
}

/**
 * \brief Release resources associated with this camera
 * \param[in] camera The camera for which to release resources
 *
 * Pipeline handlers may override this in order to perform cleanup operations
 * when a camera is released, such as freeing memory.
 */
void PipelineHandler::releaseDevice([[maybe_unused]] Camera *camera)
{
}

void PipelineHandler::unlockMediaDevices()
{
	for (std::shared_ptr<MediaDevice> &media : mediaDevices_)
		media->unlock();
}

/**
 * \fn PipelineHandler::generateConfiguration()
 * \brief Generate a camera configuration for a specified camera
 * \param[in] camera The camera to generate a default configuration for
 * \param[in] roles A list of stream roles
 *
 * Generate a default configuration for the \a camera for a specified list of
 * stream roles. The caller shall populate the \a roles with the use-cases it
 * wishes to fetch the default configuration for. The returned configuration
 * can then be examined by the caller to learn about the selected streams and
 * their default parameters.
 *
 * The intended companion to this is \a configure() which can be used to change
 * the group of streams parameters.
 *
 * \context This function may be called from any thread and shall be
 * \threadsafe. It shall not modify the state of the \a camera in the pipeline
 * handler.
 *
 * \return A valid CameraConfiguration if the requested roles can be satisfied,
 * or a null pointer otherwise.
 */

/**
 * \fn PipelineHandler::configure()
 * \brief Configure a group of streams for capture
 * \param[in] camera The camera to configure
 * \param[in] config The camera configurations to setup
 *
 * Configure the specified group of streams for \a camera according to the
 * configuration specified in \a config. The intended caller of this interface
 * is the Camera class which will receive configuration to apply from the
 * application.
 *
 * The configuration is guaranteed to have been validated with
 * CameraConfiguration::validate(). The pipeline handler implementation shall
 * not perform further validation and may rely on any custom field stored in its
 * custom CameraConfiguration derived class.
 *
 * When configuring the camera the pipeline handler shall associate a Stream
 * instance to each StreamConfiguration entry in the CameraConfiguration using
 * the StreamConfiguration::setStream() function.
 *
 * \context This function is called from the CameraManager thread.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \fn PipelineHandler::exportFrameBuffers()
 * \brief Allocate and export buffers for \a stream
 * \param[in] camera The camera
 * \param[in] stream The stream to allocate buffers for
 * \param[out] buffers Array of buffers successfully allocated
 *
 * This function allocates buffers for the \a stream from the devices associated
 * with the stream in the corresponding pipeline handler. Those buffers shall be
 * suitable to be added to a Request for the stream, and shall be mappable to
 * the CPU through their associated dmabufs with mmap().
 *
 * The function may only be called after the Camera has been configured and
 * before it gets started, or after it gets stopped. It shall be called only for
 * streams that are part of the active camera configuration.
 *
 * The only intended caller is Camera::exportFrameBuffers().
 *
 * \context This function is called from the CameraManager thread.
 *
 * \return The number of allocated buffers on success or a negative error code
 * otherwise
 */

/**
 * \fn PipelineHandler::start()
 * \brief Start capturing from a group of streams
 * \param[in] camera The camera to start
 * \param[in] controls Controls to be applied before starting the Camera
 *
 * Start the group of streams that have been configured for capture by
 * \a configure(). The intended caller of this function is the Camera class
 * which will in turn be called from the application to indicate that it has
 * configured the streams and is ready to capture.
 *
 * \context This function is called from the CameraManager thread.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \brief Stop capturing from all running streams and cancel pending requests
 * \param[in] camera The camera to stop
 *
 * This function stops capturing and processing requests immediately. All
 * pending requests are cancelled and complete immediately in an error state.
 *
 * \context This function is called from the CameraManager thread.
 */
void PipelineHandler::stop(Camera *camera)
{
	/* Stop the pipeline handler and let the queued requests complete. */
	stopDevice(camera);

	/* Cancel and signal as complete all waiting requests. */
	while (!waitingRequests_.empty()) {
		Request *request = waitingRequests_.front();
		waitingRequests_.pop();

		request->_d()->cancel();
		completeRequest(request);
	}

	/* Make sure no requests are pending. */
	Camera::Private *data = camera->_d();
	ASSERT(data->queuedRequests_.empty());

	data->requestSequence_ = 0;
}

/**
 * \fn PipelineHandler::stopDevice()
 * \brief Stop capturing from all running streams
 * \param[in] camera The camera to stop
 *
 * This function stops capturing and processing requests immediately. All
 * pending requests are cancelled and complete immediately in an error state.
 */

/**
 * \brief Determine if the camera has any requests pending
 * \param[in] camera The camera to check
 *
 * This function determines if there are any requests queued to the pipeline
 * awaiting processing.
 *
 * \return True if there are pending requests, or false otherwise
 */
bool PipelineHandler::hasPendingRequests(const Camera *camera) const
{
	return !camera->_d()->queuedRequests_.empty();
}

/**
 * \fn PipelineHandler::registerRequest()
 * \brief Register a request for use by the pipeline handler
 * \param[in] request The request to register
 *
 * This function is called when the request is created, and allows the pipeline
 * handler to perform any one-time initialization it requries for the request.
 */
void PipelineHandler::registerRequest(Request *request)
{
	/*
	 * Connect the request prepared signal to notify the pipeline handler
	 * when a request is ready to be processed.
	 */
	request->_d()->prepared.connect(this, &PipelineHandler::doQueueRequests);
}

/**
 * \fn PipelineHandler::queueRequest()
 * \brief Queue a request
 * \param[in] request The request to queue
 *
 * This function queues a capture request to the pipeline handler for
 * processing. The request is first added to the internal list of waiting
 * requests which have to be prepared to make sure they are ready for being
 * queued to the pipeline handler.
 *
 * The queue of waiting requests is iterated and all prepared requests are
 * passed to the pipeline handler in the same order they have been queued by
 * calling this function.
 *
 * If a Request fails during the preparation phase or if the pipeline handler
 * fails in queuing the request to the hardware the request is cancelled.
 *
 * Keeping track of queued requests ensures automatic completion of all requests
 * when the pipeline handler is stopped with stop(). Request completion shall be
 * signalled by the pipeline handler using the completeRequest() function.
 *
 * \context This function is called from the CameraManager thread.
 */
void PipelineHandler::queueRequest(Request *request)
{
	LIBCAMERA_TRACEPOINT(request_queue, request);

	waitingRequests_.push(request);

	request->_d()->prepare(300ms);
}

/**
 * \brief Queue one requests to the device
 */
void PipelineHandler::doQueueRequest(Request *request)
{
	LIBCAMERA_TRACEPOINT(request_device_queue, request);

	Camera *camera = request->_d()->camera();
	Camera::Private *data = camera->_d();
	data->queuedRequests_.push_back(request);

	request->_d()->sequence_ = data->requestSequence_++;

	if (request->_d()->cancelled_) {
		completeRequest(request);
		return;
	}

	int ret = queueRequestDevice(camera, request);
	if (ret) {
		request->_d()->cancel();
		completeRequest(request);
	}
}

/**
 * \brief Queue prepared requests to the device
 *
 * Iterate the list of waiting requests and queue them to the device one
 * by one if they have been prepared.
 */
void PipelineHandler::doQueueRequests()
{
	while (!waitingRequests_.empty()) {
		Request *request = waitingRequests_.front();
		if (!request->_d()->prepared_)
			break;

		doQueueRequest(request);
		waitingRequests_.pop();
	}
}

/**
 * \fn PipelineHandler::queueRequestDevice()
 * \brief Queue a request to the device
 * \param[in] camera The camera to queue the request to
 * \param[in] request The request to queue
 *
 * This function queues a capture request to the device for processing. The
 * request contains a set of buffers associated with streams and a set of
 * parameters. The pipeline handler shall program the device to ensure that the
 * parameters will be applied to the frames captured in the buffers provided in
 * the request.
 *
 * \context This function is called from the CameraManager thread.
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \brief Complete a buffer for a request
 * \param[in] request The request the buffer belongs to
 * \param[in] buffer The buffer that has completed
 *
 * This function shall be called by pipeline handlers to signal completion of
 * the \a buffer part of the \a request. It notifies applications of buffer
 * completion and updates the request's internal buffer tracking. The request
 * is not completed automatically when the last buffer completes to give
 * pipeline handlers a chance to perform any operation that may still be
 * needed. They shall complete requests explicitly with completeRequest().
 *
 * \context This function shall be called from the CameraManager thread.
 *
 * \return True if all buffers contained in the request have completed, false
 * otherwise
 */
bool PipelineHandler::completeBuffer(Request *request, FrameBuffer *buffer)
{
	Camera *camera = request->_d()->camera();
	camera->bufferCompleted.emit(request, buffer);
	return request->_d()->completeBuffer(buffer);
}

/**
 * \brief Signal request completion
 * \param[in] request The request that has completed
 *
 * The pipeline handler shall call this function to notify the \a camera that
 * the request has completed. The request is no longer managed by the pipeline
 * handler and shall not be accessed once this function returns.
 *
 * This function ensures that requests will be returned to the application in
 * submission order, the pipeline handler may call it on any complete request
 * without any ordering constraint.
 *
 * \context This function shall be called from the CameraManager thread.
 */
void PipelineHandler::completeRequest(Request *request)
{
	Camera *camera = request->_d()->camera();

	request->_d()->complete();

	Camera::Private *data = camera->_d();

	while (!data->queuedRequests_.empty()) {
		Request *req = data->queuedRequests_.front();
		if (req->status() == Request::RequestPending)
			break;

		ASSERT(!req->hasPendingBuffers());
		data->queuedRequests_.pop_front();
		camera->requestComplete(req);
	}
}

/**
 * \brief Register a camera to the camera manager and pipeline handler
 * \param[in] camera The camera to be added
 *
 * This function is called by pipeline handlers to register the cameras they
 * handle with the camera manager.
 *
 * \context This function shall be called from the CameraManager thread.
 */
void PipelineHandler::registerCamera(std::shared_ptr<Camera> camera)
{
	cameras_.push_back(camera);

	if (mediaDevices_.empty())
		LOG(Pipeline, Fatal)
			<< "Registering camera with no media devices!";

	/*
	 * Walk the entity list and map the devnums of all capture video nodes
	 * to the camera.
	 */
	std::vector<dev_t> devnums;
	for (const std::shared_ptr<MediaDevice> &media : mediaDevices_) {
		for (const MediaEntity *entity : media->entities()) {
			if (entity->pads().size() == 1 &&
			    (entity->pads()[0]->flags() & MEDIA_PAD_FL_SINK) &&
			    entity->function() == MEDIA_ENT_F_IO_V4L) {
				devnums.push_back(makedev(entity->deviceMajor(),
							  entity->deviceMinor()));
			}
		}
	}

	manager_->addCamera(std::move(camera), devnums);
}

/**
 * \brief Enable hotplug handling for a media device
 * \param[in] media The media device
 *
 * This function enables hotplug handling, and especially hot-unplug handling,
 * of the \a media device. It shall be called by pipeline handlers for all the
 * media devices that can be disconnected.
 *
 * When a media device passed to this function is later unplugged, the pipeline
 * handler gets notified and automatically disconnects all the cameras it has
 * registered without requiring any manual intervention.
 */
void PipelineHandler::hotplugMediaDevice(MediaDevice *media)
{
	media->disconnected.connect(this, [=]() { mediaDeviceDisconnected(media); });
}

/**
 * \brief Slot for the MediaDevice disconnected signal
 */
void PipelineHandler::mediaDeviceDisconnected(MediaDevice *media)
{
	media->disconnected.disconnect(this);

	if (cameras_.empty())
		return;

	disconnect();
}

/**
 * \brief Device disconnection handler
 *
 * This virtual function is called to notify the pipeline handler that the
 * device it handles has been disconnected. It notifies all cameras created by
 * the pipeline handler that they have been disconnected, and unregisters them
 * from the camera manager.
 *
 * The function can be overloaded by pipeline handlers to perform custom
 * operations at disconnection time. Any overloaded version shall call the
 * PipelineHandler::disconnect() base function for proper hot-unplug operation.
 */
void PipelineHandler::disconnect()
{
	/*
	 * Each camera holds a reference to its associated pipeline handler
	 * instance. Hence, when the last camera is dropped, the pipeline
	 * handler will get destroyed by the last manager_->removeCamera(camera)
	 * call in the loop below.
	 *
	 * This is acceptable as long as we make sure that the code path does not
	 * access any member of the (already destroyed) pipeline handler instance
	 * afterwards. Therefore, we move the cameras_ vector to a local temporary
	 * container to avoid accessing freed memory later i.e. to explicitly run
	 * cameras_.clear().
	 */
	std::vector<std::weak_ptr<Camera>> cameras{ std::move(cameras_) };

	for (const std::weak_ptr<Camera> &ptr : cameras) {
		std::shared_ptr<Camera> camera = ptr.lock();
		if (!camera)
			continue;

		camera->disconnect();
		manager_->removeCamera(camera);
	}
}

/**
 * \var PipelineHandler::manager_
 * \brief The Camera manager associated with the pipeline handler
 *
 * The camera manager pointer is stored in the pipeline handler for the
 * convenience of pipeline handler implementations. It remains valid and
 * constant for the whole lifetime of the pipeline handler.
 */

/**
 * \fn PipelineHandler::name()
 * \brief Retrieve the pipeline handler name
 * \context This function shall be \threadsafe.
 * \return The pipeline handler name
 */

/**
 * \class PipelineHandlerFactoryBase
 * \brief Base class for pipeline handler factories
 *
 * The PipelineHandlerFactoryBase class is the base of all specializations of
 * the PipelineHandlerFactory class template. It implements the factory
 * registration, maintains a registry of factories, and provides access to the
 * registered factories.
 */

/**
 * \brief Construct a pipeline handler factory base
 * \param[in] name Name of the pipeline handler class
 *
 * Creating an instance of the factory base registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used for debug purpose and shall be unique.
 */
PipelineHandlerFactoryBase::PipelineHandlerFactoryBase(const char *name)
	: name_(name)
{
	registerType(this);
}

/**
 * \brief Create an instance of the PipelineHandler corresponding to the factory
 * \param[in] manager The camera manager
 *
 * \return A shared pointer to a new instance of the PipelineHandler subclass
 * corresponding to the factory
 */
std::shared_ptr<PipelineHandler> PipelineHandlerFactoryBase::create(CameraManager *manager) const
{
	std::unique_ptr<PipelineHandler> handler = createInstance(manager);
	handler->name_ = name_.c_str();
	return std::shared_ptr<PipelineHandler>(std::move(handler));
}

/**
 * \fn PipelineHandlerFactoryBase::name()
 * \brief Retrieve the factory name
 * \return The factory name
 */

/**
 * \brief Add a pipeline handler class to the registry
 * \param[in] factory Factory to use to construct the pipeline handler
 *
 * The caller is responsible to guarantee the uniqueness of the pipeline handler
 * name.
 */
void PipelineHandlerFactoryBase::registerType(PipelineHandlerFactoryBase *factory)
{
	std::vector<PipelineHandlerFactoryBase *> &factories =
		PipelineHandlerFactoryBase::factories();

	factories.push_back(factory);
}

/**
 * \brief Retrieve the list of all pipeline handler factories
 * \return the list of pipeline handler factories
 */
std::vector<PipelineHandlerFactoryBase *> &PipelineHandlerFactoryBase::factories()
{
	/*
	 * The static factories map is defined inside the function to ensure
	 * it gets initialized on first use, without any dependency on
	 * link order.
	 */
	static std::vector<PipelineHandlerFactoryBase *> factories;
	return factories;
}

/**
 * \class PipelineHandlerFactory
 * \brief Registration of PipelineHandler classes and creation of instances
 * \tparam _PipelineHandler The pipeline handler class type for this factory
 *
 * To facilitate discovery and instantiation of PipelineHandler classes, the
 * PipelineHandlerFactory class implements auto-registration of pipeline
 * handlers. Each PipelineHandler subclass shall register itself using the
 * REGISTER_PIPELINE_HANDLER() macro, which will create a corresponding
 * instance of a PipelineHandlerFactory and register it with the static list of
 * factories.
 */

/**
 * \fn PipelineHandlerFactory::PipelineHandlerFactory(const char *name)
 * \brief Construct a pipeline handler factory
 * \param[in] name Name of the pipeline handler class
 *
 * Creating an instance of the factory registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used for debug purpose and shall be unique.
 */

/**
 * \fn PipelineHandlerFactory::createInstance() const
 * \brief Create an instance of the PipelineHandler corresponding to the factory
 * \param[in] manager The camera manager
 * \return A unique pointer to a newly constructed instance of the
 * PipelineHandler subclass corresponding to the factory
 */

/**
 * \def REGISTER_PIPELINE_HANDLER
 * \brief Register a pipeline handler with the pipeline handler factory
 * \param[in] handler Class name of PipelineHandler derived class to register
 *
 * Register a PipelineHandler subclass with the factory and make it available to
 * try and match devices.
 */

} /* namespace libcamera */
