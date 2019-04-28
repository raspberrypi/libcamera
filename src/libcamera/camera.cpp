/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.cpp - Camera device
 */

#include <libcamera/camera.h>

#include <iomanip>

#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "log.h"
#include "pipeline_handler.h"
#include "utils.h"

/**
 * \file camera.h
 * \brief Camera device handling
 *
 * At the core of libcamera is the camera device, combining one image source
 * with processing hardware able to provide one or multiple image streams. The
 * Camera class represents a camera device.
 *
 * A camera device contains a single image source, and separate camera device
 * instances relate to different image sources. For instance, a phone containing
 * front and back image sensors will be modelled with two camera devices, one
 * for each sensor. When multiple streams can be produced from the same image
 * source, all those streams are guaranteed to be part of the same camera
 * device.
 *
 * While not sharing image sources, separate camera devices can share other
 * system resources, such as an ISP. For this reason camera device instances may
 * not be fully independent, in which case usage restrictions may apply. For
 * instance, a phone with a front and a back camera device may not allow usage
 * of the two devices simultaneously.
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(Camera)

/**
 * \class CameraConfiguration
 * \brief Hold configuration for streams of the camera

 * The CameraConfiguration holds an ordered list of streams and their associated
 * StreamConfiguration. From a data storage point of view, the class operates as
 * a map of Stream pointers to StreamConfiguration, with entries accessed with
 * operator[](Stream *). Accessing an entry for a Stream pointer not yet stored
 * in the configuration inserts a new empty entry.
 *
 * The class also suppors iterators, and from that point of view operates as a
 * vector of Stream pointers. The streams are iterated in insertion order, and
 * the operator[](int) returns the Stream pointer based on its insertion index.
 * Accessing a stream with an invalid index returns a null pointer.
 */

/**
 * \typedef CameraConfiguration::iterator
 * \brief Iterator for the streams in the configuration
 */

/**
 * \typedef CameraConfiguration::const_iterator
 * \brief Const iterator for the streams in the configuration
 */

/**
 * \brief Create an empty camera configuration
 */
CameraConfiguration::CameraConfiguration()
	: order_({}), config_({})
{
}

/**
 * \brief Retrieve an iterator to the first stream in the sequence
 * \return An iterator to the first stream
 */
std::vector<Stream *>::iterator CameraConfiguration::begin()
{
	return order_.begin();
}

/**
 * \brief Retrieve an iterator pointing to the past-the-end stream in the
 * sequence
 * \return An iterator to the element following the last stream
 */
std::vector<Stream *>::iterator CameraConfiguration::end()
{
	return order_.end();
}

/**
 * \brief Retrieve a const iterator to the first element of the streams
 * \return A const iterator to the first stream
 */
std::vector<Stream *>::const_iterator CameraConfiguration::begin() const
{
	return order_.begin();
}

/**
 * \brief Retrieve a const iterator pointing to the past-the-end stream in the
 * sequence
 * \return A const iterator to the element following the last stream
 */
std::vector<Stream *>::const_iterator CameraConfiguration::end() const
{
	return order_.end();
}

/**
 * \brief Check if the camera configuration is valid
 *
 * A camera configuration is deemed to be valid if it contains at least one
 * stream configuration and all stream configurations contain valid information.
 * Stream configurations are deemed to be valid if all fields are none zero.
 *
 * \return True if the configuration is valid
 */
bool CameraConfiguration::isValid() const
{
	if (isEmpty())
		return false;

	for (auto const &it : config_) {
		const StreamConfiguration &cfg = it.second;

		if (cfg.size.width == 0 || cfg.size.height == 0 ||
		    cfg.pixelFormat == 0 || cfg.bufferCount == 0)
			return false;
	}

	return true;
}

/**
 * \brief Check if the camera configuration is empty
 * \return True if the configuration is empty
 */
bool CameraConfiguration::isEmpty() const
{
	return order_.empty();
}

/**
 * \brief Retrieve the number of stream configurations
 * \return Number of stream configurations
 */
std::size_t CameraConfiguration::size() const
{
	return order_.size();
}

/**
 * \brief Access the first stream in the configuration
 * \return The first stream in the configuration
 */
Stream *CameraConfiguration::front()
{
	return order_.front();
}

/**
 * \brief Access the first stream in the configuration
 * \return The first const stream pointer in the configuration
 */
const Stream *CameraConfiguration::front() const
{
	return order_.front();
}

/**
 * \brief Retrieve a stream pointer from index
 * \param[in] index Numerical index
 *
 * The \a index represents the zero based insertion order of stream and stream
 * configuration into the camera configuration.
 *
 * \return The stream pointer at index, or a nullptr if the index is out of
 * bounds
 */
Stream *CameraConfiguration::operator[](unsigned int index) const
{
	if (index >= order_.size())
		return nullptr;

	return order_.at(index);
}

/**
 * \brief Retrieve a reference to a stream configuration
 * \param[in] stream Stream to retrieve configuration for
 *
 * If the camera configuration does not yet contain a configuration for
 * the requested stream, create and return an empty stream configuration.
 *
 * \return The configuration for the stream
 */
StreamConfiguration &CameraConfiguration::operator[](Stream *stream)
{
	if (config_.find(stream) == config_.end())
		order_.push_back(stream);

	return config_[stream];
}

/**
 * \brief Retrieve a const reference to a stream configuration
 * \param[in] stream Stream to retrieve configuration for
 *
 * No new stream configuration is created if called with \a stream that is not
 * already part of the camera configuration, doing so is an invalid operation
 * and results in undefined behaviour.
 *
 * \return The configuration for the stream
 */
const StreamConfiguration &CameraConfiguration::operator[](Stream *stream) const
{
	return config_.at(stream);
}

/**
 * \class Camera
 * \brief Camera device
 *
 * \todo Add documentation for camera start timings. What exactly does the
 * camera expect the pipeline handler to do when start() is called?
 *
 * The Camera class models a camera capable of producing one or more image
 * streams from a single image source. It provides the main interface to
 * configuring and controlling the device, and capturing image streams. It is
 * the central object exposed by libcamera.
 *
 * To support the central nature of Camera objects, libcamera manages the
 * lifetime of camera instances with std::shared_ptr<>. Instances shall be
 * created with the create() function which returns a shared pointer. The
 * Camera constructors and destructor are private, to prevent instances from
 * being constructed and destroyed manually.
 *
 * \section camera_operation Operating the Camera
 *
 * An application needs to perform a sequence of operations on a camera before
 * it is ready to process requests. The camera needs to be acquired, configured
 * and resources allocated or imported to prepare the camera for capture. Once
 * started the camera can process requests until it is stopped. When an
 * application is done with a camera all resources allocated need to be freed
 * and the camera released.
 *
 * An application may start and stop a camera multiple times as long as it is
 * not released. The camera may also be reconfigured provided that all
 * resources allocated are freed prior to the reconfiguration.
 *
 * \subsection Camera States
 *
 * To help manage the sequence of operations needed to control the camera a set
 * of states are defined. Each state describes which operations may be performed
 * on the camera. Operations not listed in the state diagram are allowed in all
 * states.
 *
 * \dot
 * digraph camera_state_machine {
 *   node [shape = doublecircle ]; Available;
 *   node [shape = circle ]; Acquired;
 *   node [shape = circle ]; Configured;
 *   node [shape = circle ]; Prepared;
 *   node [shape = circle ]; Running;
 *
 *   Available -> Available [label = "release()"];
 *   Available -> Acquired [label = "acquire()"];
 *
 *   Acquired -> Available [label = "release()"];
 *   Acquired -> Configured [label = "configure()"];
 *
 *   Configured -> Available [label = "release()"];
 *   Configured -> Configured [label = "configure()"];
 *   Configured -> Prepared [label = "allocateBuffers()"];
 *
 *   Prepared -> Configured [label = "freeBuffers()"];
 *   Prepared -> Prepared [label = "createRequest()"];
 *   Prepared -> Running [label = "start()"];
 *
 *   Running -> Prepared [label = "stop()"];
 *   Running -> Running [label = "createRequest(), queueRequest()"];
 * }
 * \enddot
 *
 * \subsubsection Available
 * The base state of a camera, an application can inspect the properties of the
 * camera to determine if it wishes to use it. If an application wishes to use
 * a camera it should acquire() it to proceed to the Acquired state.
 *
 * \subsubsection Acquired
 * In the acquired state an application has exclusive access to the camera and
 * may modify the camera's parameters to configure it and proceed to the
 * Configured state.
 *
 * \subsubsection Configured
 * The camera is configured and ready for the application to prepare it with
 * resources. The camera may be reconfigured multiple times until resources
 * are provided and the state progresses to Prepared.
 *
 * \subsubsection Prepared
 * The camera has been configured and provided with resources and is ready to be
 * started. The application may free the camera's resources to get back to the
 * Configured state or start() it to progress to the Running state.
 *
 * \subsubsection Running
 * The camera is running and ready to process requests queued by the
 * application. The camera remains in this state until it is stopped and moved
 * to the Prepared state.
 */

/**
 * \brief Create a camera instance
 * \param[in] name The name of the camera device
 * \param[in] pipe The pipeline handler responsible for the camera device
 * \param[in] streams Array of streams the camera provides
 *
 * The caller is responsible for guaranteeing unicity of the camera name.
 *
 * \return A shared pointer to the newly created camera object
 */
std::shared_ptr<Camera> Camera::create(PipelineHandler *pipe,
				       const std::string &name,
				       const std::set<Stream *> &streams)
{
	struct Allocator : std::allocator<Camera> {
		void construct(void *p, PipelineHandler *pipe,
			       const std::string &name)
		{
			::new(p) Camera(pipe, name);
		}
		void destroy(Camera *p)
		{
			p->~Camera();
		}
	};

	std::shared_ptr<Camera> camera =
		std::allocate_shared<Camera>(Allocator(), pipe, name);

	camera->streams_ = streams;

	return camera;
}

/**
 * \brief Retrieve the name of the camera
 * \return Name of the camera device
 */
const std::string &Camera::name() const
{
	return name_;
}

/**
 * \var Camera::bufferCompleted
 * \brief Signal emitted when a buffer for a request queued to the camera has
 * completed
 */

/**
 * \var Camera::requestCompleted
 * \brief Signal emitted when a request queued to the camera has completed
 */

/**
 * \var Camera::disconnected
 * \brief Signal emitted when the camera is disconnected from the system
 *
 * This signal is emitted when libcamera detects that the camera has been
 * removed from the system. For hot-pluggable devices this is usually caused by
 * physical device disconnection. The media device is passed as a parameter.
 *
 * As soon as this signal is emitted the camera instance will refuse all new
 * application API calls by returning errors immediately.
 */

Camera::Camera(PipelineHandler *pipe, const std::string &name)
	: pipe_(pipe->shared_from_this()), name_(name), disconnected_(false),
	  state_(CameraAvailable)
{
}

Camera::~Camera()
{
	if (!stateIs(CameraAvailable))
		LOG(Camera, Error) << "Removing camera while still in use";
}

static const char *const camera_state_names[] = {
	"Available",
	"Acquired",
	"Configured",
	"Prepared",
	"Running",
};

bool Camera::stateBetween(State low, State high) const
{
	if (state_ >= low && state_ <= high)
		return true;

	ASSERT(static_cast<unsigned int>(low) < ARRAY_SIZE(camera_state_names) &&
	       static_cast<unsigned int>(high) < ARRAY_SIZE(camera_state_names));

	LOG(Camera, Debug) << "Camera in " << camera_state_names[state_]
			   << " state trying operation requiring state between "
			   << camera_state_names[low] << " and "
			   << camera_state_names[high];

	return false;
}

bool Camera::stateIs(State state) const
{
	if (state_ == state)
		return true;

	ASSERT(static_cast<unsigned int>(state) < ARRAY_SIZE(camera_state_names));

	LOG(Camera, Debug) << "Camera in " << camera_state_names[state_]
			   << " state trying operation requiring state "
			   << camera_state_names[state];

	return false;
}

/**
 * \brief Notify camera disconnection
 *
 * This method is used to notify the camera instance that the underlying
 * hardware has been unplugged. In response to the disconnection the camera
 * instance notifies the application by emitting the #disconnected signal, and
 * ensures that all new calls to the application-facing Camera API return an
 * error immediately.
 *
 * \todo Deal with pending requests if the camera is disconnected in a
 * running state.
 * \todo Update comment about Running state when importing buffers as well as
 * allocating them are supported.
 */
void Camera::disconnect()
{
	LOG(Camera, Debug) << "Disconnecting camera " << name_;

	/*
	 * If the camera was running when the hardware was removed force the
	 * state to Prepared to allow applications to call freeBuffers() and
	 * release() before deleting the camera.
	 */
	if (state_ == CameraRunning)
		state_ = CameraPrepared;

	disconnected_ = true;
	disconnected.emit(this);
}

/**
 * \brief Acquire the camera device for exclusive access
 *
 * After opening the device with open(), exclusive access must be obtained
 * before performing operations that change the device state. This function is
 * not blocking, if the device has already been acquired (by the same or another
 * process) the -EBUSY error code is returned.
 *
 * Acquiring a camera will limit usage of any other camera(s) provided by the
 * same pipeline handler to the same instance of libcamera. The limit is in
 * effect until all cameras from the pipeline handler are released. Other
 * instances of libcamera can still list and examine the cameras but will fail
 * if they attempt to acquire() any of them.
 *
 * Once exclusive access isn't needed anymore, the device should be released
 * with a call to the release() function.
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EBUSY The camera is not free and can't be acquired by the caller
 */
int Camera::acquire()
{
	if (disconnected_)
		return -ENODEV;

	if (!stateIs(CameraAvailable))
		return -EBUSY;

	if (!pipe_->lock()) {
		LOG(Camera, Info)
			<< "Pipeline handler in use by another process";
		return -EBUSY;
	}

	state_ = CameraAcquired;

	return 0;
}

/**
 * \brief Release exclusive access to the camera device
 *
 * Releasing the camera device allows other users to acquire exclusive access
 * with the acquire() function.
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EBUSY The camera is running and can't be released
 */
int Camera::release()
{
	if (!stateBetween(CameraAvailable, CameraConfigured))
		return -EBUSY;

	pipe_->unlock();

	state_ = CameraAvailable;

	return 0;
}

/**
 * \brief Retrieve all the camera's stream information
 *
 * Retrieve all of the camera's static stream information. The static
 * information describes among other things how many streams the camera
 * supports and the capabilities of each stream.
 *
 * \return An array of all the camera's streams.
 */
const std::set<Stream *> &Camera::streams() const
{
	return streams_;
}

/**
 * \brief Generate a default camera configuration according to stream usages
 * \param[in] usages A list of stream usages
 *
 * Generate a camera configuration for a set of desired usages. The caller
 * specifies a list of stream usages and the camera returns a configuration
 * containing suitable streams and their suggested default configurations.
 *
 * \return A valid CameraConfiguration if the requested usages can be satisfied,
 * or a invalid one otherwise
 */
CameraConfiguration
Camera::generateConfiguration(const std::vector<StreamUsage> &usages)
{
	if (disconnected_ || !usages.size() || usages.size() > streams_.size())
		return CameraConfiguration();

	CameraConfiguration config = pipe_->generateConfiguration(this, usages);

	std::ostringstream msg("streams configuration:", std::ios_base::ate);
	unsigned int index = 0;

	for (Stream *stream : config) {
		const StreamConfiguration &cfg = config[stream];
		msg << " (" << index << ") " << cfg.toString();
		index++;
	}

	LOG(Camera, Debug) << msg.str();

	return config;
}

/**
 * \brief Configure the camera prior to capture
 * \param[in] config The camera configurations to setup
 *
 * Prior to starting capture, the camera must be configured to select a
 * group of streams to be involved in the capture and their configuration.
 * The caller specifies which streams are to be involved and their configuration
 * by populating \a config.
 *
 * The easiest way to populate the array of config is to fetch an initial
 * configuration from the camera with generateConfiguration() and then change
 * the parameters to fit the caller's need and once all the streams parameters
 * are configured hand that over to configure() to actually setup the camera.
 *
 * Exclusive access to the camera shall be ensured by a call to acquire() prior
 * to calling this function, otherwise an -EACCES error will be returned.
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not in a state where it can be configured
 * \retval -EINVAL The configuration is not valid
 */
int Camera::configure(const CameraConfiguration &config)
{
	int ret;

	if (disconnected_)
		return -ENODEV;

	if (!stateBetween(CameraAcquired, CameraConfigured))
		return -EACCES;

	if (!config.isValid()) {
		LOG(Camera, Error)
			<< "Can't configure camera with invalid configuration";
		return -EINVAL;
	}

	std::ostringstream msg("configuring streams:", std::ios_base::ate);
	unsigned int index = 0;

	for (Stream *stream : config) {
		if (streams_.find(stream) == streams_.end())
			return -EINVAL;

		const StreamConfiguration &cfg = config[stream];
		msg << std::dec << " (" << index << ") " << cfg.toString();

		index++;
	}

	LOG(Camera, Info) << msg.str();

	ret = pipe_->configure(this, config);
	if (ret)
		return ret;

	activeStreams_.clear();
	for (Stream *stream : config) {
		const StreamConfiguration &cfg = config[stream];

		stream->configuration_ = cfg;
		activeStreams_.insert(stream);

		/*
		 * Allocate buffer objects in the pool.
		 * Memory will be allocated and assigned later.
		 */
		stream->bufferPool().createBuffers(cfg.bufferCount);
	}

	state_ = CameraConfigured;

	return 0;
}

/**
 * \brief Allocate buffers for all configured streams
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not in a state where buffers can be allocated
 * \retval -EINVAL The configuration is not valid
 */
int Camera::allocateBuffers()
{
	if (disconnected_)
		return -ENODEV;

	if (!stateIs(CameraConfigured))
		return -EACCES;

	if (activeStreams_.empty()) {
		LOG(Camera, Error)
			<< "Can't allocate buffers without streams";
		return -EINVAL;
	}

	int ret = pipe_->allocateBuffers(this, activeStreams_);
	if (ret) {
		LOG(Camera, Error) << "Failed to allocate buffers";
		return ret;
	}

	state_ = CameraPrepared;

	return 0;
}

/**
 * \brief Release all buffers from allocated pools in each stream
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EACCES The camera is not in a state where buffers can be freed
 */
int Camera::freeBuffers()
{
	if (!stateIs(CameraPrepared))
		return -EACCES;

	for (Stream *stream : activeStreams_) {
		if (!stream->bufferPool().count())
			continue;

		/*
		 * All mappings must be destroyed before buffers can be freed
		 * by the V4L2 device that has allocated them.
		 */
		stream->bufferPool().destroyBuffers();
	}

	state_ = CameraConfigured;

	return pipe_->freeBuffers(this, activeStreams_);
}

/**
 * \brief Create a request object for the camera
 *
 * This method creates an empty request for the application to fill with
 * buffers and paramaters, and queue for capture.
 *
 * The ownership of the returned request is passed to the caller, which is
 * responsible for either queueing the request or deleting it.
 *
 * This function shall only be called when the camera is in the Prepared
 * or Running state, see \ref camera_operation.
 *
 * \return A pointer to the newly created request, or nullptr on error
 */
Request *Camera::createRequest()
{
	if (disconnected_ || !stateBetween(CameraPrepared, CameraRunning))
		return nullptr;

	return new Request(this);
}

/**
 * \brief Queue a request to the camera
 * \param[in] request The request to queue to the camera
 *
 * This method queues a \a request to the camera for capture.
 *
 * After allocating the request with createRequest(), the application shall
 * fill it with at least one capture buffer before queuing it. Requests that
 * contain no buffers are invalid and are rejected without being queued.
 *
 * Once the request has been queued, the camera will notify its completion
 * through the \ref requestCompleted signal.
 *
 * Ownership of the request is transferred to the camera. It will be deleted
 * automatically after it completes.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not running so requests can't be queued
 * \retval -EINVAL The request is invalid
 */
int Camera::queueRequest(Request *request)
{
	if (disconnected_)
		return -ENODEV;

	if (!stateIs(CameraRunning))
		return -EACCES;

	for (auto const &it : request->buffers()) {
		Stream *stream = it.first;
		if (activeStreams_.find(stream) == activeStreams_.end()) {
			LOG(Camera, Error) << "Invalid request";
			return -EINVAL;
		}
	}

	int ret = request->prepare();
	if (ret) {
		LOG(Camera, Error) << "Failed to prepare request";
		return ret;
	}

	return pipe_->queueRequest(this, request);
}

/**
 * \brief Start capture from camera
 *
 * Start the camera capture session. Once the camera is started the application
 * can queue requests to the camera to process and return to the application
 * until the capture session is terminated with \a stop().
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not in a state where it can be started
 */
int Camera::start()
{
	if (disconnected_)
		return -ENODEV;

	if (!stateIs(CameraPrepared))
		return -EACCES;

	LOG(Camera, Debug) << "Starting capture";

	int ret = pipe_->start(this);
	if (ret)
		return ret;

	state_ = CameraRunning;

	return 0;
}

/**
 * \brief Stop capture from camera
 *
 * This method stops capturing and processing requests immediately. All pending
 * requests are cancelled and complete synchronously in an error state.
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not running so can't be stopped
 */
int Camera::stop()
{
	if (disconnected_)
		return -ENODEV;

	if (!stateIs(CameraRunning))
		return -EACCES;

	LOG(Camera, Debug) << "Stopping capture";

	state_ = CameraPrepared;

	pipe_->stop(this);

	return 0;
}

/**
 * \brief Handle request completion and notify application
 * \param[in] request The request that has completed
 *
 * This function is called by the pipeline handler to notify the camera that
 * the request has completed. It emits the requestCompleted signal and deletes
 * the request.
 */
void Camera::requestComplete(Request *request)
{
	std::map<Stream *, Buffer *> buffers(std::move(request->bufferMap_));
	requestCompleted.emit(request, buffers);
	delete request;
}

} /* namespace libcamera */
