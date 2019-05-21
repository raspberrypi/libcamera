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

 * The CameraConfiguration holds an ordered list of stream configurations. It
 * supports iterators and operates as a vector of StreamConfiguration instances.
 * The stream configurations are inserted by addConfiguration(), and the
 * operator[](int) returns a reference to the StreamConfiguration based on its
 * insertion index. Accessing a stream configuration with an invalid index
 * results in undefined behaviour.
 *
 * CameraConfiguration instances are retrieved from the camera with
 * Camera::generateConfiguration(). Applications may then inspect the
 * configuration, modify it, and possibly add new stream configuration entries
 * with addConfiguration(). Once the camera configuration satisfies the
 * application, it shall be validated by a call to validate(). The validation
 * implements "try" semantics: it adjusts invalid configurations to the closest
 * achievable parameters instead of rejecting them completely. Applications
 * then decide whether to accept the modified configuration, or try again with
 * a different set of parameters. Once the configuration is valid, it is passed
 * to Camera::configure().
 */

/**
 * \enum CameraConfiguration::Status
 * \brief Validity of a camera configuration
 * \var CameraConfiguration::Valid
 * The configuration is fully valid
 * \var CameraConfiguration::Adjusted
 * The configuration has been adjusted to a valid configuration
 * \var CameraConfiguration::Invalid
 * The configuration is invalid and can't be adjusted automatically
 */

/**
 * \typedef CameraConfiguration::iterator
 * \brief Iterator for the stream configurations in the camera configuration
 */

/**
 * \typedef CameraConfiguration::const_iterator
 * \brief Const iterator for the stream configuration in the camera
 * configuration
 */

/**
 * \brief Create an empty camera configuration
 */
CameraConfiguration::CameraConfiguration()
	: config_({})
{
}

CameraConfiguration::~CameraConfiguration()
{
}

/**
 * \brief Add a stream configuration to the camera configuration
 * \param[in] cfg The stream configuration
 */
void CameraConfiguration::addConfiguration(const StreamConfiguration &cfg)
{
	config_.push_back(cfg);
}

/**
 * \fn CameraConfiguration::validate()
 * \brief Validate and possibly adjust the camera configuration
 *
 * This method adjusts the camera configuration to the closest valid
 * configuration and returns the validation status.
 *
 * \todo: Define exactly when to return each status code. Should stream
 * parameters set to 0 by the caller be adjusted without returning Adjusted ?
 * This would potentially be useful for applications but would get in the way
 * in Camera::configure(). Do we need an extra status code to signal this ?
 *
 * \todo: Handle validation of buffers count when refactoring the buffers API.
 *
 * \return A CameraConfiguration::Status value that describes the validation
 * status.
 * \retval CameraConfiguration::Invalid The configuration is invalid and can't
 * be adjusted. This may only occur in extreme cases such as when the
 * configuration is empty.
 * \retval CameraConfigutation::Adjusted The configuration has been adjusted
 * and is now valid. Parameters may have changed for any stream, and stream
 * configurations may have been removed. The caller shall check the
 * configuration carefully.
 * \retval CameraConfiguration::Valid The configuration was already valid and
 * hasn't been adjusted.
 */

/**
 * \brief Retrieve a reference to a stream configuration
 * \param[in] index Numerical index
 *
 * The \a index represents the zero based insertion order of stream
 * configuration into the camera configuration with addConfiguration(). Calling
 * this method with an invalid index results in undefined behaviour.
 *
 * \return The stream configuration
 */
StreamConfiguration &CameraConfiguration::at(unsigned int index)
{
	return config_[index];
}

/**
 * \brief Retrieve a const reference to a stream configuration
 * \param[in] index Numerical index
 *
 * The \a index represents the zero based insertion order of stream
 * configuration into the camera configuration with addConfiguration(). Calling
 * this method with an invalid index results in undefined behaviour.
 *
 * \return The stream configuration
 */
const StreamConfiguration &CameraConfiguration::at(unsigned int index) const
{
	return config_[index];
}

/**
 * \fn StreamConfiguration &CameraConfiguration::operator[](unsigned int)
 * \brief Retrieve a reference to a stream configuration
 * \param[in] index Numerical index
 *
 * The \a index represents the zero based insertion order of stream
 * configuration into the camera configuration with addConfiguration(). Calling
 * this method with an invalid index results in undefined behaviour.
 *
 * \return The stream configuration
 */

/**
 * \fn const StreamConfiguration &CameraConfiguration::operator[](unsigned int) const
 * \brief Retrieve a const reference to a stream configuration
 * \param[in] index Numerical index
 *
 * The \a index represents the zero based insertion order of stream
 * configuration into the camera configuration with addConfiguration(). Calling
 * this method with an invalid index results in undefined behaviour.
 *
 * \return The stream configuration
 */

/**
 * \brief Retrieve an iterator to the first stream configuration in the
 * sequence
 * \return An iterator to the first stream configuration
 */
CameraConfiguration::iterator CameraConfiguration::begin()
{
	return config_.begin();
}

/**
 * \brief Retrieve a const iterator to the first element of the stream
 * configurations
 * \return A const iterator to the first stream configuration
 */
CameraConfiguration::const_iterator CameraConfiguration::begin() const
{
	return config_.begin();
}

/**
 * \brief Retrieve an iterator pointing to the past-the-end stream
 * configuration in the sequence
 * \return An iterator to the element following the last stream configuration
 */
CameraConfiguration::iterator CameraConfiguration::end()
{
	return config_.end();
}

/**
 * \brief Retrieve a const iterator pointing to the past-the-end stream
 * configuration in the sequence
 * \return A const iterator to the element following the last stream
 * configuration
 */
CameraConfiguration::const_iterator CameraConfiguration::end() const
{
	return config_.end();
}

/**
 * \brief Check if the camera configuration is empty
 * \return True if the configuration is empty
 */
bool CameraConfiguration::empty() const
{
	return config_.empty();
}

/**
 * \brief Retrieve the number of stream configurations
 * \return Number of stream configurations
 */
std::size_t CameraConfiguration::size() const
{
	return config_.size();
}

/**
 * \var CameraConfiguration::config_
 * \brief The vector of stream configurations
 */

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
	struct Deleter : std::default_delete<Camera> {
		void operator()(Camera *camera)
		{
			delete camera;
		}
	};

	Camera *camera = new Camera(pipe, name);
	camera->streams_ = streams;

	return std::shared_ptr<Camera>(camera, Deleter());
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
 * \brief Generate a default camera configuration according to stream roles
 * \param[in] roles A list of stream roles
 *
 * Generate a camera configuration for a set of desired stream roles. The caller
 * specifies a list of stream roles and the camera returns a configuration
 * containing suitable streams and their suggested default configurations. An
 * empty list of roles is valid, and will generate an empty configuration that
 * can be filled by the caller.
 *
 * \return A CameraConfiguration if the requested roles can be satisfied, or a
 * null pointer otherwise. The ownership of the returned configuration is
 * passed to the caller.
 */
std::unique_ptr<CameraConfiguration> Camera::generateConfiguration(const StreamRoles &roles)
{
	if (disconnected_ || roles.size() > streams_.size())
		return nullptr;

	CameraConfiguration *config = pipe_->generateConfiguration(this, roles);
	if (!config) {
		LOG(Camera, Debug)
			<< "Pipeline handler failed to generate configuration";
		return nullptr;
	}

	std::ostringstream msg("streams configuration:", std::ios_base::ate);

	if (config->empty())
		msg << " empty";

	for (unsigned int index = 0; index < config->size(); ++index)
		msg << " (" << index << ") " << config->at(index).toString();

	LOG(Camera, Debug) << msg.str();

	return std::unique_ptr<CameraConfiguration>(config);
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
 * The configuration is created by generateConfiguration(), and adjusted by the
 * caller with CameraConfiguration::validate(). This method only accepts fully
 * valid configurations and returns an error if \a config is not valid.
 *
 * Exclusive access to the camera shall be ensured by a call to acquire() prior
 * to calling this function, otherwise an -EACCES error will be returned.
 *
 * This function affects the state of the camera, see \ref camera_operation.
 *
 * Upon return the StreamConfiguration entries in \a config are associated with
 * Stream instances which can be retrieved with StreamConfiguration::stream().
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not in a state where it can be configured
 * \retval -EINVAL The configuration is not valid
 */
int Camera::configure(CameraConfiguration *config)
{
	int ret;

	if (disconnected_)
		return -ENODEV;

	if (!stateBetween(CameraAcquired, CameraConfigured))
		return -EACCES;

	if (config->validate() != CameraConfiguration::Valid) {
		LOG(Camera, Error)
			<< "Can't configure camera with invalid configuration";
		return -EINVAL;
	}

	std::ostringstream msg("configuring streams:", std::ios_base::ate);

	for (unsigned int index = 0; index < config->size(); ++index) {
		StreamConfiguration &cfg = config->at(index);
		cfg.setStream(nullptr);
		msg << " (" << index << ") " << cfg.toString();
	}

	LOG(Camera, Info) << msg.str();

	ret = pipe_->configure(this, config);
	if (ret)
		return ret;

	activeStreams_.clear();
	for (const StreamConfiguration &cfg : *config) {
		Stream *stream = cfg.stream();
		if (!stream)
			LOG(Camera, Fatal)
				<< "Pipeline handler failed to update stream configuration";

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
