/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.cpp - Camera device
 */

#include <libcamera/camera.h>

#include <atomic>
#include <iomanip>

#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/utils.h"

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

class Camera::Private
{
public:
	enum State {
		CameraAvailable,
		CameraAcquired,
		CameraConfigured,
		CameraRunning,
	};

	Private(PipelineHandler *pipe, const std::string &id,
		const std::set<Stream *> &streams);
	~Private();

	int isAccessAllowed(State state, bool allowDisconnected = false) const;
	int isAccessAllowed(State low, State high,
			    bool allowDisconnected = false) const;

	void disconnect();
	void setState(State state);

	std::shared_ptr<PipelineHandler> pipe_;
	std::string id_;
	std::set<Stream *> streams_;
	std::set<const Stream *> activeStreams_;

private:
	bool disconnected_;
	std::atomic<State> state_;
};

Camera::Private::Private(PipelineHandler *pipe, const std::string &id,
			 const std::set<Stream *> &streams)
	: pipe_(pipe->shared_from_this()), id_(id), streams_(streams),
	  disconnected_(false), state_(CameraAvailable)
{
}

Camera::Private::~Private()
{
	if (state_.load(std::memory_order_acquire) != Private::CameraAvailable)
		LOG(Camera, Error) << "Removing camera while still in use";
}

static const char *const camera_state_names[] = {
	"Available",
	"Acquired",
	"Configured",
	"Running",
};

int Camera::Private::isAccessAllowed(State state, bool allowDisconnected) const
{
	if (!allowDisconnected && disconnected_)
		return -ENODEV;

	State currentState = state_.load(std::memory_order_acquire);
	if (currentState == state)
		return 0;

	ASSERT(static_cast<unsigned int>(state) < ARRAY_SIZE(camera_state_names));

	LOG(Camera, Debug) << "Camera in " << camera_state_names[currentState]
			   << " state trying operation requiring state "
			   << camera_state_names[state];

	return -EACCES;
}

int Camera::Private::isAccessAllowed(State low, State high,
				     bool allowDisconnected) const
{
	if (!allowDisconnected && disconnected_)
		return -ENODEV;

	State currentState = state_.load(std::memory_order_acquire);
	if (currentState >= low && currentState <= high)
		return 0;

	ASSERT(static_cast<unsigned int>(low) < ARRAY_SIZE(camera_state_names) &&
	       static_cast<unsigned int>(high) < ARRAY_SIZE(camera_state_names));

	LOG(Camera, Debug) << "Camera in " << camera_state_names[currentState]
			   << " state trying operation requiring state between "
			   << camera_state_names[low] << " and "
			   << camera_state_names[high];

	return -EACCES;
}

void Camera::Private::disconnect()
{
	/*
	 * If the camera was running when the hardware was removed force the
	 * state to Configured state to allow applications to free resources
	 * and call release() before deleting the camera.
	 */
	if (state_.load(std::memory_order_acquire) == Private::CameraRunning)
		state_.store(Private::CameraConfigured, std::memory_order_release);

	disconnected_ = true;
}

void Camera::Private::setState(State state)
{
	state_.store(state, std::memory_order_release);
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
 * it is ready to process requests. The camera needs to be acquired and
 * configured to prepare the camera for capture. Once started the camera can
 * process requests until it is stopped. When an application is done with a
 * camera, the camera needs to be released.
 *
 * An application may start and stop a camera multiple times as long as it is
 * not released. The camera may also be reconfigured.
 *
 * Functions that affect the camera state as defined below are generally not
 * synchronized with each other by the Camera class. The caller is responsible
 * for ensuring their synchronization if necessary.
 *
 * \subsection Camera States
 *
 * To help manage the sequence of operations needed to control the camera a set
 * of states are defined. Each state describes which operations may be performed
 * on the camera. Performing an operation not allowed in the camera state
 * results in undefined behaviour. Operations not listed at all in the state
 * diagram are allowed in all states.
 *
 * \dot
 * digraph camera_state_machine {
 *   node [shape = doublecircle ]; Available;
 *   node [shape = circle ]; Acquired;
 *   node [shape = circle ]; Configured;
 *   node [shape = circle ]; Running;
 *
 *   Available -> Available [label = "release()"];
 *   Available -> Acquired [label = "acquire()"];
 *
 *   Acquired -> Available [label = "release()"];
 *   Acquired -> Configured [label = "configure()"];
 *
 *   Configured -> Available [label = "release()"];
 *   Configured -> Configured [label = "configure(), createRequest()"];
 *   Configured -> Running [label = "start()"];
 *
 *   Running -> Configured [label = "stop()"];
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
 * The camera is configured and ready to be started. The application may
 * release() the camera and to get back to the Available state or start()
 * it to progress to the Running state.
 *
 * \subsubsection Running
 * The camera is running and ready to process requests queued by the
 * application. The camera remains in this state until it is stopped and moved
 * to the Configured state.
 */

/**
 * \brief Create a camera instance
 * \param[in] pipe The pipeline handler responsible for the camera device
 * \param[in] id The ID of the camera device
 * \param[in] streams Array of streams the camera provides
 *
 * The caller is responsible for guaranteeing a stable and unique camera ID
 * matching the constraints described by Camera::id(). Parameters that are
 * allocated dynamically at system startup, such as bus numbers that may be
 * enumerated differently, are therefore not suitable to use in the ID.
 *
 * Pipeline handlers that use a CameraSensor may use the CameraSensor::id() to
 * generate an ID that satisfies the criteria of a stable and unique camera ID.
 *
 * \return A shared pointer to the newly created camera object
 */
std::shared_ptr<Camera> Camera::create(PipelineHandler *pipe,
				       const std::string &id,
				       const std::set<Stream *> &streams)
{
	struct Deleter : std::default_delete<Camera> {
		void operator()(Camera *camera)
		{
			camera->deleteLater();
		}
	};

	Camera *camera = new Camera(pipe, id, streams);

	return std::shared_ptr<Camera>(camera, Deleter());
}

/**
 * \brief Retrieve the ID of the camera
 *
 * The camera ID is a free-form string that identifies a camera in the system.
 * IDs are guaranteed to be unique and stable: the same camera, when connected
 * to the system in the same way (e.g. in the same USB port), will have the same
 * ID across both unplug/replug and system reboots.
 *
 * Applications may store the camera ID and use it later to acquire the same
 * camera. They shall treat the ID as an opaque identifier, without interpreting
 * its value.
 *
 * Camera IDs may change when the system hardware or firmware is modified, for
 * instance when replacing a PCI USB controller or moving it to another PCI
 * slot, or updating the ACPI tables or Device Tree.
 *
 * \context This function is \threadsafe.
 *
 * \return ID of the camera device
 */
const std::string &Camera::id() const
{
	return p_->id_;
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

Camera::Camera(PipelineHandler *pipe, const std::string &id,
	       const std::set<Stream *> &streams)
	: p_(new Private(pipe, id, streams))
{
}

Camera::~Camera()
{
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
 */
void Camera::disconnect()
{
	LOG(Camera, Debug) << "Disconnecting camera " << id();

	p_->disconnect();
	disconnected.emit(this);
}

int Camera::exportFrameBuffers(Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	int ret = p_->isAccessAllowed(Private::CameraConfigured);
	if (ret < 0)
		return ret;

	if (streams().find(stream) == streams().end())
		return -EINVAL;

	if (p_->activeStreams_.find(stream) == p_->activeStreams_.end())
		return -EINVAL;

	return p_->pipe_->invokeMethod(&PipelineHandler::exportFrameBuffers,
				       ConnectionTypeBlocking, this, stream,
				       buffers);
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
 * \context This function is \threadsafe. It may only be called when the camera
 * is in the Available state as defined in \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EBUSY The camera is not free and can't be acquired by the caller
 */
int Camera::acquire()
{
	/*
	 * No manual locking is required as PipelineHandler::lock() is
	 * thread-safe.
	 */
	int ret = p_->isAccessAllowed(Private::CameraAvailable);
	if (ret < 0)
		return ret == -EACCES ? -EBUSY : ret;

	if (!p_->pipe_->lock()) {
		LOG(Camera, Info)
			<< "Pipeline handler in use by another process";
		return -EBUSY;
	}

	p_->setState(Private::CameraAcquired);

	return 0;
}

/**
 * \brief Release exclusive access to the camera device
 *
 * Releasing the camera device allows other users to acquire exclusive access
 * with the acquire() function.
 *
 * \context This function may only be called when the camera is in the
 * Available or Configured state as defined in \ref camera_operation, and shall
 * be synchronized by the caller with other functions that affect the camera
 * state.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EBUSY The camera is running and can't be released
 */
int Camera::release()
{
	int ret = p_->isAccessAllowed(Private::CameraAvailable,
				      Private::CameraConfigured, true);
	if (ret < 0)
		return ret == -EACCES ? -EBUSY : ret;

	p_->pipe_->unlock();

	p_->setState(Private::CameraAvailable);

	return 0;
}

/**
 * \brief Retrieve the list of controls supported by the camera
 *
 * The list of controls supported by the camera and their associated
 * constraints remain constant through the lifetime of the Camera object.
 *
 * \context This function is \threadsafe.
 *
 * \return A ControlInfoMap listing the controls supported by the camera
 */
const ControlInfoMap &Camera::controls()
{
	return p_->pipe_->controls(this);
}

/**
 * \brief Retrieve the list of properties of the camera
 *
 * Camera properties are static information that describe the capabilities of
 * the camera. They remain constant through the lifetime of the Camera object.
 *
 * \return A ControlList of properties supported by the camera
 */
const ControlList &Camera::properties()
{
	return p_->pipe_->properties(this);
}

/**
 * \brief Retrieve all the camera's stream information
 *
 * Retrieve all of the camera's static stream information. The static
 * information describes among other things how many streams the camera
 * supports and the capabilities of each stream.
 *
 * \context This function is \threadsafe.
 *
 * \return An array of all the camera's streams
 */
const std::set<Stream *> &Camera::streams() const
{
	return p_->streams_;
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
 * \context This function is \threadsafe.
 *
 * \return A CameraConfiguration if the requested roles can be satisfied, or a
 * null pointer otherwise. The ownership of the returned configuration is
 * passed to the caller.
 */
std::unique_ptr<CameraConfiguration> Camera::generateConfiguration(const StreamRoles &roles)
{
	int ret = p_->isAccessAllowed(Private::CameraAvailable,
				      Private::CameraRunning);
	if (ret < 0)
		return nullptr;

	if (roles.size() > streams().size())
		return nullptr;

	CameraConfiguration *config = p_->pipe_->generateConfiguration(this, roles);
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
 * \context This function may only be called when the camera is in the Acquired
 * or Configured state as defined in \ref camera_operation, and shall be
 * synchronized by the caller with other functions that affect the camera
 * state.
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
	int ret = p_->isAccessAllowed(Private::CameraAcquired,
				      Private::CameraConfigured);
	if (ret < 0)
		return ret;

	for (auto it : *config)
		it.setStream(nullptr);

	if (config->validate() != CameraConfiguration::Valid) {
		LOG(Camera, Error)
			<< "Can't configure camera with invalid configuration";
		return -EINVAL;
	}

	std::ostringstream msg("configuring streams:", std::ios_base::ate);

	for (unsigned int index = 0; index < config->size(); ++index) {
		StreamConfiguration &cfg = config->at(index);
		msg << " (" << index << ") " << cfg.toString();
	}

	LOG(Camera, Info) << msg.str();

	ret = p_->pipe_->invokeMethod(&PipelineHandler::configure,
				      ConnectionTypeBlocking, this, config);
	if (ret)
		return ret;

	p_->activeStreams_.clear();
	for (const StreamConfiguration &cfg : *config) {
		Stream *stream = cfg.stream();
		if (!stream) {
			LOG(Camera, Fatal)
				<< "Pipeline handler failed to update stream configuration";
			p_->activeStreams_.clear();
			return -EINVAL;
		}

		stream->configuration_ = cfg;
		p_->activeStreams_.insert(stream);
	}

	p_->setState(Private::CameraConfigured);

	return 0;
}

/**
 * \brief Create a request object for the camera
 * \param[in] cookie Opaque cookie for application use
 *
 * This method creates an empty request for the application to fill with
 * buffers and parameters, and queue for capture.
 *
 * The \a cookie is stored in the request and is accessible through the
 * Request::cookie() method at any time. It is typically used by applications
 * to map the request to an external resource in the request completion
 * handler, and is completely opaque to libcamera.
 *
 * The ownership of the returned request is passed to the caller, which is
 * responsible for either queueing the request or deleting it.
 *
 * \context This function is \threadsafe. It may only be called when the camera
 * is in the Configured or Running state as defined in \ref camera_operation.
 *
 * \return A pointer to the newly created request, or nullptr on error
 */
Request *Camera::createRequest(uint64_t cookie)
{
	int ret = p_->isAccessAllowed(Private::CameraConfigured,
				      Private::CameraRunning);
	if (ret < 0)
		return nullptr;

	return new Request(this, cookie);
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
 * \context This function is \threadsafe. It may only be called when the camera
 * is in the Running state as defined in \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not running so requests can't be queued
 * \retval -EINVAL The request is invalid
 * \retval -ENOMEM No buffer memory was available to handle the request
 */
int Camera::queueRequest(Request *request)
{
	int ret = p_->isAccessAllowed(Private::CameraRunning);
	if (ret < 0)
		return ret;

	/*
	 * The camera state may chance until the end of the function. No locking
	 * is however needed as PipelineHandler::queueRequest() will handle
	 * this.
	 */

	if (request->buffers().empty()) {
		LOG(Camera, Error) << "Request contains no buffers";
		return -EINVAL;
	}

	for (auto const &it : request->buffers()) {
		const Stream *stream = it.first;

		if (p_->activeStreams_.find(stream) == p_->activeStreams_.end()) {
			LOG(Camera, Error) << "Invalid request";
			return -EINVAL;
		}
	}

	return p_->pipe_->invokeMethod(&PipelineHandler::queueRequest,
				       ConnectionTypeQueued, this, request);
}

/**
 * \brief Start capture from camera
 *
 * Start the camera capture session. Once the camera is started the application
 * can queue requests to the camera to process and return to the application
 * until the capture session is terminated with \a stop().
 *
 * \context This function may only be called when the camera is in the
 * Configured state as defined in \ref camera_operation, and shall be
 * synchronized by the caller with other functions that affect the camera
 * state.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not in a state where it can be started
 */
int Camera::start()
{
	int ret = p_->isAccessAllowed(Private::CameraConfigured);
	if (ret < 0)
		return ret;

	LOG(Camera, Debug) << "Starting capture";

	ret = p_->pipe_->invokeMethod(&PipelineHandler::start,
				      ConnectionTypeBlocking, this);
	if (ret)
		return ret;

	p_->setState(Private::CameraRunning);

	return 0;
}

/**
 * \brief Stop capture from camera
 *
 * This method stops capturing and processing requests immediately. All pending
 * requests are cancelled and complete synchronously in an error state.
 *
 * \context This function may only be called when the camera is in the Running
 * state as defined in \ref camera_operation, and shall be synchronized by the
 * caller with other functions that affect the camera state.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not running so can't be stopped
 */
int Camera::stop()
{
	int ret = p_->isAccessAllowed(Private::CameraRunning);
	if (ret < 0)
		return ret;

	LOG(Camera, Debug) << "Stopping capture";

	p_->setState(Private::CameraConfigured);

	p_->pipe_->invokeMethod(&PipelineHandler::stop, ConnectionTypeBlocking,
				this);

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
	requestCompleted.emit(request);
	delete request;
}

} /* namespace libcamera */
