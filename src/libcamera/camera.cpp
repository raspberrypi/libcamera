/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.cpp - Camera device
 */

#include <libcamera/camera.h>

#include <array>
#include <atomic>
#include <iomanip>

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include <libcamera/color_space.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_controls.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"

/**
 * \file libcamera/camera.h
 * \brief Camera device handling
 *
 * \page camera-model Camera Model
 *
 * libcamera acts as a middleware between applications and camera hardware. It
 * provides a solution to an unsolvable problem: reconciling applications,
 * which need to run on different systems without dealing with device-specific
 * details, and camera hardware, which exhibits a wide variety of features,
 * limitations and architecture variations. In order to do so, it creates an
 * abstract camera model that hides the camera hardware from applications. The
 * model is designed to strike the right balance between genericity, to please
 * generic applications, and flexibility, to expose even the most specific
 * hardware features to the most demanding applications.
 *
 * In libcamera, a Camera is defined as a device that can capture frames
 * continuously from a camera sensor and store them in memory. If supported by
 * the device and desired by the application, the camera may store each
 * captured frame in multiple copies, possibly in different formats and sizes.
 * Each of these memory outputs of the camera is called a Stream.
 *
 * A camera contains a single image source, and separate camera instances
 * relate to different image sources. For instance, a phone containing front
 * and back image sensors will be modelled with two cameras, one for each
 * sensor. When multiple streams can be produced from the same image source,
 * all those streams are guaranteed to be part of the same camera.
 *
 * While not sharing image sources, separate cameras can share other system
 * resources, such as ISPs. For this reason camera instances may not be fully
 * independent, in which case usage restrictions may apply. For instance, a
 * phone with a front and a back camera may not allow usage of the two cameras
 * simultaneously.
 *
 * The camera model defines an implicit pipeline, whose input is the camera
 * sensor, and whose outputs are the streams. Along the pipeline, the frames
 * produced by the camera sensor are transformed by the camera into a format
 * suitable for applications, with image processing that improves the quality
 * of the captured frames. The camera exposes a set of controls that
 * applications may use to manually control the processing steps. This
 * high-level camera model is the minimum baseline that all cameras must
 * conform to.
 *
 * \section camera-pipeline-model Pipeline Model
 *
 * Camera hardware differs in the supported image processing operations and the
 * order in which they are applied. The libcamera pipelines abstract the
 * hardware differences and expose a logical view of the processing operations
 * with a fixed order. This offers low-level control of those operations to
 * applications, while keeping application code generic.
 *
 * Starting from the camera sensor, a pipeline applies the following
 * operations, in that order.
 *
 * - Pixel exposure
 * - Analog to digital conversion and readout
 * - Black level subtraction
 * - Defective pixel correction
 * - Lens shading correction
 * - Spatial noise filtering
 * - Per-channel gains (white balance)
 * - Demosaicing (color filter array interpolation)
 * - Color correction matrix (typically RGB to RGB)
 * - Gamma correction
 * - Color space transformation (typically RGB to YUV)
 * - Cropping
 * - Scaling
 *
 * Not all cameras implement all operations, and they are not necessarily
 * implemented in the above order at the hardware level. The libcamera pipeline
 * handlers translate the pipeline model to the real hardware configuration.
 *
 * \subsection digital-zoom Digital Zoom
 *
 * Digital zoom is implemented as a combination of the cropping and scaling
 * stages of the pipeline. Cropping is controlled explicitly through the
 * controls::ScalerCrop control, while scaling is controlled implicitly based
 * on the crop rectangle and the output stream size. The crop rectangle is
 * expressed relatively to the full pixel array size and indicates how the field
 * of view is affected by the pipeline.
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(Camera)

/**
 * \class CameraConfiguration
 * \brief Hold configuration for streams of the camera

 * The CameraConfiguration holds an ordered list of stream configurations. It
 * supports iterators and operates as a vector of StreamConfiguration instances.
 * The stream configurations are inserted by addConfiguration(), and the
 * at() function or operator[] return a reference to the StreamConfiguration
 * based on its insertion index. Accessing a stream configuration with an
 * invalid index results in undefined behaviour.
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
	: transform(Transform::Identity), config_({})
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
 * This function adjusts the camera configuration to the closest valid
 * configuration and returns the validation status.
 *
 * \todo Define exactly when to return each status code. Should stream
 * parameters set to 0 by the caller be adjusted without returning Adjusted ?
 * This would potentially be useful for applications but would get in the way
 * in Camera::configure(). Do we need an extra status code to signal this ?
 *
 * \todo Handle validation of buffers count when refactoring the buffers API.
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
 * this function with an invalid index results in undefined behaviour.
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
 * this function with an invalid index results in undefined behaviour.
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
 * this function with an invalid index results in undefined behaviour.
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
 * this function with an invalid index results in undefined behaviour.
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
 * \enum CameraConfiguration::ColorSpaceFlag
 * \brief Specify the behaviour of validateColorSpaces
 * \var CameraConfiguration::ColorSpaceFlag::None
 * \brief No extra validation of color spaces is required
 * \var CameraConfiguration::ColorSpaceFlag::StreamsShareColorSpace
 * \brief Non-raw output streams must share the same color space
 */

/**
 * \typedef CameraConfiguration::ColorSpaceFlags
 * \brief A bitwise combination of ColorSpaceFlag values
 */

/**
 * \brief Check the color spaces requested for each stream
 * \param[in] flags Flags to control the behaviour of this function
 *
 * This function performs certain consistency checks on the color spaces of
 * the streams and may adjust them so that:
 *
 * - Any raw streams have the Raw color space
 * - If the StreamsShareColorSpace flag is set, all output streams are forced
 * to share the same color space (this may be a constraint on some platforms).
 *
 * It is optional for a pipeline handler to use this function.
 *
 * \return A CameraConfiguration::Status value that describes the validation
 * status.
 * \retval CameraConfigutation::Adjusted The configuration has been adjusted
 * and is now valid. The color space of some or all of the streams may have
 * been changed. The caller shall check the color spaces carefully.
 * \retval CameraConfiguration::Valid The configuration was already valid and
 * hasn't been adjusted.
 */
CameraConfiguration::Status CameraConfiguration::validateColorSpaces(ColorSpaceFlags flags)
{
	Status status = Valid;

	/*
	 * Set all raw streams to the Raw color space, and make a note of the
	 * largest non-raw stream with a defined color space (if there is one).
	 */
	std::optional<ColorSpace> colorSpace;

	for (auto [i, cfg] : utils::enumerate(config_)) {
		if (!cfg.colorSpace)
			continue;

		if (cfg.colorSpace->adjust(cfg.pixelFormat))
			status = Adjusted;

		if (cfg.colorSpace != ColorSpace::Raw &&
		    (!colorSpace || cfg.size > config_[i].size))
			colorSpace = cfg.colorSpace;
	}

	if (!colorSpace || !(flags & ColorSpaceFlag::StreamsShareColorSpace))
		return status;

	/* Make all output color spaces the same, if requested. */
	for (auto &cfg : config_) {
		if (cfg.colorSpace != ColorSpace::Raw &&
		    cfg.colorSpace != colorSpace) {
			cfg.colorSpace = colorSpace;
			status = Adjusted;
		}
	}

	return status;
}

/**
 * \var CameraConfiguration::transform
 * \brief User-specified transform to be applied to the image
 *
 * The transform is a user-specified 2D plane transform that will be applied
 * to the camera images by the processing pipeline before being handed to
 * the application. This is subsequent to any transform that is already
 * required to fix up any platform-defined rotation.
 *
 * The usual 2D plane transforms are allowed here (horizontal/vertical
 * flips, multiple of 90-degree rotations etc.), but the validate() function
 * may adjust this field at its discretion if the selection is not supported.
 */

/**
 * \var CameraConfiguration::config_
 * \brief The vector of stream configurations
 */

/**
 * \class Camera::Private
 * \brief Base class for camera private data
 *
 * The Camera::Private class stores all private data associated with a camera.
 * In addition to hiding core Camera data from the public API, it is expected to
 * be subclassed by pipeline handlers to store pipeline-specific data.
 *
 * Pipeline handlers can obtain the Camera::Private instance associated with a
 * camera by calling Camera::_d().
 */

/**
 * \brief Construct a Camera::Private instance
 * \param[in] pipe The pipeline handler responsible for the camera device
 */
Camera::Private::Private(PipelineHandler *pipe)
	: requestSequence_(0), pipe_(pipe->shared_from_this()),
	  disconnected_(false), state_(CameraAvailable)
{
}

Camera::Private::~Private()
{
	if (state_.load(std::memory_order_acquire) != Private::CameraAvailable)
		LOG(Camera, Error) << "Removing camera while still in use";
}

/**
 * \fn Camera::Private::pipe()
 * \brief Retrieve the pipeline handler related to this camera
 * \return The pipeline handler that created this camera
 */

/**
 * \fn Camera::Private::validator()
 * \brief Retrieve the control validator related to this camera
 * \return The control validator associated with this camera
 */

/**
 * \var Camera::Private::queuedRequests_
 * \brief The list of queued and not yet completed requests
 *
 * This list tracks requests queued in order to ensure completion of all
 * requests when the pipeline handler is stopped.
 *
 * \sa PipelineHandler::queueRequest(), PipelineHandler::stop(),
 * PipelineHandler::completeRequest()
 */

/**
 * \var Camera::Private::controlInfo_
 * \brief The set of controls supported by the camera
 *
 * The control information shall be initialised by the pipeline handler when
 * creating the camera.
 *
 * \todo This member was initially meant to stay constant after the camera is
 * created. Several pipeline handlers are already updating it when the camera
 * is configured. Update the documentation accordingly, and possibly the API as
 * well, when implementing official support for control info updates.
 */

/**
 * \var Camera::Private::properties_
 * \brief The list of properties supported by the camera
 *
 * The list of camera properties shall be initialised by the pipeline handler
 * when creating the camera, and shall not be modified afterwards.
 */

/**
 * \var Camera::Private::requestSequence_
 * \brief The queuing sequence number of the request
 *
 * When requests are queued, they are given a per-camera sequence number to
 * facilitate debugging of internal request usage.
 *
 * The requestSequence_ tracks the number of requests queued to a camera
 * over a single capture session.
 */

static const char *const camera_state_names[] = {
	"Available",
	"Acquired",
	"Configured",
	"Stopping",
	"Running",
};

bool Camera::Private::isAcquired() const
{
	return state_.load(std::memory_order_acquire) != CameraAvailable;
}

bool Camera::Private::isRunning() const
{
	return state_.load(std::memory_order_acquire) == CameraRunning;
}

int Camera::Private::isAccessAllowed(State state, bool allowDisconnected,
				     const char *from) const
{
	if (!allowDisconnected && disconnected_)
		return -ENODEV;

	State currentState = state_.load(std::memory_order_acquire);
	if (currentState == state)
		return 0;

	ASSERT(static_cast<unsigned int>(state) < std::size(camera_state_names));

	LOG(Camera, Error) << "Camera in " << camera_state_names[currentState]
			   << " state trying " << from << "() requiring state "
			   << camera_state_names[state];

	return -EACCES;
}

int Camera::Private::isAccessAllowed(State low, State high,
				     bool allowDisconnected,
				     const char *from) const
{
	if (!allowDisconnected && disconnected_)
		return -ENODEV;

	State currentState = state_.load(std::memory_order_acquire);
	if (currentState >= low && currentState <= high)
		return 0;

	ASSERT(static_cast<unsigned int>(low) < std::size(camera_state_names) &&
	       static_cast<unsigned int>(high) < std::size(camera_state_names));

	LOG(Camera, Error) << "Camera in " << camera_state_names[currentState]
			   << " state trying " << from
			   << "() requiring state between "
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
 *   node [shape = circle ]; Stopping;
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
 *   Running -> Stopping [label = "stop()"];
 *   Stopping -> Configured;
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
 * \subsubsection Stopping
 * The camera has been asked to stop. Pending requests are being completed or
 * cancelled, and no new requests are permitted to be queued. The camera will
 * transition to the Configured state when all queued requests have been
 * returned to the application.
 *
 * \subsubsection Running
 * The camera is running and ready to process requests queued by the
 * application. The camera remains in this state until it is stopped and moved
 * to the Configured state.
 */

/**
 * \brief Create a camera instance
 * \param[in] d Camera private data
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
std::shared_ptr<Camera> Camera::create(std::unique_ptr<Private> d,
				       const std::string &id,
				       const std::set<Stream *> &streams)
{
	ASSERT(d);

	struct Deleter : std::default_delete<Camera> {
		void operator()(Camera *camera)
		{
			if (Thread::current() == camera->thread())
				delete camera;
			else
				camera->deleteLater();
		}
	};

	Camera *camera = new Camera(std::move(d), id, streams);

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
	return _d()->id_;
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

Camera::Camera(std::unique_ptr<Private> d, const std::string &id,
	       const std::set<Stream *> &streams)
	: Extensible(std::move(d))
{
	_d()->id_ = id;
	_d()->streams_ = streams;
	_d()->validator_ = std::make_unique<CameraControlValidator>(this);
}

Camera::~Camera()
{
}

/**
 * \brief Notify camera disconnection
 *
 * This function is used to notify the camera instance that the underlying
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

	_d()->disconnect();
	disconnected.emit();
}

int Camera::exportFrameBuffers(Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	Private *const d = _d();

	int ret = d->isAccessAllowed(Private::CameraConfigured);
	if (ret < 0)
		return ret;

	if (streams().find(stream) == streams().end())
		return -EINVAL;

	if (d->activeStreams_.find(stream) == d->activeStreams_.end())
		return -EINVAL;

	return d->pipe_->invokeMethod(&PipelineHandler::exportFrameBuffers,
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
 * Acquiring a camera may limit usage of any other camera(s) provided by the
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
	Private *const d = _d();

	/*
	 * No manual locking is required as PipelineHandler::lock() is
	 * thread-safe.
	 */
	int ret = d->isAccessAllowed(Private::CameraAvailable);
	if (ret < 0)
		return ret == -EACCES ? -EBUSY : ret;

	if (!d->pipe_->acquire()) {
		LOG(Camera, Info)
			<< "Pipeline handler in use by another process";
		return -EBUSY;
	}

	d->setState(Private::CameraAcquired);

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
	Private *const d = _d();

	int ret = d->isAccessAllowed(Private::CameraAvailable,
				     Private::CameraConfigured, true);
	if (ret < 0)
		return ret == -EACCES ? -EBUSY : ret;

	if (d->isAcquired())
		d->pipe_->release(this);

	d->setState(Private::CameraAvailable);

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
const ControlInfoMap &Camera::controls() const
{
	return _d()->controlInfo_;
}

/**
 * \brief Retrieve the list of properties of the camera
 *
 * Camera properties are static information that describe the capabilities of
 * the camera. They remain constant through the lifetime of the Camera object.
 *
 * \return A ControlList of properties supported by the camera
 */
const ControlList &Camera::properties() const
{
	return _d()->properties_;
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
	return _d()->streams_;
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
 * null pointer otherwise.
 */
std::unique_ptr<CameraConfiguration> Camera::generateConfiguration(const StreamRoles &roles)
{
	Private *const d = _d();

	int ret = d->isAccessAllowed(Private::CameraAvailable,
				     Private::CameraRunning);
	if (ret < 0)
		return nullptr;

	if (roles.size() > streams().size())
		return nullptr;

	std::unique_ptr<CameraConfiguration> config =
		d->pipe_->generateConfiguration(this, roles);
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
 * The configuration is created by generateConfiguration(), and adjusted by the
 * caller with CameraConfiguration::validate(). This function only accepts fully
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
	Private *const d = _d();

	int ret = d->isAccessAllowed(Private::CameraAcquired,
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

	ret = d->pipe_->invokeMethod(&PipelineHandler::configure,
				     ConnectionTypeBlocking, this, config);
	if (ret)
		return ret;

	d->activeStreams_.clear();
	for (const StreamConfiguration &cfg : *config) {
		Stream *stream = cfg.stream();
		if (!stream) {
			LOG(Camera, Fatal)
				<< "Pipeline handler failed to update stream configuration";
			d->activeStreams_.clear();
			return -EINVAL;
		}

		stream->configuration_ = cfg;
		d->activeStreams_.insert(stream);
	}

	d->setState(Private::CameraConfigured);

	return 0;
}

/**
 * \brief Create a request object for the camera
 * \param[in] cookie Opaque cookie for application use
 *
 * This function creates an empty request for the application to fill with
 * buffers and parameters, and queue for capture.
 *
 * The \a cookie is stored in the request and is accessible through the
 * Request::cookie() function at any time. It is typically used by applications
 * to map the request to an external resource in the request completion
 * handler, and is completely opaque to libcamera.
 *
 * The ownership of the returned request is passed to the caller, which is
 * responsible for deleting it. The request may be deleted in the completion
 * handler, or reused after resetting its state with Request::reuse().
 *
 * \context This function is \threadsafe. It may only be called when the camera
 * is in the Configured or Running state as defined in \ref camera_operation.
 *
 * \return A pointer to the newly created request, or nullptr on error
 */
std::unique_ptr<Request> Camera::createRequest(uint64_t cookie)
{
	Private *const d = _d();

	int ret = d->isAccessAllowed(Private::CameraConfigured,
				     Private::CameraRunning);
	if (ret < 0)
		return nullptr;

	std::unique_ptr<Request> request = std::make_unique<Request>(this, cookie);

	/* Associate the request with the pipeline handler. */
	d->pipe_->registerRequest(request.get());

	return request;
}

/**
 * \brief Queue a request to the camera
 * \param[in] request The request to queue to the camera
 *
 * This function queues a \a request to the camera for capture.
 *
 * After allocating the request with createRequest(), the application shall
 * fill it with at least one capture buffer before queuing it. Requests that
 * contain no buffers are invalid and are rejected without being queued.
 *
 * Once the request has been queued, the camera will notify its completion
 * through the \ref requestCompleted signal.
 *
 * \context This function is \threadsafe. It may only be called when the camera
 * is in the Running state as defined in \ref camera_operation.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not running so requests can't be queued
 * \retval -EXDEV The request does not belong to this camera
 * \retval -EINVAL The request is invalid
 * \retval -ENOMEM No buffer memory was available to handle the request
 */
int Camera::queueRequest(Request *request)
{
	Private *const d = _d();

	int ret = d->isAccessAllowed(Private::CameraRunning);
	if (ret < 0)
		return ret;

	/* Requests can only be queued to the camera that created them. */
	if (request->_d()->camera() != this) {
		LOG(Camera, Error) << "Request was not created by this camera";
		return -EXDEV;
	}

	/*
	 * The camera state may change until the end of the function. No locking
	 * is however needed as PipelineHandler::queueRequest() will handle
	 * this.
	 */

	if (request->buffers().empty()) {
		LOG(Camera, Error) << "Request contains no buffers";
		return -EINVAL;
	}

	for (auto const &it : request->buffers()) {
		const Stream *stream = it.first;

		if (d->activeStreams_.find(stream) == d->activeStreams_.end()) {
			LOG(Camera, Error) << "Invalid request";
			return -EINVAL;
		}
	}

	d->pipe_->invokeMethod(&PipelineHandler::queueRequest,
			       ConnectionTypeQueued, request);

	return 0;
}

/**
 * \brief Start capture from camera
 * \param[in] controls Controls to be applied before starting the Camera
 *
 * Start the camera capture session, optionally providing a list of controls to
 * apply before starting. Once the camera is started the application can queue
 * requests to the camera to process and return to the application until the
 * capture session is terminated with \a stop().
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
int Camera::start(const ControlList *controls)
{
	Private *const d = _d();

	int ret = d->isAccessAllowed(Private::CameraConfigured);
	if (ret < 0)
		return ret;

	LOG(Camera, Debug) << "Starting capture";

	ASSERT(d->requestSequence_ == 0);

	ret = d->pipe_->invokeMethod(&PipelineHandler::start,
				     ConnectionTypeBlocking, this, controls);
	if (ret)
		return ret;

	d->setState(Private::CameraRunning);

	return 0;
}

/**
 * \brief Stop capture from camera
 *
 * This function stops capturing and processing requests immediately. All
 * pending requests are cancelled and complete synchronously in an error state.
 *
 * \context This function may be called in any camera state as defined in \ref
 * camera_operation, and shall be synchronized by the caller with other
 * functions that affect the camera state. If called when the camera isn't
 * running, it is a no-op.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV The camera has been disconnected from the system
 * \retval -EACCES The camera is not running so can't be stopped
 */
int Camera::stop()
{
	Private *const d = _d();

	/*
	 * \todo Make calling stop() when not in 'Running' part of the state
	 * machine rather than take this shortcut
	 */
	if (!d->isRunning())
		return 0;

	int ret = d->isAccessAllowed(Private::CameraRunning);
	if (ret < 0)
		return ret;

	LOG(Camera, Debug) << "Stopping capture";

	d->setState(Private::CameraStopping);

	d->pipe_->invokeMethod(&PipelineHandler::stop, ConnectionTypeBlocking,
			       this);

	ASSERT(!d->pipe_->hasPendingRequests(this));

	d->setState(Private::CameraConfigured);

	return 0;
}

/**
 * \brief Handle request completion and notify application
 * \param[in] request The request that has completed
 *
 * This function is called by the pipeline handler to notify the camera that
 * the request has completed. It emits the requestCompleted signal.
 */
void Camera::requestComplete(Request *request)
{
	/* Disconnected cameras are still able to complete requests. */
	if (_d()->isAccessAllowed(Private::CameraStopping, Private::CameraRunning,
				  true))
		LOG(Camera, Fatal) << "Trying to complete a request when stopped";

	requestCompleted.emit(request);
}

} /* namespace libcamera */
