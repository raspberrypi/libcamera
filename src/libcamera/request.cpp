/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * request.cpp - Capture request handling
 */

#include "libcamera/internal/request.h"

#include <map>
#include <sstream>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/fence.h>
#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_controls.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/tracepoints.h"

/**
 * \file libcamera/request.h
 * \brief Describes a frame capture request to be processed by a camera
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Request)

/**
 * \class Request::Private
 * \brief Request private data
 *
 * The Request::Private class stores all private data associated with a
 * request. It implements the d-pointer design pattern to hide core
 * Request data from the public API, and exposes utility functions to
 * internal users of the request (namely the PipelineHandler class and its
 * subclasses).
 */

/**
 * \brief Create a Request::Private
 * \param camera The Camera that creates the request
 */
Request::Private::Private(Camera *camera)
	: camera_(camera), cancelled_(false)
{
}

Request::Private::~Private()
{
	doCancelRequest();
}

/**
 * \fn Request::Private::camera()
 * \brief Retrieve the camera this request has been queued to
 * \return The Camera this request has been queued to, or nullptr if the
 * request hasn't been queued
 */

/**
 * \brief Check if a request has buffers yet to be completed
 *
 * \return True if the request has buffers pending for completion, false
 * otherwise
 */
bool Request::Private::hasPendingBuffers() const
{
	return !pending_.empty();
}

/**
 * \brief Complete a buffer for the request
 * \param[in] buffer The buffer that has completed
 *
 * A request tracks the status of all buffers it contains through a set of
 * pending buffers. This function removes the \a buffer from the set to mark it
 * as complete. All buffers associate with the request shall be marked as
 * complete by calling this function once and once only before reporting the
 * request as complete with the complete() function.
 *
 * \return True if all buffers contained in the request have completed, false
 * otherwise
 */
bool Request::Private::completeBuffer(FrameBuffer *buffer)
{
	LIBCAMERA_TRACEPOINT(request_complete_buffer, this, buffer);

	int ret = pending_.erase(buffer);
	ASSERT(ret == 1);

	buffer->_d()->setRequest(nullptr);

	if (buffer->metadata().status == FrameMetadata::FrameCancelled)
		cancelled_ = true;

	return !hasPendingBuffers();
}

/**
 * \brief Complete a queued request
 *
 * Mark the request as complete by updating its status to RequestComplete,
 * unless buffers have been cancelled in which case the status is set to
 * RequestCancelled.
 */
void Request::Private::complete()
{
	Request *request = _o<Request>();

	ASSERT(request->status() == RequestPending);
	ASSERT(!hasPendingBuffers());

	request->status_ = cancelled_ ? RequestCancelled : RequestComplete;

	LOG(Request, Debug) << request->toString();

	LIBCAMERA_TRACEPOINT(request_complete, this);
}

void Request::Private::doCancelRequest()
{
	Request *request = _o<Request>();

	for (FrameBuffer *buffer : pending_) {
		buffer->_d()->cancel();
		camera_->bufferCompleted.emit(request, buffer);
	}

	cancelled_ = true;
	pending_.clear();
	notifiers_.clear();
	timer_.reset();
}

/**
 * \brief Cancel a queued request
 *
 * Mark the request and its associated buffers as cancelled and complete it.
 *
 * Set each pending buffer in error state and emit the buffer completion signal
 * before completing the Request.
 */
void Request::Private::cancel()
{
	LIBCAMERA_TRACEPOINT(request_cancel, this);

	Request *request = _o<Request>();
	ASSERT(request->status() == RequestPending);

	doCancelRequest();
}

/**
 * \brief Reset the request internal data to default values
 *
 * After calling this function, all request internal data will have default
 * values as if the Request::Private instance had just been constructed.
 */
void Request::Private::reset()
{
	sequence_ = 0;
	cancelled_ = false;
	prepared_ = false;
	pending_.clear();
	notifiers_.clear();
	timer_.reset();
}

/*
 * Helper function to save some lines of code and make sure prepared_ is set
 * to true before emitting the signal.
 */
void Request::Private::emitPrepareCompleted()
{
	prepared_ = true;
	prepared.emit();
}

/**
 * \brief Prepare the Request to be queued to the device
 * \param[in] timeout Optional expiration timeout
 *
 * Prepare a Request to be queued to the hardware device by ensuring it is
 * ready for the incoming memory transfers.
 *
 * This currently means waiting on each frame buffer acquire fence to be
 * signalled. An optional expiration timeout can be specified. If not all the
 * fences have been signalled correctly before the timeout expires the Request
 * is cancelled.
 *
 * The function immediately emits the prepared signal if all the prepare
 * operations have been completed synchronously. If instead the prepare
 * operations require to wait the completion of asynchronous events, such as
 * fences notifications or timer expiration, the prepared signal is emitted upon
 * the asynchronous event completion.
 *
 * As we currently only handle fences, the function emits the prepared signal
 * immediately if there are no fences to wait on. Otherwise the prepared signal
 * is emitted when all fences have been signalled or the optional timeout has
 * expired.
 *
 * If not all the fences have been correctly signalled or the optional timeout
 * has expired the Request will be cancelled and the Request::prepared signal
 * emitted.
 *
 * The intended user of this function is the PipelineHandler base class, which
 * 'prepares' a Request before queuing it to the hardware device.
 */
void Request::Private::prepare(std::chrono::milliseconds timeout)
{
	/* Create and connect one notifier for each synchronization fence. */
	for (FrameBuffer *buffer : pending_) {
		const Fence *fence = buffer->_d()->fence();
		if (!fence)
			continue;

		std::unique_ptr<EventNotifier> notifier =
			std::make_unique<EventNotifier>(fence->fd().get(),
							EventNotifier::Read);

		notifier->activated.connect(this, [this, buffer] {
							notifierActivated(buffer);
					    });

		notifiers_[buffer] = std::move(notifier);
	}

	if (notifiers_.empty()) {
		emitPrepareCompleted();
		return;
	}

	/*
	 * In case a timeout is specified, create a timer and set it up.
	 *
	 * The timer must be created here instead of in the Request constructor,
	 * in order to be bound to the pipeline handler thread.
	 */
	if (timeout != 0ms) {
		timer_ = std::make_unique<Timer>();
		timer_->timeout.connect(this, &Request::Private::timeout);
		timer_->start(timeout);
	}
}

/**
 * \var Request::Private::prepared
 * \brief Request preparation completed Signal
 *
 * The signal is emitted once the request preparation has completed and is ready
 * to be queued. The Request might complete with errors in which case it is
 * cancelled.
 *
 * The intended slot for this signal is the PipelineHandler::doQueueRequests()
 * function which queues Request after they have been prepared or cancel them
 * if they have failed preparing.
 */

void Request::Private::notifierActivated(FrameBuffer *buffer)
{
	/* Close the fence if successfully signalled. */
	ASSERT(buffer);
	buffer->releaseFence();

	/* Remove the entry from the map and check if other fences are pending. */
	auto it = notifiers_.find(buffer);
	ASSERT(it != notifiers_.end());
	notifiers_.erase(it);

	Request *request = _o<Request>();
	LOG(Request, Debug)
		<< "Request " << request->cookie() << " buffer " << buffer
		<< " fence signalled";

	if (!notifiers_.empty())
		return;

	/* All fences completed, delete the timer and emit the prepared signal. */
	timer_.reset();
	emitPrepareCompleted();
}

void Request::Private::timeout()
{
	/* A timeout can only happen if there are fences not yet signalled. */
	ASSERT(!notifiers_.empty());
	notifiers_.clear();

	Request *request = _o<Request>();
	LOG(Request, Debug) << "Request prepare timeout: " << request->cookie();

	cancel();

	emitPrepareCompleted();
}

/**
 * \enum Request::Status
 * Request completion status
 * \var Request::RequestPending
 * The request hasn't completed yet
 * \var Request::RequestComplete
 * The request has completed
 * \var Request::RequestCancelled
 * The request has been cancelled due to capture stop
 */

/**
 * \enum Request::ReuseFlag
 * Flags to control the behavior of Request::reuse()
 * \var Request::Default
 * Don't reuse buffers
 * \var Request::ReuseBuffers
 * Reuse the buffers that were previously added by addBuffer()
 */

/**
 * \typedef Request::BufferMap
 * \brief A map of Stream to FrameBuffer pointers
 */

/**
 * \class Request
 * \brief A frame capture request
 *
 * A Request allows an application to associate buffers and controls on a
 * per-frame basis to be queued to the camera device for processing.
 */

/**
 * \brief Create a capture request for a camera
 * \param[in] camera The camera that creates the request
 * \param[in] cookie Opaque cookie for application use
 *
 * The \a cookie is stored in the request and is accessible through the
 * cookie() function at any time. It is typically used by applications to map
 * the request to an external resource in the request completion handler, and is
 * completely opaque to libcamera.
 */
Request::Request(Camera *camera, uint64_t cookie)
	: Extensible(std::make_unique<Private>(camera)),
	  cookie_(cookie), status_(RequestPending)
{
	controls_ = new ControlList(controls::controls,
				    camera->_d()->validator());

	/**
	 * \todo Add a validator for metadata controls.
	 */
	metadata_ = new ControlList(controls::controls);

	LIBCAMERA_TRACEPOINT(request_construct, this);

	LOG(Request, Debug) << "Created request - cookie: " << cookie_;
}

Request::~Request()
{
	LIBCAMERA_TRACEPOINT(request_destroy, this);

	delete metadata_;
	delete controls_;
}

/**
 * \brief Reset the request for reuse
 * \param[in] flags Indicate whether or not to reuse the buffers
 *
 * Reset the status and controls associated with the request, to allow it to
 * be reused and requeued without destruction. This function shall be called
 * prior to queueing the request to the camera, in lieu of constructing a new
 * request. The application can reuse the buffers that were previously added
 * to the request via addBuffer() by setting \a flags to ReuseBuffers.
 */
void Request::reuse(ReuseFlag flags)
{
	LIBCAMERA_TRACEPOINT(request_reuse, this);

	_d()->reset();

	if (flags & ReuseBuffers) {
		for (auto pair : bufferMap_) {
			FrameBuffer *buffer = pair.second;
			buffer->_d()->setRequest(this);
			_d()->pending_.insert(buffer);
		}
	} else {
		bufferMap_.clear();
	}

	status_ = RequestPending;

	controls_->clear();
	metadata_->clear();
}

/**
 * \fn Request::controls()
 * \brief Retrieve the request's ControlList
 *
 * Requests store a list of controls to be applied to all frames captured for
 * the request. They are created with an empty list of controls that can be
 * accessed through this function. Control values can be retrieved using
 * ControlList::get() and updated using ControlList::set().
 *
 * Only controls supported by the camera to which this request will be
 * submitted shall be included in the controls list. Attempting to add an
 * unsupported control causes undefined behaviour.
 *
 * \return A reference to the ControlList in this request
 */

/**
 * \fn Request::buffers()
 * \brief Retrieve the request's streams to buffers map
 *
 * Return a reference to the map that associates each Stream part of the
 * request to the FrameBuffer the Stream output should be directed to.
 *
 * \return The map of Stream to FrameBuffer
 */

/**
 * \brief Add a FrameBuffer with its associated Stream to the Request
 * \param[in] stream The stream the buffer belongs to
 * \param[in] buffer The FrameBuffer to add to the request
 * \param[in] fence The optional fence
 *
 * A reference to the buffer is stored in the request. The caller is responsible
 * for ensuring that the buffer will remain valid until the request complete
 * callback is called.
 *
 * A request can only contain one buffer per stream. If a buffer has already
 * been added to the request for the same stream, this function returns -EEXIST.
 *
 * A Fence can be optionally associated with the \a buffer.
 *
 * When a valid Fence is provided to this function, \a fence is moved to \a
 * buffer and this Request will only be queued to the device once the
 * fences of all its buffers have been correctly signalled.
 *
 * If the \a fence associated with \a buffer isn't signalled, the request will
 * fail after a timeout. The buffer will still contain the fence, which
 * applications must retrieve with FrameBuffer::releaseFence() before the buffer
 * can be reused in another request. Attempting to add a buffer that still
 * contains a fence to a request will result in this function returning -EEXIST.
 *
 * \sa FrameBuffer::releaseFence()
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EEXIST The request already contains a buffer for the stream
 *  or the buffer still references a fence
 * \retval -EINVAL The buffer does not reference a valid Stream
 */
int Request::addBuffer(const Stream *stream, FrameBuffer *buffer,
		       std::unique_ptr<Fence> fence)
{
	if (!stream) {
		LOG(Request, Error) << "Invalid stream reference";
		return -EINVAL;
	}

	auto it = bufferMap_.find(stream);
	if (it != bufferMap_.end()) {
		LOG(Request, Error) << "FrameBuffer already set for stream";
		return -EEXIST;
	}

	buffer->_d()->setRequest(this);
	_d()->pending_.insert(buffer);
	bufferMap_[stream] = buffer;

	/*
	 * Make sure the fence has been extracted from the buffer
	 * to avoid waiting on a stale fence.
	 */
	if (buffer->_d()->fence()) {
		LOG(Request, Error) << "Can't add buffer that still references a fence";
		return -EEXIST;
	}

	if (fence && fence->isValid())
		buffer->_d()->setFence(std::move(fence));

	return 0;
}

/**
 * \var Request::bufferMap_
 * \brief Mapping of streams to buffers for this request
 *
 * The bufferMap_ tracks the buffers associated with each stream. If a stream is
 * not utilised in this request there will be no buffer for that stream in the
 * map.
 */

/**
 * \brief Return the buffer associated with a stream
 * \param[in] stream The stream the buffer is associated to
 * \return The buffer associated with the stream, or nullptr if the stream is
 * not part of this request
 */
FrameBuffer *Request::findBuffer(const Stream *stream) const
{
	const auto it = bufferMap_.find(stream);
	if (it == bufferMap_.end())
		return nullptr;

	return it->second;
}

/**
 * \fn Request::metadata()
 * \brief Retrieve the request's metadata
 * \todo Offer a read-only API towards applications while keeping a read/write
 * API internally.
 * \return The metadata associated with the request
 */

/**
 * \brief Retrieve the sequence number for the request
 *
 * When requests are queued, they are given a sequential number to track the
 * order in which requests are queued to a camera. This number counts all
 * requests given to a camera and is reset to zero between camera stop/start
 * sequences.
 *
 * It can be used to support debugging and identifying the flow of requests
 * through a pipeline, but does not guarantee to represent the sequence number
 * of any images in the stream. The sequence number is stored as an unsigned
 * integer and will wrap when overflowed.
 *
 * \return The request sequence number
 */
uint32_t Request::sequence() const
{
	return _d()->sequence_;
}

/**
 * \fn Request::cookie()
 * \brief Retrieve the cookie set when the request was created
 * \return The request cookie
 */

/**
 * \fn Request::status()
 * \brief Retrieve the request completion status
 *
 * The request status indicates whether the request has completed successfully
 * or with an error. When requests are created and before they complete the
 * request status is set to RequestPending, and is updated at completion time
 * to RequestComplete. If a request is cancelled at capture stop before it has
 * completed, its status is set to RequestCancelled.
 *
 * \return The request completion status
 */

/**
 * \brief Check if a request has buffers yet to be completed
 *
 * \return True if the request has buffers pending for completion, false
 * otherwise
 */
bool Request::hasPendingBuffers() const
{
	return !_d()->pending_.empty();
}

/**
 * \brief Generate a string representation of the Request internals
 *
 * This function facilitates debugging of Request state while it is used
 * internally within libcamera.
 *
 * \return A string representing the current state of the request
 */
std::string Request::toString() const
{
	std::stringstream ss;
	ss << *this;

	return ss.str();
}

/**
 * \brief Insert a text representation of a Request into an output stream
 * \param[in] out The output stream
 * \param[in] r The Request
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const Request &r)
{
	/* Pending, Completed, Cancelled(X). */
	static const char *statuses = "PCX";

	/* Example Output: Request(55:P:1/2:6523524) */
	out << "Request(" << r.sequence() << ":" << statuses[r.status()] << ":"
	    << r._d()->pending_.size() << "/" << r.buffers().size() << ":"
	    << r.cookie() << ")";

	return out;
}

} /* namespace libcamera */
