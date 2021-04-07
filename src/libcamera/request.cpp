/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * request.cpp - Capture request handling
 */

#include <libcamera/request.h>

#include <map>
#include <sstream>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_controls.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/tracepoints.h"

/**
 * \file request.h
 * \brief Describes a frame capture request to be processed by a camera
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Request)

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
	: camera_(camera), sequence_(0), cookie_(cookie),
	  status_(RequestPending), cancelled_(false)
{
	/**
	 * \todo Should the Camera expose a validator instance, to avoid
	 * creating a new instance for each request?
	 */
	validator_ = new CameraControlValidator(camera);
	controls_ = new ControlList(controls::controls, validator_);

	/**
	 * \todo: Add a validator for metadata controls.
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
	delete validator_;
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

	pending_.clear();
	if (flags & ReuseBuffers) {
		for (auto pair : bufferMap_) {
			FrameBuffer *buffer = pair.second;
			buffer->_d()->setRequest(this);
			pending_.insert(buffer);
		}
	} else {
		bufferMap_.clear();
	}

	sequence_ = 0;
	status_ = RequestPending;
	cancelled_ = false;

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
 *
 * A reference to the buffer is stored in the request. The caller is responsible
 * for ensuring that the buffer will remain valid until the request complete
 * callback is called.
 *
 * A request can only contain one buffer per stream. If a buffer has already
 * been added to the request for the same stream, this function returns -EEXIST.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EEXIST The request already contains a buffer for the stream
 * \retval -EINVAL The buffer does not reference a valid Stream
 */
int Request::addBuffer(const Stream *stream, FrameBuffer *buffer)
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
	pending_.insert(buffer);
	bufferMap_[stream] = buffer;

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
 * \fn Request::sequence()
 * \brief Retrieve the sequence number for the request
 *
 * When requests are queued, they are given a sequential number to track the
 * order in which requests are queued to a camera. This number counts all
 * requests given to a camera through its lifetime, and is not reset to zero
 * between camera stop/start sequences.
 *
 * It can be used to support debugging and identifying the flow of requests
 * through a pipeline, but does not guarantee to represent the sequence number
 * of any images in the stream. The sequence number is stored as an unsigned
 * integer and will wrap when overflowed.
 *
 * \return The request sequence number
 */

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
 * \fn Request::hasPendingBuffers()
 * \brief Check if a request has buffers yet to be completed
 *
 * \return True if the request has buffers pending for completion, false
 * otherwise
 */

/**
 * \brief Complete a queued request
 *
 * Mark the request as complete by updating its status to RequestComplete,
 * unless buffers have been cancelled in which case the status is set to
 * RequestCancelled.
 */
void Request::complete()
{
	ASSERT(status_ == RequestPending);
	ASSERT(!hasPendingBuffers());

	status_ = cancelled_ ? RequestCancelled : RequestComplete;

	LOG(Request, Debug) << toString();

	LIBCAMERA_TRACEPOINT(request_complete, this);
}

/**
 * \brief Cancel a queued request
 *
 * Mark the request and its associated buffers as cancelled and complete it.
 *
 * Set each pending buffer in error state and emit the buffer completion signal
 * before completing the Request.
 */
void Request::cancel()
{
	LIBCAMERA_TRACEPOINT(request_cancel, this);

	ASSERT(status_ == RequestPending);

	for (FrameBuffer *buffer : pending_) {
		buffer->cancel();
		camera_->bufferCompleted.emit(this, buffer);
	}

	pending_.clear();
	cancelled_ = true;
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
bool Request::completeBuffer(FrameBuffer *buffer)
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

	/* Pending, Completed, Cancelled(X). */
	static const char *statuses = "PCX";

	/* Example Output: Request(55:P:1/2:6523524) */
	ss << "Request(" << sequence_ << ":" << statuses[status_] << ":"
	   << pending_.size() << "/" << bufferMap_.size() << ":"
	   << cookie_ << ")";

	return ss.str();
}

} /* namespace libcamera */
