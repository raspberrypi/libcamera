/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * request.cpp - Capture request handling
 */

#include <libcamera/request.h>

#include <map>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_controls.h"
#include "libcamera/internal/log.h"

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
 * cookie() method at any time. It is typically used by applications to map the
 * request to an external resource in the request completion handler, and is
 * completely opaque to libcamera.
 *
 */
Request::Request(Camera *camera, uint64_t cookie)
	: camera_(camera), cookie_(cookie), status_(RequestPending),
	  cancelled_(false)
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
}

Request::~Request()
{
	delete metadata_;
	delete controls_;
	delete validator_;
}

/**
 * \fn Request::controls()
 * \brief Retrieve the request's ControlList
 *
 * Requests store a list of controls to be applied to all frames captured for
 * the request. They are created with an empty list of controls that can be
 * accessed through this method and updated with ControlList::operator[]() or
 * ControlList::update().
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
 * been added to the request for the same stream, this method returns -EEXIST.
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

	buffer->request_ = this;
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
	ASSERT(!hasPendingBuffers());
	status_ = cancelled_ ? RequestCancelled : RequestComplete;

	LOG(Request, Debug)
		<< "Request has completed - cookie: " << cookie_
		<< (cancelled_ ? " [Cancelled]" : "");
}

/**
 * \brief Complete a buffer for the request
 * \param[in] buffer The buffer that has completed
 *
 * A request tracks the status of all buffers it contains through a set of
 * pending buffers. This function removes the \a buffer from the set to mark it
 * as complete. All buffers associate with the request shall be marked as
 * complete by calling this function once and once only before reporting the
 * request as complete with the complete() method.
 *
 * \return True if all buffers contained in the request have completed, false
 * otherwise
 */
bool Request::completeBuffer(FrameBuffer *buffer)
{
	int ret = pending_.erase(buffer);
	ASSERT(ret == 1);

	buffer->request_ = nullptr;

	if (buffer->metadata().status == FrameMetadata::FrameCancelled)
		cancelled_ = true;

	return !hasPendingBuffers();
}

} /* namespace libcamera */
