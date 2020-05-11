/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * buffer.cpp - Buffer handling
 */

#include <libcamera/buffer.h>

#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "libcamera/internal/log.h"

/**
 * \file buffer.h
 * \brief Buffer handling
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Buffer)

/**
 * \struct FrameMetadata
 * \brief Metadata related to a captured frame
 *
 * The FrameMetadata structure stores all metadata related to a captured frame,
 * as stored in a FrameBuffer, such as capture status, timestamp and bytesused.
 */

/**
 * \enum FrameMetadata::Status
 * \brief Define the frame completion status
 * \var FrameMetadata::FrameSuccess
 * The frame has been captured with success and contains valid data. All fields
 * of the FrameMetadata structure are valid.
 * \var FrameMetadata::FrameError
 * An error occurred during capture of the frame. The frame data may be partly
 * or fully invalid. The sequence and timestamp fields of the FrameMetadata
 * structure is valid, the other fields may be invalid.
 * \var FrameMetadata::FrameCancelled
 * Capture stopped before the frame completed. The frame data is not valid. All
 * fields of the FrameMetadata structure but the status field are invalid.
 */

/**
 * \struct FrameMetadata::Plane
 * \brief Per-plane frame metadata
 *
 * Frames are stored in memory in one or multiple planes. The
 * FrameMetadata::Plane structure stores per-plane metadata.
 */

/**
 * \var FrameMetadata::Plane::bytesused
 * \brief Number of bytes occupied by the data in the plane, including line
 * padding
 *
 * This value may vary per frame for compressed formats. For uncompressed
 * formats it will be constant for all frames, but may be smaller than the
 * FrameBuffer size.
 */

/**
 * \var FrameMetadata::status
 * \brief Status of the frame
 *
 * The validity of other fields of the FrameMetadata structure depends on the
 * status value.
 */

/**
 * \var FrameMetadata::sequence
 * \brief Frame sequence number
 *
 * The sequence number is a monotonically increasing number assigned to the
 * frames captured by the stream. The value is increased by one for each frame.
 * Gaps in the sequence numbers indicate dropped frames.
 */

/**
 * \var FrameMetadata::timestamp
 * \brief Time when the frame was captured
 *
 * The timestamp is expressed as a number of nanoseconds relative to the system
 * clock since an unspecified time point.
 *
 * \todo Be more precise on what timestamps refer to.
 */

/**
 * \var FrameMetadata::planes
 * \brief Array of per-plane metadata
 */

/**
 * \class FrameBuffer
 * \brief Frame buffer data and its associated dynamic metadata
 *
 * The FrameBuffer class is the primary interface for applications, IPAs and
 * pipeline handlers to interact with frame memory. It contains all the static
 * and dynamic information to manage the whole life cycle of a frame capture,
 * from buffer creation to consumption.
 *
 * The static information describes the memory planes that make a frame. The
 * planes are specified when creating the FrameBuffer and are expressed as a set
 * of dmabuf file descriptors and length.
 *
 * The dynamic information is grouped in a FrameMetadata instance. It is updated
 * during the processing of a queued capture request, and is valid from the
 * completion of the buffer as signaled by Camera::bufferComplete() until the
 * FrameBuffer is either reused in a new request or deleted.
 *
 * The creator of a FrameBuffer (application, IPA or pipeline handler) may
 * associate to it an integer cookie for any private purpose. The cookie may be
 * set when creating the FrameBuffer, and updated at any time with setCookie().
 * The cookie is transparent to the libcamera core and shall only be set by the
 * creator of the FrameBuffer. This mechanism supplements the Request cookie.
 */

/**
 * \struct FrameBuffer::Plane
 * \brief A memory region to store a single plane of a frame
 *
 * Planar pixel formats use multiple memory regions to store the different
 * colour components of a frame. The Plane structure describes such a memory
 * region by a dmabuf file descriptor and a length. A FrameBuffer then
 * contains one or multiple planes, depending on the pixel format of the
 * frames it is meant to store.
 *
 * To support DMA access, planes are associated with dmabuf objects represented
 * by FileDescriptor handles. The Plane class doesn't handle mapping of the
 * memory to the CPU, but applications and IPAs may use the dmabuf file
 * descriptors to map the plane memory with mmap() and access its contents.
 *
 * \todo Once we have a Kernel API which can express offsets within a plane
 * this structure shall be extended to contain this information. See commit
 * 83148ce8be55e for initial documentation of this feature.
 */

/**
 * \var FrameBuffer::Plane::fd
 * \brief The dmabuf file descriptor
 */

/**
 * \var FrameBuffer::Plane::length
 * \brief The plane length in bytes
 */

/**
 * \brief Construct a FrameBuffer with an array of planes
 * \param[in] planes The frame memory planes
 * \param[in] cookie Cookie
 */
FrameBuffer::FrameBuffer(const std::vector<Plane> &planes, unsigned int cookie)
	: planes_(planes), request_(nullptr), cookie_(cookie)
{
}

/**
 * \fn FrameBuffer::planes()
 * \brief Retrieve the static plane descriptors
 * \return Array of plane descriptors
 */

/**
 * \fn FrameBuffer::request()
 * \brief Retrieve the request this buffer belongs to
 *
 * The intended callers of this method are buffer completion handlers that
 * need to associate a buffer to the request it belongs to.
 *
 * A Buffer is associated to a request by Request::addBuffer() and the
 * association is valid until the buffer completes. The returned request
 * pointer is valid only during that interval.
 *
 * \return The Request the Buffer belongs to, or nullptr if the buffer is
 * not associated with a request
 */

/**
 * \fn FrameBuffer::setRequest()
 * \brief Set the request this buffer belongs to
 * \param[in] request Request to set
 *
 * The intended callers of this method are pipeline handlers and only for
 * buffers that are internal to the pipeline.
 *
 * \todo Shall be hidden from applications with a d-pointer design.
 */

/**
 * \fn FrameBuffer::metadata()
 * \brief Retrieve the dynamic metadata
 * \return Dynamic metadata for the frame contained in the buffer
 */

/**
 * \fn FrameBuffer::cookie()
 * \brief Retrieve the cookie
 *
 * The cookie belongs to the creator of the FrameBuffer, which controls its
 * lifetime and value.
 *
 * \sa setCookie()
 *
 * \return The cookie
 */

/**
 * \fn FrameBuffer::setCookie()
 * \brief Set the cookie
 * \param[in] cookie Cookie to set
 *
 * The cookie belongs to the creator of the FrameBuffer. Its value may be
 * modified at any time with this method. Applications and IPAs shall not modify
 * the cookie value of buffers they haven't created themselves. The libcamera
 * core never modifies the buffer cookie.
 */

/**
 * \brief Copy the contents from another buffer
 * \param[in] src Buffer to copy
 *
 * Copy the buffer contents and metadata from \a src to this buffer. The
 * destination FrameBuffer shall have the same number of planes as the source
 * buffer, and each destination plane shall be larger than or equal to the
 * corresponding source plane.
 *
 * The complete metadata of the source buffer is copied to the destination
 * buffer. If an error occurs during the copy, the destination buffer's metadata
 * status is set to FrameMetadata::FrameError, and other metadata fields are not
 * modified.
 *
 * The operation is performed using memcpy() so is very slow, users needs to
 * consider this before copying buffers.
 *
 * \return 0 on success or a negative error code otherwise
 */
int FrameBuffer::copyFrom(const FrameBuffer *src)
{
	if (planes_.size() != src->planes_.size()) {
		LOG(Buffer, Error) << "Different number of planes";
		metadata_.status = FrameMetadata::FrameError;
		return -EINVAL;
	}

	for (unsigned int i = 0; i < planes_.size(); i++) {
		if (planes_[i].length < src->planes_[i].length) {
			LOG(Buffer, Error) << "Plane " << i << " is too small";
			metadata_.status = FrameMetadata::FrameError;
			return -EINVAL;
		}
	}

	for (unsigned int i = 0; i < planes_.size(); i++) {
		void *dstmem = mmap(nullptr, planes_[i].length, PROT_WRITE,
				    MAP_SHARED, planes_[i].fd.fd(), 0);

		if (dstmem == MAP_FAILED) {
			LOG(Buffer, Error)
				<< "Failed to map destination plane " << i;
			metadata_.status = FrameMetadata::FrameError;
			return -EINVAL;
		}

		void *srcmem = mmap(nullptr, src->planes_[i].length, PROT_READ,
				    MAP_SHARED, src->planes_[i].fd.fd(), 0);

		if (srcmem == MAP_FAILED) {
			munmap(dstmem, planes_[i].length);
			LOG(Buffer, Error)
				<< "Failed to map source plane " << i;
			metadata_.status = FrameMetadata::FrameError;
			return -EINVAL;
		}

		memcpy(dstmem, srcmem, src->planes_[i].length);

		munmap(srcmem, src->planes_[i].length);
		munmap(dstmem, planes_[i].length);
	}

	metadata_ = src->metadata_;

	return 0;
}

} /* namespace libcamera */
