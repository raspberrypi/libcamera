/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * buffer.cpp - Buffer handling
 */

#include <libcamera/buffer.h>
#include "libcamera/internal/buffer.h"

#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "libcamera/internal/log.h"

/**
 * \file libcamera/buffer.h
 * \brief Buffer handling
 *
 * \file libcamera/internal/buffer.h
 * \brief Internal buffer handling support
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
 * A FrameBuffer is associated to a request by Request::addBuffer() and the
 * association is valid until the buffer completes. The returned request
 * pointer is valid only during that interval.
 *
 * \return The Request the FrameBuffer belongs to, or nullptr if the buffer is
 * not associated with a request
 */

/**
 * \fn FrameBuffer::setRequest()
 * \brief Set the request this buffer belongs to
 * \param[in] request Request to set
 *
 * For buffers added to requests by applications, this method is called by
 * Request::addBuffer() or Request::reuse(). For buffers internal to pipeline
 * handlers, it is called by the pipeline handlers themselves.
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
 * \fn FrameBuffer::cancel()
 * \brief Marks the buffer as cancelled
 *
 * If a buffer is not used by a request, it shall be marked as cancelled to
 * indicate that the metadata is invalid.
 */

/**
 * \class MappedBuffer
 * \brief Provide an interface to support managing memory mapped buffers
 *
 * The MappedBuffer interface provides access to a set of MappedPlanes which
 * are available for access by the CPU.
 *
 * This class is not meant to be constructed directly, but instead derived
 * classes should be used to implement the correct mapping of a source buffer.
 *
 * This allows treating CPU accessible memory through a generic interface
 * regardless of whether it originates from a libcamera FrameBuffer or other
 * source.
 */

/**
 * \typedef MappedBuffer::Plane
 * \brief A mapped region of memory accessible to the CPU
 *
 * The MappedBuffer::Plane uses the Span interface to describe the mapped memory
 * region.
 */

/**
 * \brief Construct an empty MappedBuffer
 */
MappedBuffer::MappedBuffer()
	: error_(0)
{
}

/**
 * \brief Move constructor, construct the MappedBuffer with the contents of \a
 * other using move semantics
 * \param[in] other The other MappedBuffer
 *
 * Moving a MappedBuffer moves the mappings contained in the \a other to the new
 * MappedBuffer and invalidates the \a other.
 *
 * No mappings are unmapped or destroyed in this process.
 */
MappedBuffer::MappedBuffer(MappedBuffer &&other)
{
	*this = std::move(other);
}

/**
 * \brief Move assignment operator, replace the mappings with those of \a other
* \param[in] other The other MappedBuffer
 *
 * Moving a MappedBuffer moves the mappings contained in the \a other to the new
 * MappedBuffer and invalidates the \a other.
 *
 * No mappings are unmapped or destroyed in this process.
 */
MappedBuffer &MappedBuffer::operator=(MappedBuffer &&other)
{
	error_ = other.error_;
	maps_ = std::move(other.maps_);
	other.error_ = -ENOENT;

	return *this;
}

MappedBuffer::~MappedBuffer()
{
	for (Plane &map : maps_)
		munmap(map.data(), map.size());
}

/**
 * \fn MappedBuffer::isValid()
 * \brief Check if the MappedBuffer instance is valid
 * \return True if the MappedBuffer has valid mappings, false otherwise
 */

/**
 * \fn MappedBuffer::error()
 * \brief Retrieve the map error status
 *
 * This function retrieves the error status from the MappedBuffer.
 * The error status is a negative number as defined by errno.h. If
 * no error occurred, this function returns 0.
 *
 * \return The map error code
 */

/**
 * \fn MappedBuffer::maps()
 * \brief Retrieve the mapped planes
 *
 * This function retrieves the successfully mapped planes stored as a vector
 * of Span<uint8_t> to provide access to the mapped memory.
 *
 * \return A vector of the mapped planes
 */

/**
 * \var MappedBuffer::error_
 * \brief Stores the error value if present
 *
 * MappedBuffer derived classes shall set this to a negative value as defined
 * by errno.h if an error occured during the mapping process.
 */

/**
 * \var MappedBuffer::maps_
 * \brief Stores the internal mapped planes
 *
 * MappedBuffer derived classes shall store the mappings they create in this
 * vector which is parsed during destruct to unmap any memory mappings which
 * completed successfully.
 */

/**
 * \class MappedFrameBuffer
 * \brief Map a FrameBuffer using the MappedBuffer interface
 */

/**
 * \brief Map all planes of a FrameBuffer
 * \param[in] buffer FrameBuffer to be mapped
 * \param[in] flags Protection flags to apply to map
 *
 * Construct an object to map a frame buffer for CPU access.
 * The flags are passed directly to mmap and should be either PROT_READ,
 * PROT_WRITE, or a bitwise-or combination of both.
 */
MappedFrameBuffer::MappedFrameBuffer(const FrameBuffer *buffer, int flags)
{
	maps_.reserve(buffer->planes().size());

	for (const FrameBuffer::Plane &plane : buffer->planes()) {
		void *address = mmap(nullptr, plane.length, flags,
				     MAP_SHARED, plane.fd.fd(), 0);
		if (address == MAP_FAILED) {
			error_ = -errno;
			LOG(Buffer, Error) << "Failed to mmap plane";
			break;
		}

		maps_.emplace_back(static_cast<uint8_t *>(address), plane.length);
	}
}

} /* namespace libcamera */
