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

#include "log.h"

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
 * \class BufferMemory
 * \brief A memory buffer to store an image
 *
 * The BufferMemory class represents the memory buffers used to store full frame
 * images, which may contain multiple separate memory Plane objects if the
 * image format is multi-planar.
 */

/**
 * \fn BufferMemory::planes() const
 * \brief Retrieve the planes within the buffer
 * \return A const reference to a vector holding all planes within the buffer
 */

/**
 * \fn BufferMemory::planes()
 * \brief Retrieve the planes within the buffer
 * \return A reference to a vector holding all planes within the buffer
 */

/**
 * \class BufferPool
 * \brief A pool of buffers
 *
 * The BufferPool class groups together a collection of Buffers to store frames.
 * The buffers must be exported by a device before they can be imported into
 * another device for further use.
 */

BufferPool::~BufferPool()
{
	destroyBuffers();
}

/**
 * \brief Create buffers in the Pool
 * \param[in] count The number of buffers to create
 */
void BufferPool::createBuffers(unsigned int count)
{
	buffers_.resize(count);
}

/**
 * \brief Release all buffers from pool
 *
 * If no buffers have been created or if buffers have already been released no
 * operation is performed.
 */
void BufferPool::destroyBuffers()
{
	buffers_.resize(0);
}

/**
 * \fn BufferPool::count()
 * \brief Retrieve the number of buffers contained within the pool
 * \return The number of buffers contained in the pool
 */

/**
 * \fn BufferPool::buffers()
 * \brief Retrieve all the buffers in the pool
 * \return A vector containing all the buffers in the pool.
 */

/**
 * \class Buffer
 * \brief A buffer handle and dynamic metadata
 *
 * The Buffer class references a buffer memory and associates dynamic metadata
 * related to the frame contained in the buffer. It allows referencing buffer
 * memory through a single interface regardless of whether the memory is
 * allocated internally in libcamera or provided externally through dmabuf.
 *
 * Buffer instances are allocated dynamically for a stream through
 * Stream::createBuffer(), added to a request with Request::addBuffer() and
 * deleted automatically after the request complete handler returns.
 */

/**
 * \brief Construct a buffer not associated with any stream
 *
 * This method constructs an orphaned buffer not associated with any stream. It
 * is not meant to be called by applications, they should instead create buffers
 * for a stream with Stream::createBuffer().
 */
Buffer::Buffer(unsigned int index, const Buffer *metadata)
	: index_(index), dmabuf_({ -1, -1, -1 }), request_(nullptr),
	  stream_(nullptr)
{
	if (metadata)
		metadata_ = metadata->metadata();
	else
		metadata_ = {};

	metadata_.status = FrameMetadata::FrameSuccess;
}

/**
 * \fn Buffer::index()
 * \brief Retrieve the Buffer index
 * \return The buffer index
 */

/**
 * \fn Buffer::dmabufs()
 * \brief Retrieve the dmabuf file descriptors for all buffer planes
 *
 * The dmabufs array contains one dmabuf file descriptor per plane. Unused
 * entries are set to -1.
 *
 * \return The dmabuf file descriptors
 */

/**
 * \fn Buffer::mem()
 * \brief Retrieve the BufferMemory this buffer is associated with
 *
 * The association between the buffer and a BufferMemory instance is valid from
 * the time the request containing this buffer is queued to a camera to the end
 * of that request's completion handler.
 *
 * \return The BufferMemory this buffer is associated with
 */

/**
 * \fn Buffer::metadata()
 * \brief Retrieve the buffer metadata
 *
 * The buffer metadata is updated when the buffer contents are modified, for
 * example when a frame has been captured to the buffer by the hardware.
 *
 * \return Metadata for the buffer
 */

/**
 * \fn Buffer::request()
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
 * either completed or not associated with a request
 */

/**
 * \fn Buffer::stream()
 * \brief Retrieve the stream this buffer is associated with
 *
 * A Buffer is associated to the stream that created it with
 * Stream::createBuffer() and the association is valid until the buffer is
 * destroyed. Buffer instances that are created directly are not associated
 * with any stream.
 *
 * \return The Stream the Buffer is associated with, or nullptr if the buffer
 * is not associated with a stream
 */

/**
 * \brief Mark a buffer as cancel by setting its status to BufferCancelled
 */
void Buffer::cancel()
{
	metadata_.status = FrameMetadata::FrameCancelled;
	metadata_.sequence = 0;
	metadata_.timestamp = 0;
	metadata_.planes = {};
}

/**
 * \var Buffer::request_
 * \brief The request this buffer belongs to
 *
 * This member is intended to be set by Request::addBuffer() and
 * Request::completeBuffer().
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

} /* namespace libcamera */
