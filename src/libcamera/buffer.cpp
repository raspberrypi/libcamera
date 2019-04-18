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
 * \class Plane
 * \brief A memory region to store a single plane of a frame
 *
 * Planar pixel formats use multiple memory regions to store planes
 * corresponding to the different colour components of a frame. The Plane class
 * tracks the specific details of a memory region used to store a single plane
 * for a given frame and provides the means to access the memory, both for the
 * application and for DMA. A Buffer then contains one or multiple planes
 * depending on its pixel format.
 *
 * To support DMA access, planes are associated with dmabuf objects represented
 * by file handles. Each plane carries a dmabuf file handle and an offset within
 * the buffer. Those file handles may refer to the same dmabuf object, depending
 * on whether the devices accessing the memory regions composing the image
 * support non-contiguous DMA to planes ore require DMA-contiguous memory.
 *
 * To support CPU access, planes carry the CPU address of their backing memory.
 * Similarly to the dmabuf file handles, the CPU addresses for planes composing
 * an image may or may not be contiguous.
 */

Plane::Plane()
	: fd_(-1), length_(0), mem_(0)
{
}

Plane::~Plane()
{
	munmap();

	if (fd_ != -1)
		close(fd_);
}

/**
 * \fn Plane::dmabuf()
 * \brief Get the dmabuf file handle backing the buffer
 */

/**
 * \brief Set the dmabuf file handle backing the buffer
 * \param[in] fd The dmabuf file handle
 * \param[in] length The size of the memory region
 *
 * The \a fd dmabuf file handle is duplicated and stored. The caller may close
 * the original file handle.
 *
 * \return 0 on success or a negative error code otherwise
 */
int Plane::setDmabuf(int fd, unsigned int length)
{
	if (fd < 0) {
		LOG(Buffer, Error) << "Invalid dmabuf fd provided";
		return -EINVAL;
	}

	if (fd_ != -1) {
		close(fd_);
		fd_ = -1;
	}

	fd_ = dup(fd);
	if (fd_ == -1) {
		int ret = -errno;
		LOG(Buffer, Error)
			<< "Failed to duplicate dmabuf: " << strerror(-ret);
		return ret;
	}

	length_ = length;

	return 0;
}

/**
 * \brief Map the plane memory data to a CPU accessible address
 *
 * The file descriptor to map the memory from must be set by a call to
 * setDmaBuf() before calling this function.
 *
 * \sa setDmaBuf()
 *
 * \return 0 on success or a negative error code otherwise
 */
int Plane::mmap()
{
	void *map;

	if (mem_)
		return 0;

	map = ::mmap(NULL, length_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
	if (map == MAP_FAILED) {
		int ret = -errno;
		LOG(Buffer, Error)
			<< "Failed to mmap plane: " << strerror(-ret);
		return ret;
	}

	mem_ = map;

	return 0;
}

/**
 * \brief Unmap any existing CPU accessible mapping
 *
 * Unmap the memory mapped by an earlier call to mmap().
 *
 * \return 0 on success or a negative error code otherwise
 */
int Plane::munmap()
{
	int ret = 0;

	if (mem_)
		ret = ::munmap(mem_, length_);

	if (ret) {
		ret = -errno;
		LOG(Buffer, Warning)
			<< "Failed to unmap plane: " << strerror(-ret);
	} else {
		mem_ = 0;
	}

	return ret;
}

/**
 * \fn Plane::mem()
 * \brief Retrieve the CPU accessible memory address of the Plane
 * \return The CPU accessible memory address on success or nullptr otherwise.
 */
void *Plane::mem()
{
	if (!mem_)
		mmap();

	return mem_;
}

/**
 * \fn Plane::length()
 * \brief Retrieve the length of the memory region
 * \return The length of the memory region
 */

/**
 * \class Buffer
 * \brief A memory buffer to store an image
 *
 * The Buffer class represents the memory buffers used to store a
 * full frame image, which may contain multiple separate memory Plane
 * objects if the image format is multi-planar.
 */

/**
 * \enum Buffer::Status
 * Buffer completion status
 * \var Buffer::BufferSuccess
 * The buffer has completed with success and contains valid data. All its other
 * metadata (such as bytesused(), timestamp() or sequence() number) are valid.
 * \var Buffer::BufferError
 * The buffer has completed with an error and doesn't contain valid data. Its
 * other metadata are valid.
 * \var Buffer::BufferCancelled
 * The buffer has been cancelled due to capture stop. Its other metadata are
 * invalid and shall not be used.
 */

Buffer::Buffer()
	: index_(-1), request_(nullptr)
{
}

/**
 * \fn Buffer::index()
 * \brief Retrieve the Buffer index
 * \return The buffer index
 */

/**
 * \fn Buffer::planes()
 * \brief Retrieve the planes within the buffer
 * \return A reference to a vector holding all Planes within the buffer
 */

/**
 * \fn Buffer::bytesused()
 * \brief Retrieve the number of bytes occupied by the data in the buffer
 * \return Number of bytes occupied in the buffer
 */

/**
 * \fn Buffer::timestamp()
 * \brief Retrieve the time when the buffer was processed
 *
 * The timestamp is expressed as a number number of nanoseconds since the epoch.
 *
 * \return Timestamp when the buffer was processed
 */

/**
 * \fn Buffer::sequence()
 * \brief Retrieve the buffer sequence number
 *
 * The sequence number is a monotonically increasing number assigned to the
 * buffer processed by the stream. Gaps in the sequence numbers indicate
 * dropped frames.
 *
 * \return Sequence number of the buffer
 */

/**
 * \fn Buffer::status()
 * \brief Retrieve the buffer status
 *
 * The buffer status reports whether the buffer has completed successfully
 * (BufferSuccess) or if an error occurred (BufferError).
 *
 * \return The buffer status
 */

/**
 * \fn Buffer::request()
 * \brief Retrieve the request this buffer belongs to
 *
 * The intended callers of this method are buffer completion handlers that
 * need to associate a buffer to the request it belongs to.
 *
 * A Buffer is associated to a request by Request::prepare() and the
 * association is valid until the buffer completes. The returned request
 * pointer is valid only during that interval.
 *
 * \return The Request the Buffer belongs to, or nullptr if the buffer is
 * either completed or not associated with a request
 * \sa Buffer::setRequest()
 */

/**
 * \brief Mark a buffer as cancel by setting its status to BufferCancelled
 */
void Buffer::cancel()
{
	bytesused_ = 0;
	timestamp_ = 0;
	sequence_ = 0;
	status_ = BufferCancelled;
}

/**
 * \fn Buffer::setRequest()
 * \brief Set the request this buffer belongs to
 *
 * The intended callers are Request::prepare() and Request::completeBuffer().
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
	unsigned int index = 0;

	buffers_.resize(count);
	for (Buffer &buffer : buffers_)
		buffer.index_ = index++;
}

/**
 * \brief Release all buffers from pool
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

} /* namespace libcamera */
