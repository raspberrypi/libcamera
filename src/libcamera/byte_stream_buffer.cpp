/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Byte stream buffer
 */

#include "libcamera/internal/byte_stream_buffer.h"

#include <stdint.h>
#include <string.h>

#include <libcamera/base/log.h>

/**
 * \file byte_stream_buffer.h
 * \brief Managed memory container for serialized data
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Serialization)

/**
 * \class ByteStreamBuffer
 * \brief Wrap a memory buffer and provide sequential data read and write
 *
 * The ByteStreamBuffer class wraps a memory buffer and exposes sequential read
 * and write operation with integrated boundary checks. Access beyond the end
 * of the buffer are blocked and logged, allowing error checks to take place at
 * the of of access operations instead of at each access. This simplifies
 * serialization and deserialization of data.
 *
 * A byte stream buffer is created with a base memory pointer and a size. If the
 * memory pointer is const, the buffer operates in read-only mode, and write
 * operations are denied. Otherwise the buffer operates in write-only mode, and
 * read operations are denied.
 *
 * Once a buffer is created, data is read or written with read() and write()
 * respectively. Access is strictly sequential, the buffer keeps track of the
 * current access location and advances it automatically. Reading or writing
 * the same location multiple times is thus not possible. Bytes may also be
 * skipped with the skip() function.
 *
 *
 * The ByteStreamBuffer also supports carving out pieces of memory into other
 * ByteStreamBuffer instances. Like a read or write operation, a carveOut()
 * advances the internal access location, but allows the carved out memory to
 * be accessed at a later time.
 *
 * All accesses beyond the end of the buffer (read, write, skip or carve out)
 * are blocked. The first of such accesses causes a message to be logged, and
 * the buffer being marked as having overflown. If the buffer has been carved
 * out from a parent buffer, the parent buffer is also marked as having
 * overflown. Any later access on an overflown buffer is blocked. The buffer
 * overflow status can be checked with the overflow() function.
 */

/**
 * \brief Construct a read ByteStreamBuffer from the memory area \a base
 * of \a size
 * \param[in] base The address of the memory area to wrap
 * \param[in] size The size of the memory area to wrap
 */
ByteStreamBuffer::ByteStreamBuffer(const uint8_t *base, size_t size)
	: parent_(nullptr), base_(base), size_(size), overflow_(false),
	  read_(base), write_(nullptr)
{
}

/**
 * \brief Construct a write ByteStreamBuffer from the memory area \a base
 * of \a size
 * \param[in] base The address of the memory area to wrap
 * \param[in] size The size of the memory area to wrap
 */
ByteStreamBuffer::ByteStreamBuffer(uint8_t *base, size_t size)
	: parent_(nullptr), base_(base), size_(size), overflow_(false),
	  read_(nullptr), write_(base)
{
}

/**
 * \brief Construct a ByteStreamBuffer from the contents of \a other using move
 * semantics
 * \param[in] other The other buffer
 *
 * After the move construction the \a other buffer is invalidated. Any attempt
 * to access its contents will be considered as an overflow.
 */
ByteStreamBuffer::ByteStreamBuffer(ByteStreamBuffer &&other)
{
	*this = std::move(other);
}

/**
 * \brief Replace the contents of the buffer with those of \a other using move
 * semantics
 * \param[in] other The other buffer
 *
 * After the assignment the \a other buffer is invalidated. Any attempt to
 * access its contents will be considered as an overflow.
 */
ByteStreamBuffer &ByteStreamBuffer::operator=(ByteStreamBuffer &&other)
{
	parent_ = other.parent_;
	base_ = other.base_;
	size_ = other.size_;
	overflow_ = other.overflow_;
	read_ = other.read_;
	write_ = other.write_;

	other.parent_ = nullptr;
	other.base_ = nullptr;
	other.size_ = 0;
	other.overflow_ = false;
	other.read_ = nullptr;
	other.write_ = nullptr;

	return *this;
}

/**
 * \fn ByteStreamBuffer::base()
 * \brief Retrieve a pointer to the start location of the managed memory buffer
 * \return A pointer to the managed memory buffer
 */

/**
 * \fn ByteStreamBuffer::offset()
 * \brief Retrieve the offset of the current access location from the base
 * \return The offset in bytes
 */

/**
 * \fn ByteStreamBuffer::size()
 * \brief Retrieve the size of the managed memory buffer
 * \return The size of managed memory buffer
 */

/**
 * \fn ByteStreamBuffer::overflow()
 * \brief Check if the buffer has overflown
 * \return True if the buffer has overflow, false otherwise
 */

void ByteStreamBuffer::setOverflow()
{
	if (parent_)
		parent_->setOverflow();

	overflow_ = true;
}

/**
 * \brief Carve out an area of \a size bytes into a new ByteStreamBuffer
 * \param[in] size The size of the newly created memory buffer
 *
 * This function carves out an area of \a size bytes from the buffer into a new
 * ByteStreamBuffer, and returns the new buffer. It operates identically to a
 * read or write access from the point of view of the current buffer, but allows
 * the new buffer to be read or written at a later time after other read or
 * write accesses on the current buffer.
 *
 * \return A newly created ByteStreamBuffer of \a size
 */
ByteStreamBuffer ByteStreamBuffer::carveOut(size_t size)
{
	if (!size_ || overflow_)
		return ByteStreamBuffer(static_cast<const uint8_t *>(nullptr), 0);

	const uint8_t *curr = read_ ? read_ : write_;
	if (curr + size > base_ + size_) {
		LOG(Serialization, Error)
			<< "Unable to reserve " << size << " bytes";
		setOverflow();

		return ByteStreamBuffer(static_cast<const uint8_t *>(nullptr), 0);
	}

	if (read_) {
		ByteStreamBuffer b(read_, size);
		b.parent_ = this;
		read_ += size;
		return b;
	} else {
		ByteStreamBuffer b(write_, size);
		b.parent_ = this;
		write_ += size;
		return b;
	}
}

/**
 * \brief Skip \a size bytes from the buffer
 * \param[in] size The number of bytes to skip
 *
 * This function skips the next \a size bytes from the buffer.
 *
 * \return 0 on success, a negative error code otherwise
 * \retval -ENOSPC no more space is available in the managed memory buffer
 */
int ByteStreamBuffer::skip(size_t size)
{
	if (overflow_)
		return -ENOSPC;

	const uint8_t *curr = read_ ? read_ : write_;
	if (curr + size > base_ + size_) {
		LOG(Serialization, Error)
			<< "Unable to skip " << size << " bytes";
		setOverflow();

		return -ENOSPC;
	}

	if (read_) {
		read_ += size;
	} else {
		memset(write_, 0, size);
		write_ += size;
	}

	return 0;
}

/**
 * \fn template<typename T> int ByteStreamBuffer::read(T *t)
 * \brief Read data from the managed memory buffer into \a t
 * \param[out] t Pointer to the memory containing the read data
 * \return 0 on success, a negative error code otherwise
 * \retval -EACCES attempting to read from a write buffer
 * \retval -ENOSPC no more space is available in the managed memory buffer
 */

/**
 * \fn template<typename T> int ByteStreamBuffer::read(const Span<T> &data)
 * \brief Read data from the managed memory buffer into Span \a data
 * \param[out] data Span representing the destination memory
 * \return 0 on success, a negative error code otherwise
 * \retval -EACCES attempting to read from a write buffer
 * \retval -ENOSPC no more space is available in the managed memory buffer
 */

/**
 * \fn template<typename T> const T *ByteStreamBuffer::read(size_t count)
 * \brief Read data from the managed memory buffer without performing a copy
 * \param[in] count Number of data items to read
 *
 * This function reads \a count elements of type \a T from the buffer. Unlike
 * the other read variants, it doesn't copy the data but returns a pointer to
 * the first element. If data can't be read for any reason (usually due to
 * reading more data than available), the function returns nullptr.
 *
 * \return A pointer to the data on success, or nullptr otherwise
 */

/**
 * \fn template<typename T> int ByteStreamBuffer::write(const T *t)
 * \brief Write \a t to the managed memory buffer
 * \param[in] t The data to write to memory
 * \return 0 on success, a negative error code otherwise
 * \retval -EACCES attempting to write to a read buffer
 * \retval -ENOSPC no more space is available in the managed memory buffer
 */

/**
 * \fn template<typename T> int ByteStreamBuffer::write(const Span<T> &data)
 * \brief Write \a data to the managed memory buffer
 * \param[in] data The data to write to memory
 * \return 0 on success, a negative error code otherwise
 * \retval -EACCES attempting to write to a read buffer
 * \retval -ENOSPC no more space is available in the managed memory buffer
 */

const uint8_t *ByteStreamBuffer::read(size_t size, size_t count)
{
	if (!read_)
		return nullptr;

	if (overflow_)
		return nullptr;

	size_t bytes;
	if (__builtin_mul_overflow(size, count, &bytes)) {
		setOverflow();
		return nullptr;
	}

	if (read_ + bytes > base_ + size_) {
		LOG(Serialization, Error)
			<< "Unable to read " << bytes << " bytes: out of bounds";
		setOverflow();
		return nullptr;
	}

	const uint8_t *data = read_;
	read_ += bytes;
	return data;
}

int ByteStreamBuffer::read(uint8_t *data, size_t size)
{
	if (!read_)
		return -EACCES;

	if (overflow_)
		return -ENOSPC;

	if (read_ + size > base_ + size_) {
		LOG(Serialization, Error)
			<< "Unable to read " << size << " bytes: out of bounds";
		setOverflow();
		return -ENOSPC;
	}

	memcpy(data, read_, size);
	read_ += size;

	return 0;
}

int ByteStreamBuffer::write(const uint8_t *data, size_t size)
{
	if (!write_)
		return -EACCES;

	if (overflow_)
		return -ENOSPC;

	if (write_ + size > base_ + size_) {
		LOG(Serialization, Error)
			<< "Unable to write " << size << " bytes: no space left";
		setOverflow();
		return -ENOSPC;
	}

	memcpy(write_, data, size);
	write_ += size;

	return 0;
}

} /* namespace libcamera */
