/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Byte stream buffer
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <type_traits>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

namespace libcamera {

class ByteStreamBuffer
{
public:
	ByteStreamBuffer(const uint8_t *base, size_t size);
	ByteStreamBuffer(uint8_t *base, size_t size);
	ByteStreamBuffer(ByteStreamBuffer &&other);
	ByteStreamBuffer &operator=(ByteStreamBuffer &&other);

	const uint8_t *base() const { return base_; }
	uint32_t offset() const { return (write_ ? write_ : read_) - base_; }
	size_t size() const { return size_; }
	bool overflow() const { return overflow_; }

	ByteStreamBuffer carveOut(size_t size);
	int skip(size_t size);

	template<typename T>
	int read(T *t)
	{
		return read(reinterpret_cast<uint8_t *>(t), sizeof(*t));
	}

	template<typename T>
	int read(const Span<T> &data)
	{
		return read(reinterpret_cast<uint8_t *>(data.data()),
			    data.size_bytes());
	}

	template<typename T>
	const std::remove_reference_t<T> *read(size_t count = 1)
	{
		using return_type = const std::remove_reference_t<T> *;
		return reinterpret_cast<return_type>(read(sizeof(T), count));
	}

	template<typename T>
	int write(const T *t)
	{
		return write(reinterpret_cast<const uint8_t *>(t), sizeof(*t));
	}

	template<typename T>
	int write(const Span<T> &data)
	{
		return write(reinterpret_cast<const uint8_t *>(data.data()),
			     data.size_bytes());
	}

private:
	LIBCAMERA_DISABLE_COPY(ByteStreamBuffer)

	void setOverflow();

	int read(uint8_t *data, size_t size);
	const uint8_t *read(size_t size, size_t count);
	int write(const uint8_t *data, size_t size);

	ByteStreamBuffer *parent_;

	const uint8_t *base_;
	size_t size_;
	bool overflow_;

	const uint8_t *read_;
	uint8_t *write_;
};

} /* namespace libcamera */
