/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * byte_stream_buffer.h - Byte stream buffer
 */
#ifndef __LIBCAMERA_BYTE_STREAM_BUFFER_H__
#define __LIBCAMERA_BYTE_STREAM_BUFFER_H__

#include <stddef.h>
#include <stdint.h>

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
	int write(const T *t)
	{
		return write(reinterpret_cast<const uint8_t *>(t), sizeof(*t));
	}

private:
	ByteStreamBuffer(const ByteStreamBuffer &other) = delete;
	ByteStreamBuffer &operator=(const ByteStreamBuffer &other) = delete;

	void setOverflow();

	int read(uint8_t *data, size_t size);
	int write(const uint8_t *data, size_t size);

	ByteStreamBuffer *parent_;

	const uint8_t *base_;
	size_t size_;
	bool overflow_;

	const uint8_t *read_;
	uint8_t *write_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_BYTE_STREAM_BUFFER_H__ */
