/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * buffer_writer.h - Buffer writer
 */
#ifndef __LIBCAMERA_BUFFER_WRITER_H__
#define __LIBCAMERA_BUFFER_WRITER_H__

#include <string>

#include <libcamera/buffer.h>

class BufferWriter
{
public:
	BufferWriter(const std::string &pattern = "frame-#.bin");

	int write(libcamera::Buffer *buffer, const std::string &streamName);

private:
	std::string pattern_;
};

#endif /* __LIBCAMERA_BUFFER_WRITER_H__ */
