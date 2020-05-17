/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * buffer_writer.h - Buffer writer
 */
#ifndef __CAM_BUFFER_WRITER_H__
#define __CAM_BUFFER_WRITER_H__

#include <map>
#include <string>

#include <libcamera/buffer.h>

class BufferWriter
{
public:
	BufferWriter(const std::string &pattern = "frame-#.bin");
	~BufferWriter();

	void mapBuffer(libcamera::FrameBuffer *buffer);

	int write(libcamera::FrameBuffer *buffer,
		  const std::string &streamName);

private:
	std::string pattern_;
	std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;
};

#endif /* __CAM_BUFFER_WRITER_H__ */
