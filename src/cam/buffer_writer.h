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

#include <libcamera/stream.h>

#include "frame_sink.h"

class BufferWriter : public FrameSink
{
public:
	BufferWriter(const std::string &pattern = "");
	~BufferWriter();

	int configure(const libcamera::CameraConfiguration &config) override;

	void mapBuffer(libcamera::FrameBuffer *buffer) override;

	bool processRequest(libcamera::Request *request) override;

private:
	void writeBuffer(const libcamera::Stream *stream,
			 libcamera::FrameBuffer *buffer);

	std::map<const libcamera::Stream *, std::string> streamNames_;
	std::string pattern_;
	std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;
};

#endif /* __CAM_BUFFER_WRITER_H__ */
