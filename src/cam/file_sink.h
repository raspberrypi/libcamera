/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * file_sink.h - File Sink
 */
#ifndef __CAM_FILE_SINK_H__
#define __CAM_FILE_SINK_H__

#include <map>
#include <string>

#include <libcamera/stream.h>

#include "frame_sink.h"

class FileSink : public FrameSink
{
public:
	FileSink(const std::string &pattern = "");
	~FileSink();

	int configure(const libcamera::CameraConfiguration &config) override;

	void mapBuffer(libcamera::FrameBuffer *buffer) override;

	bool processRequest(libcamera::Request *request) override;

private:
	void writeBuffer(const libcamera::Stream *stream,
			 libcamera::FrameBuffer *buffer);

	std::map<const libcamera::Stream *, std::string> streamNames_;
	std::string pattern_;
	std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;
	std::map<const libcamera::FrameBuffer::Plane *, uint8_t *> planeData_;
};

#endif /* __CAM_FILE_SINK_H__ */
