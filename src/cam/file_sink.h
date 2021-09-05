/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * file_sink.h - File Sink
 */
#ifndef __CAM_FILE_SINK_H__
#define __CAM_FILE_SINK_H__

#include <map>
#include <memory>
#include <string>

#include <libcamera/stream.h>

#include "frame_sink.h"

class Image;

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
	std::map<libcamera::FrameBuffer *, std::unique_ptr<Image>> mappedBuffers_;
};

#endif /* __CAM_FILE_SINK_H__ */
