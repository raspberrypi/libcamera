/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * file_sink.cpp - File Sink
 */

#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/camera.h>

#include "file_sink.h"

using namespace libcamera;

FileSink::FileSink(const std::string &pattern)
	: pattern_(pattern)
{
}

FileSink::~FileSink()
{
	for (auto &iter : mappedBuffers_) {
		void *memory = iter.second.first;
		unsigned int length = iter.second.second;
		munmap(memory, length);
	}
	mappedBuffers_.clear();
}

int FileSink::configure(const libcamera::CameraConfiguration &config)
{
	int ret = FrameSink::configure(config);
	if (ret < 0)
		return ret;

	streamNames_.clear();
	for (unsigned int index = 0; index < config.size(); ++index) {
		const StreamConfiguration &cfg = config.at(index);
		streamNames_[cfg.stream()] = "stream" + std::to_string(index);
	}

	return 0;
}

void FileSink::mapBuffer(FrameBuffer *buffer)
{
	for (const FrameBuffer::Plane &plane : buffer->planes()) {
		void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
				    plane.fd.fd(), 0);

		mappedBuffers_[plane.fd.fd()] =
			std::make_pair(memory, plane.length);
	}
}

bool FileSink::processRequest(Request *request)
{
	for (auto [stream, buffer] : request->buffers())
		writeBuffer(stream, buffer);

	return true;
}

void FileSink::writeBuffer(const Stream *stream, FrameBuffer *buffer)
{
	std::string filename;
	size_t pos;
	int fd, ret = 0;

	if (!pattern_.empty())
		filename = pattern_;

	if (filename.empty() || filename.back() == '/')
		filename += "frame-#.bin";

	pos = filename.find_first_of('#');
	if (pos != std::string::npos) {
		std::stringstream ss;
		ss << streamNames_[stream] << "-" << std::setw(6)
		   << std::setfill('0') << buffer->metadata().sequence;
		filename.replace(pos, 1, ss.str());
	}

	fd = open(filename.c_str(), O_CREAT | O_WRONLY |
		  (pos == std::string::npos ? O_APPEND : O_TRUNC),
		  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
	if (fd == -1) {
		ret = -errno;
		std::cerr << "failed to open file " << filename << ": "
			  << strerror(-ret) << std::endl;
		return;
	}

	for (unsigned int i = 0; i < buffer->planes().size(); ++i) {
		const FrameBuffer::Plane &plane = buffer->planes()[i];
		const FrameMetadata::Plane &meta = buffer->metadata().planes[i];

		void *data = mappedBuffers_[plane.fd.fd()].first;
		unsigned int length = std::min(meta.bytesused, plane.length);

		if (meta.bytesused > plane.length)
			std::cerr << "payload size " << meta.bytesused
				  << " larger than plane size " << plane.length
				  << std::endl;

		ret = ::write(fd, data, length);
		if (ret < 0) {
			ret = -errno;
			std::cerr << "write error: " << strerror(-ret)
				  << std::endl;
			break;
		} else if (ret != (int)length) {
			std::cerr << "write error: only " << ret
				  << " bytes written instead of "
				  << length << std::endl;
			break;
		}
	}

	close(fd);
}
