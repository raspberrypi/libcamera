/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * file_sink.cpp - File Sink
 */

#include <assert.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>

#include <libcamera/camera.h>

#include "../common/dng_writer.h"
#include "../common/image.h"

#include "file_sink.h"

using namespace libcamera;

FileSink::FileSink([[maybe_unused]] const libcamera::Camera *camera,
		   const std::map<const libcamera::Stream *, std::string> &streamNames,
		   const std::string &pattern)
	:
#ifdef HAVE_TIFF
	  camera_(camera),
#endif
	  streamNames_(streamNames), pattern_(pattern)
{
}

FileSink::~FileSink()
{
}

int FileSink::configure(const libcamera::CameraConfiguration &config)
{
	int ret = FrameSink::configure(config);
	if (ret < 0)
		return ret;

	return 0;
}

void FileSink::mapBuffer(FrameBuffer *buffer)
{
	std::unique_ptr<Image> image =
		Image::fromFrameBuffer(buffer, Image::MapMode::ReadOnly);
	assert(image != nullptr);

	mappedBuffers_[buffer] = std::move(image);
}

bool FileSink::processRequest(Request *request)
{
	for (auto [stream, buffer] : request->buffers())
		writeBuffer(stream, buffer, request->metadata());

	return true;
}

void FileSink::writeBuffer(const Stream *stream, FrameBuffer *buffer,
			   [[maybe_unused]] const ControlList &metadata)
{
	std::string filename;
	size_t pos;
	int fd, ret = 0;

	if (!pattern_.empty())
		filename = pattern_;

#ifdef HAVE_TIFF
	bool dng = filename.find(".dng", filename.size() - 4) != std::string::npos;
#endif /* HAVE_TIFF */

	if (filename.empty() || filename.back() == '/')
		filename += "frame-#.bin";

	pos = filename.find_first_of('#');
	if (pos != std::string::npos) {
		std::stringstream ss;
		ss << streamNames_[stream] << "-" << std::setw(6)
		   << std::setfill('0') << buffer->metadata().sequence;
		filename.replace(pos, 1, ss.str());
	}

	Image *image = mappedBuffers_[buffer].get();

#ifdef HAVE_TIFF
	if (dng) {
		ret = DNGWriter::write(filename.c_str(), camera_,
				       stream->configuration(), metadata,
				       buffer, image->data(0).data());
		if (ret < 0)
			std::cerr << "failed to write DNG file `" << filename
				  << "'" << std::endl;

		return;
	}
#endif /* HAVE_TIFF */

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
		const FrameMetadata::Plane &meta = buffer->metadata().planes()[i];

		Span<uint8_t> data = image->data(i);
		unsigned int length = std::min<unsigned int>(meta.bytesused, data.size());

		if (meta.bytesused > data.size())
			std::cerr << "payload size " << meta.bytesused
				  << " larger than plane size " << data.size()
				  << std::endl;

		ret = ::write(fd, data.data(), length);
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
