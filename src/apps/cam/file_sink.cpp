/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * File Sink
 */

#include "file_sink.h"

#include <array>
#include <assert.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <utility>

#include <libcamera/camera.h>

#include "../common/dng_writer.h"
#include "../common/image.h"
#include "../common/ppm_writer.h"

using namespace libcamera;

FileSink::FileSink([[maybe_unused]] const libcamera::Camera *camera,
		   const std::map<const libcamera::Stream *, std::string> &streamNames)
	:
#ifdef HAVE_TIFF
	  camera_(camera),
#endif
	  pattern_(kDefaultFilePattern), fileType_(FileType::Binary),
	  streamNames_(streamNames)
{
}

FileSink::~FileSink()
{
}

int FileSink::setFilePattern(const std::string &pattern)
{
	static const std::array<std::pair<std::string, FileType>, 2> types{{
		{ ".dng", FileType::Dng },
		{ ".ppm", FileType::Ppm },
	}};

	pattern_ = pattern;

	if (pattern_.empty() || pattern_.back() == '/')
		pattern_ += kDefaultFilePattern;

	fileType_ = FileType::Binary;

	for (const auto &type : types) {
		if (pattern_.size() < type.first.size())
			continue;

		if (pattern_.find(type.first, pattern_.size() - type.first.size()) !=
		    std::string::npos) {
			fileType_ = type.second;
			break;
		}
	}

#ifndef HAVE_TIFF
	if (fileType_ == FileType::Dng) {
		std::cerr << "DNG support not available" << std::endl;
		return -EINVAL;
	}
#endif /* HAVE_TIFF */

	return 0;
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
	std::string filename = pattern_;
	size_t pos;
	int fd, ret = 0;

	pos = filename.find_first_of('#');
	if (pos != std::string::npos) {
		std::stringstream ss;
		ss << streamNames_[stream] << "-" << std::setw(6)
		   << std::setfill('0') << buffer->metadata().sequence;
		filename.replace(pos, 1, ss.str());
	}

	Image *image = mappedBuffers_[buffer].get();

#ifdef HAVE_TIFF
	if (fileType_ == FileType::Dng) {
		ret = DNGWriter::write(filename.c_str(), camera_,
				       stream->configuration(), metadata,
				       buffer, image->data(0).data());
		if (ret < 0)
			std::cerr << "failed to write DNG file `" << filename
				  << "'" << std::endl;

		return;
	}
#endif /* HAVE_TIFF */
	if (fileType_ == FileType::Ppm) {
		ret = PPMWriter::write(filename.c_str(), stream->configuration(),
				       image->data(0));
		if (ret < 0)
			std::cerr << "failed to write PPM file `" << filename
				  << "'" << std::endl;

		return;
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
		/*
		 * This was formerly a local "const FrameMetadata::Plane &"
		 * however this causes a false positive warning for dangling
		 * references on gcc 13.
		 */
		const unsigned int bytesused = buffer->metadata().planes()[i].bytesused;

		Span<uint8_t> data = image->data(i);
		const unsigned int length = std::min<unsigned int>(bytesused, data.size());

		if (bytesused > data.size())
			std::cerr << "payload size " << bytesused
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
