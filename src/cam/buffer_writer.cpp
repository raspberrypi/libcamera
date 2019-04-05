/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * buffer_writer.cpp - Buffer writer
 */

#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>

#include "buffer_writer.h"

BufferWriter::BufferWriter(const std::string &pattern)
	: pattern_(pattern)
{
}

int BufferWriter::write(libcamera::Buffer *buffer,
			const std::string &streamName)
{
	std::string filename;
	size_t pos;
	int fd, ret = 0;

	filename = pattern_;
	pos = filename.find_first_of('#');
	if (pos != std::string::npos) {
		std::stringstream ss;
		ss << streamName << "-" << std::setw(6)
		   << std::setfill('0') << buffer->sequence();
		filename.replace(pos, 1, ss.str());
	}

	fd = open(filename.c_str(), O_CREAT | O_WRONLY |
		  (pos == std::string::npos ? O_APPEND : O_TRUNC),
		  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
	if (fd == -1)
		return -errno;

	for (libcamera::Plane &plane : buffer->planes()) {
		void *data = plane.mem();
		unsigned int length = plane.length();

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

	return ret;
}
