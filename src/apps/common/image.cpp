/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * image.cpp - Multi-planar image with access to pixel data
 */

#include "image.h"

#include <assert.h>
#include <errno.h>
#include <iostream>
#include <map>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace libcamera;

std::unique_ptr<Image> Image::fromFrameBuffer(const FrameBuffer *buffer, MapMode mode)
{
	std::unique_ptr<Image> image{ new Image() };

	assert(!buffer->planes().empty());

	int mmapFlags = 0;

	if (mode & MapMode::ReadOnly)
		mmapFlags |= PROT_READ;

	if (mode & MapMode::WriteOnly)
		mmapFlags |= PROT_WRITE;

	struct MappedBufferInfo {
		uint8_t *address = nullptr;
		size_t mapLength = 0;
		size_t dmabufLength = 0;
	};
	std::map<int, MappedBufferInfo> mappedBuffers;

	for (const FrameBuffer::Plane &plane : buffer->planes()) {
		const int fd = plane.fd.get();
		if (mappedBuffers.find(fd) == mappedBuffers.end()) {
			const size_t length = lseek(fd, 0, SEEK_END);
			mappedBuffers[fd] = MappedBufferInfo{ nullptr, 0, length };
		}

		const size_t length = mappedBuffers[fd].dmabufLength;

		if (plane.offset > length ||
		    plane.offset + plane.length > length) {
			std::cerr << "plane is out of buffer: buffer length="
				  << length << ", plane offset=" << plane.offset
				  << ", plane length=" << plane.length
				  << std::endl;
			return nullptr;
		}
		size_t &mapLength = mappedBuffers[fd].mapLength;
		mapLength = std::max(mapLength,
				     static_cast<size_t>(plane.offset + plane.length));
	}

	for (const FrameBuffer::Plane &plane : buffer->planes()) {
		const int fd = plane.fd.get();
		auto &info = mappedBuffers[fd];
		if (!info.address) {
			void *address = mmap(nullptr, info.mapLength, mmapFlags,
					     MAP_SHARED, fd, 0);
			if (address == MAP_FAILED) {
				int error = -errno;
				std::cerr << "Failed to mmap plane: "
					  << strerror(-error) << std::endl;
				return nullptr;
			}

			info.address = static_cast<uint8_t *>(address);
			image->maps_.emplace_back(info.address, info.mapLength);
		}

		image->planes_.emplace_back(info.address + plane.offset, plane.length);
	}

	return image;
}

Image::Image() = default;

Image::~Image()
{
	for (Span<uint8_t> &map : maps_)
		munmap(map.data(), map.size());
}

unsigned int Image::numPlanes() const
{
	return planes_.size();
}

Span<uint8_t> Image::data(unsigned int plane)
{
	assert(plane <= planes_.size());
	return planes_[plane];
}

Span<const uint8_t> Image::data(unsigned int plane) const
{
	assert(plane <= planes_.size());
	return planes_[plane];
}
