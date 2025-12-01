
/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2424 Raspberry Pi Ltd
 *
 * v4l2_device.cpp - PiSP V4L2 device helper
 */

#include <algorithm>
#include <assert.h>
#include <fcntl.h>
#include <map>
#include <poll.h>
#include <stdexcept>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>

#include "libpisp/common/utils.hpp"

#include "v4l2_device.hpp"

using namespace libpisp::helpers;

namespace {

struct FormatInfo
{
	unsigned int v4l2_pixfmt;
	unsigned int num_memory_planes;
};

static FormatInfo get_v4l2_format(const std::string &format)
{
	std::map<std::string, FormatInfo> formats {
		{ "RGB888", { V4L2_PIX_FMT_RGB24, 1 } },
		{ "RGBX8888", { V4L2_PIX_FMT_RGBX32, 1 } },
		{ "YUV420P", { V4L2_PIX_FMT_YUV420, 1 } },
		{ "YUV422P", { V4L2_PIX_FMT_YUV422P, 1 } },
		{ "YUV444P", { V4L2_PIX_FMT_YUV444M, 3 } },
		{ "YUYV", { V4L2_PIX_FMT_YUYV, 1 } },
		{ "UYVY", { V4L2_PIX_FMT_UYVY, 1 } },
	};

	auto it = formats.find(format);
	if (it == formats.end())
		return { 0, {} };

	return it->second;
}

} // namespace

V4l2Device::V4l2Device(const std::string &device)
	: fd_(device, O_RDWR | O_NONBLOCK | O_CLOEXEC), num_memory_planes_(1)
{
	struct v4l2_capability caps;

	int ret = ioctl(fd_.Get(), VIDIOC_QUERYCAP, &caps);
	if (ret < 0)
		throw std::runtime_error("Cannot query device caps");

	if (caps.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
		buf_type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	else if (caps.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE)
		buf_type_ = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	else if (caps.capabilities & V4L2_CAP_META_OUTPUT)
		buf_type_ = V4L2_BUF_TYPE_META_OUTPUT;
	else
		throw std::runtime_error("Invalid buffer_type - caps: " + std::to_string(caps.capabilities));
}

V4l2Device::~V4l2Device()
{
	ReturnBuffers();
	Close();
}

int V4l2Device::RequestBuffers(unsigned int count)
{
	int ret;

	ReturnBuffers();

	v4l2_requestbuffers req_bufs {};

	req_bufs.count = count;
	req_bufs.type = buf_type_;
	req_bufs.memory = V4L2_MEMORY_MMAP;

	ret = ioctl(fd_.Get(), VIDIOC_REQBUFS, &req_bufs);
	if (ret < 0)
		throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::to_string(ret));

	for (unsigned int i = 0; i < req_bufs.count; i++)
	{
		v4l2_plane planes[VIDEO_MAX_PLANES] = {};
		v4l2_buffer buffer {};

		buffer.index = i;
		buffer.type = buf_type_;
		buffer.memory = V4L2_MEMORY_MMAP;
		if (!isMeta())
		{
			buffer.m.planes = planes;
			buffer.length = num_memory_planes_;
		}

		ret = ioctl(fd_.Get(), VIDIOC_QUERYBUF, &buffer);
		if (ret < 0)
			throw std::runtime_error("VIDIOC_QUERYBUF failed: " + std::to_string(ret));

		// Don't keep this pointer dangling when putting into v4l2_buffers_.
		buffer.m.planes = NULL;
		v4l2_buffers_.emplace_back(buffer);
		available_buffers_.push(i);

		for (unsigned int p = 0; p < num_memory_planes_; p++)
		{
			size_t size = !isMeta() ? planes[p].length : buffer.length;
			unsigned int offset = !isMeta() ? planes[p].m.mem_offset : buffer.m.offset;

			void *mem = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_.Get(), offset);
			if (mem == MAP_FAILED)
				throw std::runtime_error("Unable to mmap buffer");

			v4l2_buffers_.back().size[p] = size;
			v4l2_buffers_.back().mem[p] = (uint8_t *)mem;
		}
	}

	return v4l2_buffers_.size();
}

void V4l2Device::ReturnBuffers()
{
	v4l2_requestbuffers req_bufs {};

	if (!v4l2_buffers_.size())
		return;

	for (auto const &b : v4l2_buffers_)
	{
		for (unsigned int p = 0; p < num_memory_planes_; p++)
			munmap(b.mem[p], b.size[p]);
	}

	req_bufs.type = buf_type_;
	req_bufs.count = 0;
	req_bufs.memory = V4L2_MEMORY_MMAP;
	ioctl(fd_.Get(), VIDIOC_REQBUFS, &req_bufs);

	v4l2_buffers_.clear();
}

std::optional<V4l2Device::Buffer> V4l2Device::AcquireBuffer()
{
	if (available_buffers_.empty())
		return {};

	unsigned int index = available_buffers_.front();
	available_buffers_.pop();
	return findBuffer(index);
}

void V4l2Device::ReleaseBuffer(const Buffer &buffer)
{
	available_buffers_.push(buffer.buffer.index);
}

int V4l2Device::QueueBuffer(unsigned int index)
{
	std::optional<Buffer> buf = findBuffer(index);
	if (!buf)
		return -1;

	v4l2_plane planes[VIDEO_MAX_PLANES] = {};
	if (!isMeta())
	{
		buf->buffer.m.planes = planes;
		buf->buffer.length = num_memory_planes_;
		for (unsigned int p = 0; p < num_memory_planes_; p++)
		{
			buf->buffer.m.planes[p].bytesused = buf->size[p];
			buf->buffer.m.planes[p].length = buf->size[p];
		}
	}
	else
		buf->buffer.bytesused = buf->size[0];

	buf->buffer.timestamp.tv_sec = time(NULL);
	buf->buffer.field = V4L2_FIELD_NONE;
	buf->buffer.flags = 0;

	int ret = ioctl(fd_.Get(), VIDIOC_QBUF, &buf->buffer);
	if (ret < 0)
		throw std::runtime_error("Unable to queue buffer: " + std::string(strerror(errno)));

	return ret;
}

int V4l2Device::DequeueBuffer(unsigned int timeout_ms)
{
	short int poll_event = isOutput() ? POLLOUT : POLLIN;
	pollfd p = { fd_.Get(), poll_event, 0 };

	int ret = poll(&p, 1, timeout_ms);
	if (ret <= 0)
		return -1;

	if (!(p.revents & poll_event))
		return -1;

	v4l2_buffer buf = {};
	v4l2_plane planes[VIDEO_MAX_PLANES] = {};

	buf.type = buf_type_;
	buf.memory = V4L2_MEMORY_MMAP;
	if (!isMeta())
	{
		buf.m.planes = planes;
		buf.length = VIDEO_MAX_PLANES;
	}

	ret = ioctl(fd_.Get(), VIDIOC_DQBUF, &buf);
	if (ret)
		return -1;

	return buf.index;
}

void V4l2Device::SetFormat(const pisp_image_format_config &format, bool use_opaque_format)
{
	struct v4l2_format f = {};
	FormatInfo info = get_v4l2_format(libpisp::get_pisp_image_format(format.format));

	num_memory_planes_ = info.num_memory_planes;

	f.type = buf_type_;
	f.fmt.pix_mp.width = format.width;
	f.fmt.pix_mp.height = format.height;
	f.fmt.pix_mp.pixelformat = info.v4l2_pixfmt;
	f.fmt.pix_mp.field = V4L2_FIELD_NONE;
	f.fmt.pix_mp.num_planes = num_memory_planes_;

	unsigned int num_image_planes = libpisp::num_planes((pisp_image_format)format.format);

	if (use_opaque_format || info.v4l2_pixfmt == 0)
	{
		// This format is not specified by V4L2, we use an opaque buffer buffer as a workaround.
		// Size the dimensions down so the kernel drive does not attempt to resize it.
		f.fmt.pix_mp.width = 16;
		f.fmt.pix_mp.height = 16;
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV444M;
		num_memory_planes_ = 3;
		f.fmt.pix_mp.plane_fmt[0].bytesperline = format.stride;

		f.fmt.pix_mp.plane_fmt[0].sizeimage = 0;
		for (unsigned int i = 0; i < 3; i++)
			f.fmt.pix_mp.plane_fmt[0].sizeimage += libpisp::get_plane_size(format, i);

		f.fmt.pix_mp.plane_fmt[1].sizeimage = f.fmt.pix_mp.plane_fmt[2].sizeimage = f.fmt.pix_mp.plane_fmt[0].sizeimage;
		f.fmt.pix_mp.plane_fmt[1].bytesperline = f.fmt.pix_mp.plane_fmt[2].bytesperline = format.stride2;
	}
	else
	{
		unsigned int p = 0;
		for (; p < num_memory_planes_; p++)
		{
			const unsigned int stride = p == 0 ? format.stride : format.stride2;
			// Wallpaper stride is not something the V4L2 kernel knows about!
			f.fmt.pix_mp.plane_fmt[p].bytesperline = stride;
			f.fmt.pix_mp.plane_fmt[p].sizeimage = libpisp::get_plane_size(format, p);
		}

		for (; p < num_image_planes; p++)
			f.fmt.pix_mp.plane_fmt[num_memory_planes_ - 1].sizeimage += libpisp::get_plane_size(format, p);
	}

	int ret = ioctl(fd_.Get(), VIDIOC_S_FMT, &f);
	if (ret)
		throw std::runtime_error("Unable to set format: " + std::string(strerror(errno)));
}

void V4l2Device::StreamOn()
{
	int ret = ioctl(fd_.Get(), VIDIOC_STREAMON, &buf_type_);
	if (ret < 0)
		throw std::runtime_error("Stream on failed: " + std::string(strerror(errno)));
}

void V4l2Device::StreamOff()
{
	int ret = ioctl(fd_.Get(), VIDIOC_STREAMOFF, &buf_type_);
	if (ret < 0)
		throw std::runtime_error("Stream off failed: " + std::string(strerror(errno)));
}

std::optional<V4l2Device::Buffer> V4l2Device::findBuffer(unsigned int index) const
{
	auto it = std::find_if(v4l2_buffers_.begin(), v4l2_buffers_.end(),
						   [index](auto const &b) { return b.buffer.index == index; });
	if (it == v4l2_buffers_.end())
	{
		throw std::runtime_error("find buffers failed");
		return {};
	}

	return *it;
}
