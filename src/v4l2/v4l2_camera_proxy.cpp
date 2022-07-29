/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_camera_proxy.cpp - Proxy to V4L2 compatibility camera
 */

#include "v4l2_camera_proxy.h"

#include <algorithm>
#include <array>
#include <errno.h>
#include <numeric>
#include <set>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <libcamera/base/log.h>
#include <libcamera/base/object.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/formats.h>

#include "libcamera/internal/formats.h"

#include "v4l2_camera.h"
#include "v4l2_camera_file.h"
#include "v4l2_compat_manager.h"

#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))

using namespace libcamera;

LOG_DECLARE_CATEGORY(V4L2Compat)

V4L2CameraProxy::V4L2CameraProxy(unsigned int index,
				 std::shared_ptr<Camera> camera)
	: refcount_(0), index_(index), bufferCount_(0), currentBuf_(0),
	  vcam_(std::make_unique<V4L2Camera>(camera)), owner_(nullptr)
{
	querycap(camera);
}

int V4L2CameraProxy::open(V4L2CameraFile *file)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	MutexLocker locker(proxyMutex_);

	if (refcount_++) {
		files_.insert(file);
		return 0;
	}

	/*
	 * We open the camera here, once, and keep it open until the last
	 * V4L2CameraFile is closed. The proxy is initially not owned by any
	 * file. The first file that calls reqbufs with count > 0 or s_fmt
	 * will become the owner, and no other file will be allowed to call
	 * buffer-related ioctls (except querybuf), set the format, or start or
	 * stop the stream until ownership is released with a call to reqbufs
	 * with count = 0.
	 */

	int ret = vcam_->open(&streamConfig_);
	if (ret < 0) {
		refcount_--;
		return ret;
	}

	setFmtFromConfig(streamConfig_);

	files_.insert(file);

	return 0;
}

void V4L2CameraProxy::close(V4L2CameraFile *file)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	MutexLocker locker(proxyMutex_);

	files_.erase(file);

	release(file);

	if (--refcount_ > 0)
		return;

	vcam_->close();
}

void *V4L2CameraProxy::mmap(V4L2CameraFile *file, void *addr, size_t length,
			    int prot, int flags, off64_t offset)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	MutexLocker locker(proxyMutex_);

	/*
	 * Mimic the videobuf2 behaviour, which requires PROT_READ and
	 * MAP_SHARED.
	 */
	if (!(prot & PROT_READ)) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	if (!(flags & MAP_SHARED)) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	unsigned int index = offset / sizeimage_;
	if (static_cast<off_t>(index * sizeimage_) != offset ||
	    length != sizeimage_) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	int fd = vcam_->getBufferFd(index);
	if (fd < 0) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	void *map = V4L2CompatManager::instance()->fops().mmap(addr, length, prot,
							       flags, fd, 0);
	if (map == MAP_FAILED)
		return map;

	buffers_[index].flags |= V4L2_BUF_FLAG_MAPPED;
	mmaps_[map] = index;

	return map;
}

int V4L2CameraProxy::munmap(V4L2CameraFile *file, void *addr, size_t length)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	MutexLocker locker(proxyMutex_);

	auto iter = mmaps_.find(addr);
	if (iter == mmaps_.end() || length != sizeimage_) {
		errno = EINVAL;
		return -1;
	}

	if (V4L2CompatManager::instance()->fops().munmap(addr, length))
		LOG(V4L2Compat, Error) << "Failed to unmap " << addr
				       << " with length " << length;

	buffers_[iter->second].flags &= ~V4L2_BUF_FLAG_MAPPED;
	mmaps_.erase(iter);

	return 0;
}

bool V4L2CameraProxy::validateBufferType(uint32_t type)
{
	return type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

bool V4L2CameraProxy::validateMemoryType(uint32_t memory)
{
	return memory == V4L2_MEMORY_MMAP;
}

void V4L2CameraProxy::setFmtFromConfig(const StreamConfiguration &streamConfig)
{
	const Size &size = streamConfig.size;

	v4l2PixFormat_.width        = size.width;
	v4l2PixFormat_.height       = size.height;
	v4l2PixFormat_.pixelformat  = V4L2PixelFormat::fromPixelFormat(streamConfig.pixelFormat)[0];
	v4l2PixFormat_.field        = V4L2_FIELD_NONE;
	v4l2PixFormat_.bytesperline = streamConfig.stride;
	v4l2PixFormat_.sizeimage    = streamConfig.frameSize;
	v4l2PixFormat_.colorspace   = V4L2_COLORSPACE_SRGB;
	v4l2PixFormat_.priv         = V4L2_PIX_FMT_PRIV_MAGIC;
	v4l2PixFormat_.ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	v4l2PixFormat_.quantization = V4L2_QUANTIZATION_DEFAULT;
	v4l2PixFormat_.xfer_func    = V4L2_XFER_FUNC_DEFAULT;

	sizeimage_ = streamConfig.frameSize;
}

void V4L2CameraProxy::querycap(std::shared_ptr<Camera> camera)
{
	std::string driver = "libcamera";
	std::string bus_info = driver + ":" + std::to_string(index_);

	utils::strlcpy(reinterpret_cast<char *>(capabilities_.driver), driver.c_str(),
		       sizeof(capabilities_.driver));
	utils::strlcpy(reinterpret_cast<char *>(capabilities_.card), camera->id().c_str(),
		       sizeof(capabilities_.card));
	utils::strlcpy(reinterpret_cast<char *>(capabilities_.bus_info), bus_info.c_str(),
		       sizeof(capabilities_.bus_info));
	/* \todo Put this in a header/config somewhere. */
	capabilities_.version = KERNEL_VERSION(5, 2, 0);
	capabilities_.device_caps = V4L2_CAP_VIDEO_CAPTURE
				  | V4L2_CAP_STREAMING
				  | V4L2_CAP_EXT_PIX_FORMAT;
	capabilities_.capabilities = capabilities_.device_caps
				   | V4L2_CAP_DEVICE_CAPS;
	memset(capabilities_.reserved, 0, sizeof(capabilities_.reserved));
}

void V4L2CameraProxy::updateBuffers()
{
	std::vector<V4L2Camera::Buffer> completedBuffers = vcam_->completedBuffers();
	for (const V4L2Camera::Buffer &buffer : completedBuffers) {
		const FrameMetadata &fmd = buffer.data_;
		struct v4l2_buffer &buf = buffers_[buffer.index_];

		switch (fmd.status) {
		case FrameMetadata::FrameSuccess:
			buf.bytesused = std::accumulate(fmd.planes().begin(),
							fmd.planes().end(), 0,
							[](unsigned int total, const auto &plane) {
								return total + plane.bytesused;
							});
			buf.field = V4L2_FIELD_NONE;
			buf.timestamp.tv_sec = fmd.timestamp / 1000000000;
			buf.timestamp.tv_usec = (fmd.timestamp / 1000) % 1000000;
			buf.sequence = fmd.sequence;

			buf.flags |= V4L2_BUF_FLAG_DONE;
			break;
		case FrameMetadata::FrameError:
			buf.flags |= V4L2_BUF_FLAG_ERROR;
			break;
		default:
			break;
		}
	}
}

int V4L2CameraProxy::vidioc_querycap(V4L2CameraFile *file, struct v4l2_capability *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	*arg = capabilities_;

	return 0;
}

int V4L2CameraProxy::vidioc_enum_framesizes(V4L2CameraFile *file, struct v4l2_frmsizeenum *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	V4L2PixelFormat v4l2Format = V4L2PixelFormat(arg->pixel_format);
	PixelFormat format = v4l2Format.toPixelFormat();
	/*
	 * \todo This might need to be expanded as few pipeline handlers
	 * report StreamFormats.
	 */
	const std::vector<Size> &frameSizes = streamConfig_.formats().sizes(format);

	if (arg->index >= frameSizes.size())
		return -EINVAL;

	arg->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	arg->discrete.width = frameSizes[arg->index].width;
	arg->discrete.height = frameSizes[arg->index].height;
	memset(arg->reserved, 0, sizeof(arg->reserved));

	return 0;
}

int V4L2CameraProxy::vidioc_enum_fmt(V4L2CameraFile *file, struct v4l2_fmtdesc *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (!validateBufferType(arg->type) ||
	    arg->index >= streamConfig_.formats().pixelformats().size())
		return -EINVAL;

	PixelFormat format = streamConfig_.formats().pixelformats()[arg->index];
	V4L2PixelFormat v4l2Format = V4L2PixelFormat::fromPixelFormat(format)[0];

	arg->flags = format == formats::MJPEG ? V4L2_FMT_FLAG_COMPRESSED : 0;
	utils::strlcpy(reinterpret_cast<char *>(arg->description),
		       v4l2Format.description(), sizeof(arg->description));
	arg->pixelformat = v4l2Format;

	memset(arg->reserved, 0, sizeof(arg->reserved));

	return 0;
}

int V4L2CameraProxy::vidioc_g_fmt(V4L2CameraFile *file, struct v4l2_format *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (!validateBufferType(arg->type))
		return -EINVAL;

	memset(&arg->fmt, 0, sizeof(arg->fmt));
	arg->fmt.pix = v4l2PixFormat_;

	return 0;
}

int V4L2CameraProxy::tryFormat(struct v4l2_format *arg)
{
	V4L2PixelFormat v4l2Format = V4L2PixelFormat(arg->fmt.pix.pixelformat);
	PixelFormat format = v4l2Format.toPixelFormat();
	Size size(arg->fmt.pix.width, arg->fmt.pix.height);

	StreamConfiguration config;
	int ret = vcam_->validateConfiguration(format, size, &config);
	if (ret < 0) {
		LOG(V4L2Compat, Error)
			<< "Failed to negotiate a valid format: "
			<< format;
		return -EINVAL;
	}

	arg->fmt.pix.width        = config.size.width;
	arg->fmt.pix.height       = config.size.height;
	arg->fmt.pix.pixelformat  = V4L2PixelFormat::fromPixelFormat(config.pixelFormat)[0];
	arg->fmt.pix.field        = V4L2_FIELD_NONE;
	arg->fmt.pix.bytesperline = config.stride;
	arg->fmt.pix.sizeimage    = config.frameSize;
	arg->fmt.pix.colorspace   = V4L2_COLORSPACE_SRGB;
	arg->fmt.pix.priv         = V4L2_PIX_FMT_PRIV_MAGIC;
	arg->fmt.pix.ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	arg->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
	arg->fmt.pix.xfer_func    = V4L2_XFER_FUNC_DEFAULT;

	return 0;
}

int V4L2CameraProxy::vidioc_s_fmt(V4L2CameraFile *file, struct v4l2_format *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (!validateBufferType(arg->type))
		return -EINVAL;

	if (file->priority() < maxPriority())
		return -EBUSY;

	int ret = acquire(file);
	if (ret < 0)
		return ret;

	ret = tryFormat(arg);
	if (ret < 0)
		return ret;

	Size size(arg->fmt.pix.width, arg->fmt.pix.height);
	V4L2PixelFormat v4l2Format = V4L2PixelFormat(arg->fmt.pix.pixelformat);
	ret = vcam_->configure(&streamConfig_, size, v4l2Format.toPixelFormat(),
			       bufferCount_);
	if (ret < 0)
		return -EINVAL;

	setFmtFromConfig(streamConfig_);

	return 0;
}

int V4L2CameraProxy::vidioc_try_fmt(V4L2CameraFile *file, struct v4l2_format *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (!validateBufferType(arg->type))
		return -EINVAL;

	int ret = tryFormat(arg);
	if (ret < 0)
		return ret;

	return 0;
}

enum v4l2_priority V4L2CameraProxy::maxPriority()
{
	auto max = std::max_element(files_.begin(), files_.end(),
				    [](const V4L2CameraFile *a, const V4L2CameraFile *b) {
					    return a->priority() < b->priority();
				    });
	return max != files_.end() ? (*max)->priority() : V4L2_PRIORITY_UNSET;
}

int V4L2CameraProxy::vidioc_g_priority(V4L2CameraFile *file, enum v4l2_priority *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	*arg = maxPriority();

	return 0;
}

int V4L2CameraProxy::vidioc_s_priority(V4L2CameraFile *file, enum v4l2_priority *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (*arg > V4L2_PRIORITY_RECORD)
		return -EINVAL;

	if (file->priority() < maxPriority())
		return -EBUSY;

	file->setPriority(*arg);

	return 0;
}

int V4L2CameraProxy::vidioc_enuminput(V4L2CameraFile *file, struct v4l2_input *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (arg->index != 0)
		return -EINVAL;

	memset(arg, 0, sizeof(*arg));

	utils::strlcpy(reinterpret_cast<char *>(arg->name),
		       reinterpret_cast<char *>(capabilities_.card),
		       sizeof(arg->name));
	arg->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

int V4L2CameraProxy::vidioc_g_input(V4L2CameraFile *file, int *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	*arg = 0;

	return 0;
}

int V4L2CameraProxy::vidioc_s_input(V4L2CameraFile *file, int *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (*arg != 0)
		return -EINVAL;

	return 0;
}

void V4L2CameraProxy::freeBuffers()
{
	vcam_->freeBuffers();
	buffers_.clear();
	bufferCount_ = 0;
}

int V4L2CameraProxy::vidioc_reqbufs(V4L2CameraFile *file, struct v4l2_requestbuffers *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (!validateBufferType(arg->type) ||
	    !validateMemoryType(arg->memory))
		return -EINVAL;

	LOG(V4L2Compat, Debug) << arg->count << " buffers requested ";

	if (file->priority() < maxPriority())
		return -EBUSY;

	if (!hasOwnership(file) && owner_)
		return -EBUSY;

	arg->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP;
	arg->flags = 0;
	memset(arg->reserved, 0, sizeof(arg->reserved));

	if (arg->count == 0) {
		/* \todo Add buffer orphaning support */
		if (!mmaps_.empty())
			return -EBUSY;

		if (vcam_->isRunning())
			return -EBUSY;

		freeBuffers();
		release(file);

		return 0;
	}

	if (bufferCount_ > 0)
		freeBuffers();

	Size size(v4l2PixFormat_.width, v4l2PixFormat_.height);
	V4L2PixelFormat v4l2Format = V4L2PixelFormat(v4l2PixFormat_.pixelformat);
	int ret = vcam_->configure(&streamConfig_, size,
				   v4l2Format.toPixelFormat(), arg->count);
	if (ret < 0)
		return -EINVAL;

	setFmtFromConfig(streamConfig_);

	arg->count = streamConfig_.bufferCount;
	bufferCount_ = arg->count;

	ret = vcam_->allocBuffers(arg->count);
	if (ret < 0) {
		arg->count = 0;
		return ret;
	}

	buffers_.resize(arg->count);
	for (unsigned int i = 0; i < arg->count; i++) {
		struct v4l2_buffer buf = {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.length = v4l2PixFormat_.sizeimage;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.m.offset = i * v4l2PixFormat_.sizeimage;
		buf.index = i;
		buf.flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

		buffers_[i] = buf;
	}

	LOG(V4L2Compat, Debug) << "Allocated " << arg->count << " buffers";

	acquire(file);

	return 0;
}

int V4L2CameraProxy::vidioc_querybuf(V4L2CameraFile *file, struct v4l2_buffer *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (arg->index >= bufferCount_)
		return -EINVAL;

	if (!validateBufferType(arg->type) ||
	    arg->index >= bufferCount_)
		return -EINVAL;

	updateBuffers();

	*arg = buffers_[arg->index];

	return 0;
}

int V4L2CameraProxy::vidioc_prepare_buf(V4L2CameraFile *file, struct v4l2_buffer *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__
		<< "(index=" << arg->index << ")";

	if (!hasOwnership(file))
		return -EBUSY;

	if (arg->index >= bufferCount_)
		return -EINVAL;

	if (arg->flags & V4L2_BUF_FLAG_REQUEST_FD)
		return -EINVAL;

	if (!validateBufferType(arg->type) ||
	    !validateMemoryType(arg->memory))
		return -EINVAL;

	struct v4l2_buffer &buffer = buffers_[arg->index];

	if (buffer.flags & V4L2_BUF_FLAG_QUEUED ||
	    buffer.flags & V4L2_BUF_FLAG_PREPARED)
		return -EINVAL;

	buffer.flags |= V4L2_BUF_FLAG_PREPARED;

	arg->flags = buffer.flags;

	return 0;
}

int V4L2CameraProxy::vidioc_qbuf(V4L2CameraFile *file, struct v4l2_buffer *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__
		<< "(index=" << arg->index << ")";

	if (arg->index >= bufferCount_)
		return -EINVAL;

	if (buffers_[arg->index].flags & V4L2_BUF_FLAG_QUEUED)
		return -EINVAL;

	if (!hasOwnership(file))
		return -EBUSY;

	if (!validateBufferType(arg->type) ||
	    !validateMemoryType(arg->memory) ||
	    arg->index >= bufferCount_)
		return -EINVAL;

	int ret = vcam_->qbuf(arg->index);
	if (ret < 0)
		return ret;

	buffers_[arg->index].flags |= V4L2_BUF_FLAG_QUEUED;

	arg->flags = buffers_[arg->index].flags;

	return ret;
}

int V4L2CameraProxy::vidioc_dqbuf(V4L2CameraFile *file, struct v4l2_buffer *arg,
				  Mutex *lock)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (arg->index >= bufferCount_)
		return -EINVAL;

	if (!hasOwnership(file))
		return -EBUSY;

	if (!vcam_->isRunning())
		return -EINVAL;

	if (!validateBufferType(arg->type) ||
	    !validateMemoryType(arg->memory))
		return -EINVAL;

	if (!file->nonBlocking()) {
		lock->unlock();
		vcam_->waitForBufferAvailable();
		lock->lock();
	} else if (!vcam_->isBufferAvailable())
		return -EAGAIN;

	/*
	 * We need to check here again in case stream was turned off while we
	 * were blocked on waitForBufferAvailable().
	 */
	if (!vcam_->isRunning())
		return -EINVAL;

	updateBuffers();

	struct v4l2_buffer &buf = buffers_[currentBuf_];

	buf.flags &= ~(V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE | V4L2_BUF_FLAG_PREPARED);
	buf.length = sizeimage_;
	*arg = buf;

	currentBuf_ = (currentBuf_ + 1) % bufferCount_;

	uint64_t data;
	int ret = ::read(file->efd(), &data, sizeof(data));
	if (ret != sizeof(data))
		LOG(V4L2Compat, Error) << "Failed to clear eventfd POLLIN";

	return 0;
}

int V4L2CameraProxy::vidioc_expbuf(V4L2CameraFile *file, struct v4l2_exportbuffer *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (!hasOwnership(file))
		return -EBUSY;

	/* \todo Verify that the memory type is MMAP when adding DMABUF support */
	if (!validateBufferType(arg->type))
		return -EINVAL;

	if (arg->index >= bufferCount_)
		return -EINVAL;

	if (arg->flags & ~(O_CLOEXEC | O_ACCMODE))
		return -EINVAL;

	memset(arg->reserved, 0, sizeof(arg->reserved));

	/* \todo honor the O_ACCMODE flags passed to this function */
	arg->fd = fcntl(vcam_->getBufferFd(arg->index),
			arg->flags & O_CLOEXEC ? F_DUPFD_CLOEXEC : F_DUPFD, 0);

	return 0;
}

int V4L2CameraProxy::vidioc_streamon(V4L2CameraFile *file, int *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (bufferCount_ == 0)
		return -EINVAL;

	if (!validateBufferType(*arg))
		return -EINVAL;

	if (file->priority() < maxPriority())
		return -EBUSY;

	if (!hasOwnership(file))
		return -EBUSY;

	if (vcam_->isRunning())
		return 0;

	currentBuf_ = 0;

	return vcam_->streamOn();
}

int V4L2CameraProxy::vidioc_streamoff(V4L2CameraFile *file, int *arg)
{
	LOG(V4L2Compat, Debug)
		<< "[" << file->description() << "] " << __func__ << "()";

	if (!validateBufferType(*arg))
		return -EINVAL;

	if (file->priority() < maxPriority())
		return -EBUSY;

	if (!hasOwnership(file) && owner_)
		return -EBUSY;

	int ret = vcam_->streamOff();

	for (struct v4l2_buffer &buf : buffers_)
		buf.flags &= ~(V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE);

	return ret;
}

const std::set<unsigned long> V4L2CameraProxy::supportedIoctls_ = {
	VIDIOC_QUERYCAP,
	VIDIOC_ENUM_FRAMESIZES,
	VIDIOC_ENUM_FMT,
	VIDIOC_G_FMT,
	VIDIOC_S_FMT,
	VIDIOC_TRY_FMT,
	VIDIOC_G_PRIORITY,
	VIDIOC_S_PRIORITY,
	VIDIOC_ENUMINPUT,
	VIDIOC_G_INPUT,
	VIDIOC_S_INPUT,
	VIDIOC_REQBUFS,
	VIDIOC_QUERYBUF,
	VIDIOC_PREPARE_BUF,
	VIDIOC_QBUF,
	VIDIOC_DQBUF,
	VIDIOC_EXPBUF,
	VIDIOC_STREAMON,
	VIDIOC_STREAMOFF,
};

int V4L2CameraProxy::ioctl(V4L2CameraFile *file, unsigned long request, void *arg)
{
	MutexLocker locker(proxyMutex_);

	if (!arg && (_IOC_DIR(request) & _IOC_WRITE)) {
		errno = EFAULT;
		return -1;
	}

	if (supportedIoctls_.find(request) == supportedIoctls_.end()) {
		errno = ENOTTY;
		return -1;
	}

	if (!arg && (_IOC_DIR(request) & _IOC_READ)) {
		errno = EFAULT;
		return -1;
	}

	int ret;
	switch (request) {
	case VIDIOC_QUERYCAP:
		ret = vidioc_querycap(file, static_cast<struct v4l2_capability *>(arg));
		break;
	case VIDIOC_ENUM_FRAMESIZES:
		ret = vidioc_enum_framesizes(file, static_cast<struct v4l2_frmsizeenum *>(arg));
		break;
	case VIDIOC_ENUM_FMT:
		ret = vidioc_enum_fmt(file, static_cast<struct v4l2_fmtdesc *>(arg));
		break;
	case VIDIOC_G_FMT:
		ret = vidioc_g_fmt(file, static_cast<struct v4l2_format *>(arg));
		break;
	case VIDIOC_S_FMT:
		ret = vidioc_s_fmt(file, static_cast<struct v4l2_format *>(arg));
		break;
	case VIDIOC_TRY_FMT:
		ret = vidioc_try_fmt(file, static_cast<struct v4l2_format *>(arg));
		break;
	case VIDIOC_G_PRIORITY:
		ret = vidioc_g_priority(file, static_cast<enum v4l2_priority *>(arg));
		break;
	case VIDIOC_S_PRIORITY:
		ret = vidioc_s_priority(file, static_cast<enum v4l2_priority *>(arg));
		break;
	case VIDIOC_ENUMINPUT:
		ret = vidioc_enuminput(file, static_cast<struct v4l2_input *>(arg));
		break;
	case VIDIOC_G_INPUT:
		ret = vidioc_g_input(file, static_cast<int *>(arg));
		break;
	case VIDIOC_S_INPUT:
		ret = vidioc_s_input(file, static_cast<int *>(arg));
		break;
	case VIDIOC_REQBUFS:
		ret = vidioc_reqbufs(file, static_cast<struct v4l2_requestbuffers *>(arg));
		break;
	case VIDIOC_QUERYBUF:
		ret = vidioc_querybuf(file, static_cast<struct v4l2_buffer *>(arg));
		break;
	case VIDIOC_QBUF:
		ret = vidioc_qbuf(file, static_cast<struct v4l2_buffer *>(arg));
		break;
	case VIDIOC_DQBUF:
		ret = vidioc_dqbuf(file, static_cast<struct v4l2_buffer *>(arg), &proxyMutex_);
		break;
	case VIDIOC_EXPBUF:
		ret = vidioc_expbuf(file, static_cast<struct v4l2_exportbuffer *>(arg));
		break;
	case VIDIOC_STREAMON:
		ret = vidioc_streamon(file, static_cast<int *>(arg));
		break;
	case VIDIOC_STREAMOFF:
		ret = vidioc_streamoff(file, static_cast<int *>(arg));
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	return ret;
}

bool V4L2CameraProxy::hasOwnership(V4L2CameraFile *file)
{
	return owner_ == file;
}

/**
 * \brief Acquire exclusive ownership of the V4L2Camera
 *
 * \return Zero on success or if already acquired, and negative error on
 * failure.
 *
 * This is sufficient for poll()ing for buffers. Events, however, are signaled
 * on the file level, so all fds must be signaled. poll()ing from a different
 * fd than the one that locks the device is a corner case, and is currently not
 * supported.
 */
int V4L2CameraProxy::acquire(V4L2CameraFile *file)
{
	if (owner_ == file)
		return 0;

	if (owner_)
		return -EBUSY;

	vcam_->bind(file->efd());

	owner_ = file;

	return 0;
}

void V4L2CameraProxy::release(V4L2CameraFile *file)
{
	if (owner_ != file)
		return;

	vcam_->unbind();

	owner_ = nullptr;
}
