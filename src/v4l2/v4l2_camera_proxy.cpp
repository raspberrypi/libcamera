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
#include <linux/videodev2.h>
#include <string.h>
#include <sys/mman.h>

#include <libcamera/camera.h>
#include <libcamera/object.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/utils.h"

#include "v4l2_camera.h"
#include "v4l2_compat_manager.h"

#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))

using namespace libcamera;

LOG_DECLARE_CATEGORY(V4L2Compat);

V4L2CameraProxy::V4L2CameraProxy(unsigned int index,
				 std::shared_ptr<Camera> camera)
	: refcount_(0), index_(index), bufferCount_(0), currentBuf_(0),
	  vcam_(std::make_unique<V4L2Camera>(camera))
{
	querycap(camera);
}

int V4L2CameraProxy::open(bool nonBlocking)
{
	LOG(V4L2Compat, Debug) << "Servicing open";

	int ret = vcam_->open();
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	nonBlocking_ = nonBlocking;

	vcam_->getStreamConfig(&streamConfig_);
	setFmtFromConfig(streamConfig_);
	sizeimage_ = calculateSizeImage(streamConfig_);

	refcount_++;

	return 0;
}

void V4L2CameraProxy::dup()
{
	refcount_++;
}

void V4L2CameraProxy::close()
{
	LOG(V4L2Compat, Debug) << "Servicing close";

	if (--refcount_ > 0)
		return;

	vcam_->close();
}

void *V4L2CameraProxy::mmap(void *addr, size_t length, int prot, int flags,
			    off_t offset)
{
	LOG(V4L2Compat, Debug) << "Servicing mmap";

	/* \todo Validate prot and flags properly. */
	if (prot != (PROT_READ | PROT_WRITE)) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	unsigned int index = offset / sizeimage_;
	if (static_cast<off_t>(index * sizeimage_) != offset ||
	    length != sizeimage_) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	FileDescriptor fd = vcam_->getBufferFd(index);
	if (!fd.isValid()) {
		errno = EINVAL;
		return MAP_FAILED;
	}

	void *map = V4L2CompatManager::instance()->fops().mmap(addr, length, prot,
							       flags, fd.fd(), 0);
	if (map == MAP_FAILED)
		return map;

	buffers_[index].flags |= V4L2_BUF_FLAG_MAPPED;
	mmaps_[map] = index;

	return map;
}

int V4L2CameraProxy::munmap(void *addr, size_t length)
{
	LOG(V4L2Compat, Debug) << "Servicing munmap";

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

void V4L2CameraProxy::setFmtFromConfig(StreamConfiguration &streamConfig)
{
	curV4L2Format_.fmt.pix.width = streamConfig.size.width;
	curV4L2Format_.fmt.pix.height = streamConfig.size.height;
	curV4L2Format_.fmt.pix.pixelformat = drmToV4L2(streamConfig.pixelFormat);
	curV4L2Format_.fmt.pix.field = V4L2_FIELD_NONE;
	curV4L2Format_.fmt.pix.bytesperline =
		bplMultiplier(curV4L2Format_.fmt.pix.pixelformat) *
		curV4L2Format_.fmt.pix.width;
	curV4L2Format_.fmt.pix.sizeimage =
		imageSize(curV4L2Format_.fmt.pix.pixelformat,
			  curV4L2Format_.fmt.pix.width,
			  curV4L2Format_.fmt.pix.height);
	curV4L2Format_.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
}

unsigned int V4L2CameraProxy::calculateSizeImage(StreamConfiguration &streamConfig)
{
	/*
	 * \todo Merge this method with setFmtFromConfig (need imageSize to
	 * support all libcamera formats first, or filter out MJPEG for now).
	 */
	return imageSize(drmToV4L2(streamConfig.pixelFormat),
			 streamConfig.size.width,
			 streamConfig.size.height);
}

void V4L2CameraProxy::querycap(std::shared_ptr<Camera> camera)
{
	std::string driver = "libcamera";
	std::string bus_info = driver + ":" + std::to_string(index_);

	utils::strlcpy(reinterpret_cast<char *>(capabilities_.driver), driver.c_str(),
		       sizeof(capabilities_.driver));
	utils::strlcpy(reinterpret_cast<char *>(capabilities_.card), camera->name().c_str(),
		       sizeof(capabilities_.card));
	utils::strlcpy(reinterpret_cast<char *>(capabilities_.bus_info), bus_info.c_str(),
		       sizeof(capabilities_.bus_info));
	/* \todo Put this in a header/config somewhere. */
	capabilities_.version = KERNEL_VERSION(5, 2, 0);
	capabilities_.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	capabilities_.capabilities = capabilities_.device_caps
				   | V4L2_CAP_DEVICE_CAPS;
	memset(capabilities_.reserved, 0, sizeof(capabilities_.reserved));
}

void V4L2CameraProxy::updateBuffers()
{
	std::vector<V4L2Camera::Buffer> completedBuffers = vcam_->completedBuffers();
	for (const V4L2Camera::Buffer &buffer : completedBuffers) {
		const FrameMetadata &fmd = buffer.data;
		struct v4l2_buffer &buf = buffers_[buffer.index];

		switch (fmd.status) {
		case FrameMetadata::FrameSuccess:
			buf.bytesused = fmd.planes[0].bytesused;
			buf.field = V4L2_FIELD_NONE;
			buf.timestamp.tv_sec = fmd.timestamp / 1000000000;
			buf.timestamp.tv_usec = fmd.timestamp % 1000000;
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

int V4L2CameraProxy::vidioc_querycap(struct v4l2_capability *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_querycap";

	*arg = capabilities_;

	return 0;
}

int V4L2CameraProxy::vidioc_enum_fmt(struct v4l2_fmtdesc *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_enum_fmt";

	if (!validateBufferType(arg->type) ||
	    arg->index >= streamConfig_.formats().pixelformats().size())
		return -EINVAL;

	/* \todo Add map from format to description. */
	utils::strlcpy(reinterpret_cast<char *>(arg->description), "Video Format Description",
		       sizeof(arg->description));
	arg->pixelformat = drmToV4L2(streamConfig_.formats().pixelformats()[arg->index]);

	return 0;
}

int V4L2CameraProxy::vidioc_g_fmt(struct v4l2_format *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_g_fmt";

	if (!validateBufferType(arg->type))
		return -EINVAL;

	memset(&arg->fmt, 0, sizeof(arg->fmt));
	arg->fmt.pix = curV4L2Format_.fmt.pix;

	return 0;
}

void V4L2CameraProxy::tryFormat(struct v4l2_format *arg)
{
	PixelFormat format = v4l2ToDrm(arg->fmt.pix.pixelformat);
	const std::vector<PixelFormat> &formats =
		streamConfig_.formats().pixelformats();
	if (std::find(formats.begin(), formats.end(), format) == formats.end())
		format = streamConfig_.formats().pixelformats()[0];

	Size size(arg->fmt.pix.width, arg->fmt.pix.height);
	const std::vector<Size> &sizes = streamConfig_.formats().sizes(format);
	if (std::find(sizes.begin(), sizes.end(), size) == sizes.end())
		size = streamConfig_.formats().sizes(format)[0];

	arg->fmt.pix.width        = size.width;
	arg->fmt.pix.height       = size.height;
	arg->fmt.pix.pixelformat  = drmToV4L2(format);
	arg->fmt.pix.field        = V4L2_FIELD_NONE;
	arg->fmt.pix.bytesperline = bplMultiplier(drmToV4L2(format)) *
				    arg->fmt.pix.width;
	arg->fmt.pix.sizeimage    = imageSize(drmToV4L2(format),
					      arg->fmt.pix.width,
					      arg->fmt.pix.height);
	arg->fmt.pix.colorspace   = V4L2_COLORSPACE_SRGB;
}

int V4L2CameraProxy::vidioc_s_fmt(struct v4l2_format *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_s_fmt";

	if (!validateBufferType(arg->type))
		return -EINVAL;

	tryFormat(arg);

	Size size(arg->fmt.pix.width, arg->fmt.pix.height);
	int ret = vcam_->configure(&streamConfig_, size,
				   v4l2ToDrm(arg->fmt.pix.pixelformat),
				   bufferCount_);
	if (ret < 0)
		return -EINVAL;

	unsigned int sizeimage = calculateSizeImage(streamConfig_);
	if (sizeimage == 0)
		return -EINVAL;

	sizeimage_ = sizeimage;

	setFmtFromConfig(streamConfig_);

	return 0;
}

int V4L2CameraProxy::vidioc_try_fmt(struct v4l2_format *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_try_fmt";

	if (!validateBufferType(arg->type))
		return -EINVAL;

	tryFormat(arg);

	return 0;
}

int V4L2CameraProxy::freeBuffers()
{
	LOG(V4L2Compat, Debug) << "Freeing libcamera bufs";

	int ret = vcam_->streamOff();
	if (ret < 0) {
		LOG(V4L2Compat, Error) << "Failed to stop stream";
		return ret;
	}
	vcam_->freeBuffers();
	bufferCount_ = 0;

	return 0;
}

int V4L2CameraProxy::vidioc_reqbufs(struct v4l2_requestbuffers *arg)
{
	int ret;

	LOG(V4L2Compat, Debug) << "Servicing vidioc_reqbufs";

	if (!validateBufferType(arg->type) ||
	    !validateMemoryType(arg->memory))
		return -EINVAL;

	LOG(V4L2Compat, Debug) << arg->count << " buffers requested ";

	arg->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP;

	if (arg->count == 0)
		return freeBuffers();

	Size size(curV4L2Format_.fmt.pix.width, curV4L2Format_.fmt.pix.height);
	ret = vcam_->configure(&streamConfig_, size,
			       v4l2ToDrm(curV4L2Format_.fmt.pix.pixelformat),
			       arg->count);
	if (ret < 0)
		return -EINVAL;

	sizeimage_ = calculateSizeImage(streamConfig_);
	/*
	 * If we return -EINVAL here then the application will think that we
	 * don't support streaming mmap. Since we don't support readwrite and
	 * userptr either, the application will get confused and think that
	 * we don't support anything.
	 * On the other hand, if a format has a zero sizeimage (eg. MJPEG),
	 * we'll get a floating point exception when we try to stream it.
	 */
	if (sizeimage_ == 0)
		LOG(V4L2Compat, Warning)
			<< "sizeimage of at least one format is zero. "
			<< "Streaming this format will cause a floating point exception.";

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
		buf.length = curV4L2Format_.fmt.pix.sizeimage;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.m.offset = i * curV4L2Format_.fmt.pix.sizeimage;
		buf.index = i;

		buffers_[i] = buf;
	}

	LOG(V4L2Compat, Debug) << "Allocated " << arg->count << " buffers";

	return 0;
}

int V4L2CameraProxy::vidioc_querybuf(struct v4l2_buffer *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_querybuf";

	if (!validateBufferType(arg->type) ||
	    arg->index >= bufferCount_)
		return -EINVAL;

	updateBuffers();

	*arg = buffers_[arg->index];

	return 0;
}

int V4L2CameraProxy::vidioc_qbuf(struct v4l2_buffer *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_qbuf, index = "
			       << arg->index;

	if (!validateBufferType(arg->type) ||
	    !validateMemoryType(arg->memory) ||
	    arg->index >= bufferCount_)
		return -EINVAL;

	int ret = vcam_->qbuf(arg->index);
	if (ret < 0)
		return ret;

	arg->flags |= V4L2_BUF_FLAG_QUEUED;
	arg->flags &= ~V4L2_BUF_FLAG_DONE;

	return ret;
}

int V4L2CameraProxy::vidioc_dqbuf(struct v4l2_buffer *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_dqbuf";

	if (!validateBufferType(arg->type) ||
	    !validateMemoryType(arg->memory))
		return -EINVAL;

	if (!nonBlocking_)
		vcam_->bufferSema_.acquire();
	else if (!vcam_->bufferSema_.tryAcquire())
		return -EAGAIN;

	updateBuffers();

	struct v4l2_buffer &buf = buffers_[currentBuf_];

	buf.flags &= ~V4L2_BUF_FLAG_QUEUED;
	buf.length = sizeimage_;
	*arg = buf;

	currentBuf_ = (currentBuf_ + 1) % bufferCount_;

	return 0;
}

int V4L2CameraProxy::vidioc_streamon(int *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_streamon";

	if (!validateBufferType(*arg))
		return -EINVAL;

	return vcam_->streamOn();
}

int V4L2CameraProxy::vidioc_streamoff(int *arg)
{
	LOG(V4L2Compat, Debug) << "Servicing vidioc_streamoff";

	if (!validateBufferType(*arg))
		return -EINVAL;

	int ret = vcam_->streamOff();

	for (struct v4l2_buffer &buf : buffers_)
		buf.flags &= ~(V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE);

	return ret;
}

int V4L2CameraProxy::ioctl(unsigned long request, void *arg)
{
	int ret;
	switch (request) {
	case VIDIOC_QUERYCAP:
		ret = vidioc_querycap(static_cast<struct v4l2_capability *>(arg));
		break;
	case VIDIOC_ENUM_FMT:
		ret = vidioc_enum_fmt(static_cast<struct v4l2_fmtdesc *>(arg));
		break;
	case VIDIOC_G_FMT:
		ret = vidioc_g_fmt(static_cast<struct v4l2_format *>(arg));
		break;
	case VIDIOC_S_FMT:
		ret = vidioc_s_fmt(static_cast<struct v4l2_format *>(arg));
		break;
	case VIDIOC_TRY_FMT:
		ret = vidioc_try_fmt(static_cast<struct v4l2_format *>(arg));
		break;
	case VIDIOC_REQBUFS:
		ret = vidioc_reqbufs(static_cast<struct v4l2_requestbuffers *>(arg));
		break;
	case VIDIOC_QUERYBUF:
		ret = vidioc_querybuf(static_cast<struct v4l2_buffer *>(arg));
		break;
	case VIDIOC_QBUF:
		ret = vidioc_qbuf(static_cast<struct v4l2_buffer *>(arg));
		break;
	case VIDIOC_DQBUF:
		ret = vidioc_dqbuf(static_cast<struct v4l2_buffer *>(arg));
		break;
	case VIDIOC_STREAMON:
		ret = vidioc_streamon(static_cast<int *>(arg));
		break;
	case VIDIOC_STREAMOFF:
		ret = vidioc_streamoff(static_cast<int *>(arg));
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

struct PixelFormatPlaneInfo {
	unsigned int bitsPerPixel;
	unsigned int hSubSampling;
	unsigned int vSubSampling;
};

struct PixelFormatInfo {
	PixelFormat format;
	uint32_t v4l2Format;
	unsigned int numPlanes;
	std::array<PixelFormatPlaneInfo, 3> planes;
};

namespace {

static const std::array<PixelFormatInfo, 13> pixelFormatInfo = {{
	/* RGB formats. */
	{ PixelFormat(DRM_FORMAT_RGB888),	V4L2_PIX_FMT_BGR24,	1, {{ { 24, 1, 1 }, {  0, 0, 0 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_BGR888),	V4L2_PIX_FMT_RGB24,	1, {{ { 24, 1, 1 }, {  0, 0, 0 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_BGRA8888),	V4L2_PIX_FMT_ARGB32,	1, {{ { 32, 1, 1 }, {  0, 0, 0 }, {  0, 0, 0 } }} },
	/* YUV packed formats. */
	{ PixelFormat(DRM_FORMAT_UYVY),		V4L2_PIX_FMT_UYVY,	1, {{ { 16, 1, 1 }, {  0, 0, 0 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_VYUY),		V4L2_PIX_FMT_VYUY,	1, {{ { 16, 1, 1 }, {  0, 0, 0 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_YUYV),		V4L2_PIX_FMT_YUYV,	1, {{ { 16, 1, 1 }, {  0, 0, 0 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_YVYU),		V4L2_PIX_FMT_YVYU,	1, {{ { 16, 1, 1 }, {  0, 0, 0 }, {  0, 0, 0 } }} },
	/* YUY planar formats. */
	{ PixelFormat(DRM_FORMAT_NV12),		V4L2_PIX_FMT_NV12,	2, {{ {  8, 1, 1 }, { 16, 2, 2 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_NV21),		V4L2_PIX_FMT_NV21,	2, {{ {  8, 1, 1 }, { 16, 2, 2 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_NV16),		V4L2_PIX_FMT_NV16,	2, {{ {  8, 1, 1 }, { 16, 2, 1 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_NV61),		V4L2_PIX_FMT_NV61,	2, {{ {  8, 1, 1 }, { 16, 2, 1 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_NV24),		V4L2_PIX_FMT_NV24,	2, {{ {  8, 1, 1 }, { 16, 2, 1 }, {  0, 0, 0 } }} },
	{ PixelFormat(DRM_FORMAT_NV42),		V4L2_PIX_FMT_NV42,	2, {{ {  8, 1, 1 }, { 16, 1, 1 }, {  0, 0, 0 } }} },
}};

} /* namespace */

/* \todo make libcamera export these */
unsigned int V4L2CameraProxy::bplMultiplier(uint32_t format)
{
	auto info = std::find_if(pixelFormatInfo.begin(), pixelFormatInfo.end(),
				 [format](const PixelFormatInfo &info) {
					 return info.v4l2Format == format;
				 });
	if (info == pixelFormatInfo.end())
		return 0;

	return info->planes[0].bitsPerPixel / 8;
}

unsigned int V4L2CameraProxy::imageSize(uint32_t format, unsigned int width,
					unsigned int height)
{
	auto info = std::find_if(pixelFormatInfo.begin(), pixelFormatInfo.end(),
				 [format](const PixelFormatInfo &info) {
					 return info.v4l2Format == format;
				 });
	if (info == pixelFormatInfo.end())
		return 0;

	unsigned int multiplier = 0;
	for (unsigned int i = 0; i < info->numPlanes; ++i)
		multiplier += info->planes[i].bitsPerPixel
			    / info->planes[i].hSubSampling
			    / info->planes[i].vSubSampling;

	return width * height * multiplier / 8;
}

PixelFormat V4L2CameraProxy::v4l2ToDrm(uint32_t format)
{
	auto info = std::find_if(pixelFormatInfo.begin(), pixelFormatInfo.end(),
				 [format](const PixelFormatInfo &info) {
					 return info.v4l2Format == format;
				 });
	if (info == pixelFormatInfo.end())
		return PixelFormat();

	return info->format;
}

uint32_t V4L2CameraProxy::drmToV4L2(const PixelFormat &format)
{
	auto info = std::find_if(pixelFormatInfo.begin(), pixelFormatInfo.end(),
				 [format](const PixelFormatInfo &info) {
					 return info.format == format;
				 });
	if (info == pixelFormatInfo.end())
		return format;

	return info->v4l2Format;
}
