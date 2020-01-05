/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_camera_proxy.h - Proxy to V4L2 compatibility camera
 */

#ifndef __V4L2_CAMERA_PROXY_H__
#define __V4L2_CAMERA_PROXY_H__

#include <linux/videodev2.h>
#include <map>
#include <memory>
#include <sys/types.h>
#include <vector>

#include <libcamera/camera.h>

#include "v4l2_camera.h"

using namespace libcamera;

class V4L2CameraProxy
{
public:
	V4L2CameraProxy(unsigned int index, std::shared_ptr<Camera> camera);

	int open(bool nonBlocking);
	void dup();
	void close();
	void *mmap(size_t length, int prot, int flags, off_t offset);
	int munmap(void *addr, size_t length);

	int ioctl(unsigned long request, void *arg);

private:
	bool validateBufferType(uint32_t type);
	bool validateMemoryType(uint32_t memory);
	void setFmtFromConfig(StreamConfiguration &streamConfig);
	unsigned int calculateSizeImage(StreamConfiguration &streamConfig);
	void querycap(std::shared_ptr<Camera> camera);
	void updateBuffers();
	int freeBuffers();

	int vidioc_querycap(struct v4l2_capability *arg);
	int vidioc_enum_fmt(struct v4l2_fmtdesc *arg);
	int vidioc_g_fmt(struct v4l2_format *arg);
	int vidioc_s_fmt(struct v4l2_format *arg);
	int vidioc_try_fmt(struct v4l2_format *arg);
	int vidioc_reqbufs(struct v4l2_requestbuffers *arg);
	int vidioc_querybuf(struct v4l2_buffer *arg);
	int vidioc_qbuf(struct v4l2_buffer *arg);
	int vidioc_dqbuf(struct v4l2_buffer *arg);
	int vidioc_streamon(int *arg);
	int vidioc_streamoff(int *arg);

	static unsigned int bplMultiplier(uint32_t format);
	static unsigned int imageSize(uint32_t format, unsigned int width,
				      unsigned int height);

	static PixelFormat v4l2ToDrm(uint32_t format);
	static uint32_t drmToV4L2(PixelFormat format);

	unsigned int refcount_;
	unsigned int index_;
	bool nonBlocking_;

	struct v4l2_format curV4L2Format_;
	StreamConfiguration streamConfig_;
	struct v4l2_capability capabilities_;
	unsigned int bufferCount_;
	unsigned int currentBuf_;
	unsigned int sizeimage_;

	std::vector<struct v4l2_buffer> buffers_;
	std::map<void *, unsigned int> mmaps_;

	std::unique_ptr<V4L2Camera> vcam_;
};

#endif /* __V4L2_CAMERA_PROXY_H__ */
