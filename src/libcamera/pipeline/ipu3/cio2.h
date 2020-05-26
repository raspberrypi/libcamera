/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * cio2.h - Intel IPU3 CIO2
 */
#ifndef __LIBCAMERA_PIPELINE_IPU3_CIO2_H__
#define __LIBCAMERA_PIPELINE_IPU3_CIO2_H__

#include <memory>
#include <queue>
#include <vector>

namespace libcamera {

class CameraSensor;
class FrameBuffer;
class MediaDevice;
class V4L2DeviceFormat;
class V4L2Subdevice;
class V4L2VideoDevice;
struct Size;

class CIO2Device
{
public:
	static constexpr unsigned int CIO2_BUFFER_COUNT = 4;

	CIO2Device();
	~CIO2Device();

	int init(const MediaDevice *media, unsigned int index);
	int configure(const Size &size, V4L2DeviceFormat *outputFormat);

	int allocateBuffers();
	void freeBuffers();

	FrameBuffer *getBuffer();
	void putBuffer(FrameBuffer *buffer);

	int start();
	int stop();

	V4L2VideoDevice *output_;
	V4L2Subdevice *csi2_;
	CameraSensor *sensor_;

private:
	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
	std::queue<FrameBuffer *> availableBuffers_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_IPU3_CIO2_H__ */
