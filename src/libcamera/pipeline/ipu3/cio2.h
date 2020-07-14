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

#include <libcamera/signal.h>

#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CameraSensor;
class FrameBuffer;
class MediaDevice;
class PixelFormat;
class Request;
class Size;
class SizeRange;
class V4L2Subdevice;
struct StreamConfiguration;

class CIO2Device
{
public:
	static constexpr unsigned int CIO2_BUFFER_COUNT = 4;

	CIO2Device();
	~CIO2Device();

	std::vector<PixelFormat> formats() const;
	std::vector<SizeRange> sizes() const;

	int init(const MediaDevice *media, unsigned int index);
	int configure(const Size &size, V4L2DeviceFormat *outputFormat);

	StreamConfiguration generateConfiguration(Size size) const;

	int exportBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	int start();
	int stop();

	CameraSensor *sensor() { return sensor_; }
	const CameraSensor *sensor() const { return sensor_; }

	int queueBuffer(Request *request, FrameBuffer *rawBuffer);
	void tryReturnBuffer(FrameBuffer *buffer);
	Signal<FrameBuffer *> &bufferReady() { return output_->bufferReady; }

private:
	void freeBuffers();

	void cio2BufferReady(FrameBuffer *buffer);

	CameraSensor *sensor_;
	V4L2Subdevice *csi2_;
	V4L2VideoDevice *output_;

	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
	std::queue<FrameBuffer *> availableBuffers_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_IPU3_CIO2_H__ */
