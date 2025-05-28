/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Intel IPU3 CIO2
 */

#pragma once

#include <memory>
#include <queue>
#include <vector>

#include <libcamera/base/signal.h>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CameraSensor;
class FrameBuffer;
class MediaDevice;
class PixelFormat;
class Request;
class Size;
class SizeRange;
struct StreamConfiguration;
enum class Transform;

class CIO2Device
{
public:
	static constexpr unsigned int kBufferCount = 4;

	CIO2Device();

	std::vector<PixelFormat> formats() const;
	std::vector<SizeRange> sizes(const PixelFormat &format) const;

	int init(const MediaDevice *media, unsigned int index);
	int configure(const Size &size, const Transform &transform,
		      V4L2DeviceFormat *outputFormat);

	StreamConfiguration generateConfiguration(Size size) const;

	int exportBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	V4L2SubdeviceFormat getSensorFormat(const std::vector<unsigned int> &mbusCodes,
					    const Size &size) const;

	int start();
	int stop();

	CameraSensor *sensor() { return sensor_.get(); }
	const CameraSensor *sensor() const { return sensor_.get(); }

	FrameBuffer *queueBuffer(Request *request, FrameBuffer *rawBuffer);
	void tryReturnBuffer(FrameBuffer *buffer);
	Signal<FrameBuffer *> &bufferReady() { return output_->bufferReady; }
	Signal<uint32_t> &frameStart() { return csi2_->frameStart; }

	Signal<> bufferAvailable;

private:
	void freeBuffers();

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<V4L2Subdevice> csi2_;
	std::unique_ptr<V4L2VideoDevice> output_;

	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
	std::queue<FrameBuffer *> availableBuffers_;
};

} /* namespace libcamera */
