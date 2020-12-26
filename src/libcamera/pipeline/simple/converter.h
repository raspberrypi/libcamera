/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 *
 * converter.h - Format converter for simple pipeline handler
 */

#ifndef __LIBCAMERA_PIPELINE_SIMPLE_CONVERTER_H__
#define __LIBCAMERA_PIPELINE_SIMPLE_CONVERTER_H__

#include <memory>
#include <queue>
#include <tuple>
#include <vector>

#include <libcamera/pixel_format.h>
#include <libcamera/signal.h>

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class Size;
class SizeRange;
struct StreamConfiguration;
class V4L2M2MDevice;

class SimpleConverter
{
public:
	SimpleConverter(MediaDevice *media);

	int open();
	void close();

	std::vector<PixelFormat> formats(PixelFormat input);
	SizeRange sizes(const Size &input);

	int configure(PixelFormat inputFormat, const Size &inputSize,
		      const StreamConfiguration &outputCfg);
	int exportBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	int start(unsigned int count);
	void stop();

	int queueBuffers(FrameBuffer *input, FrameBuffer *output);

	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const Size &size, const PixelFormat &pixelFormat);

	Signal<FrameBuffer *, FrameBuffer *> bufferReady;

private:
	void captureBufferReady(FrameBuffer *buffer);
	void outputBufferReady(FrameBuffer *buffer);

	std::unique_ptr<V4L2M2MDevice> m2m_;

	std::queue<FrameBuffer *> captureDoneQueue_;
	std::queue<FrameBuffer *> outputDoneQueue_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_SIMPLE_CONVERTER_H__ */
