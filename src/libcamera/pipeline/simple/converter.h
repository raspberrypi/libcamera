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
#include <vector>

#include <libcamera/pixelformats.h>
#include <libcamera/signal.h>

namespace libcamera {

class FrameBuffer;
class MediaDevice;
struct Size;
class V4L2M2MDevice;

class SimpleConverter
{
public:
	SimpleConverter(MediaDevice *media);
	~SimpleConverter();

	int open();
	void close();

	std::vector<PixelFormat> formats(PixelFormat input);

	int configure(PixelFormat inputFormat, PixelFormat outputFormat,
		      const Size &size);
	int exportBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	int start(unsigned int count);
	void stop();

	int queueBuffers(FrameBuffer *input, FrameBuffer *output);

	Signal<FrameBuffer *, FrameBuffer *> bufferReady;

private:
	void captureBufferReady(FrameBuffer *buffer);
	void outputBufferReady(FrameBuffer *buffer);

	V4L2M2MDevice *m2m_;

	std::queue<FrameBuffer *> captureDoneQueue_;
	std::queue<FrameBuffer *> outputDoneQueue_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_SIMPLE_CONVERTER_H__ */
