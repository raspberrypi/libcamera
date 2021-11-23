/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 *
 * converter.h - Format converter for simple pipeline handler
 */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/pixel_format.h>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>

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

	bool isValid() const { return m2m_ != nullptr; }

	std::vector<PixelFormat> formats(PixelFormat input);
	SizeRange sizes(const Size &input);

	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &pixelFormat, const Size &size);

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfg);
	int exportBuffers(unsigned int ouput, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	int start();
	void stop();

	int queueBuffers(FrameBuffer *input,
			 const std::map<unsigned int, FrameBuffer *> &outputs);

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

private:
	class Stream : protected Loggable
	{
	public:
		Stream(SimpleConverter *converter, unsigned int index);

		bool isValid() const { return m2m_ != nullptr; }

		int configure(const StreamConfiguration &inputCfg,
			      const StreamConfiguration &outputCfg);
		int exportBuffers(unsigned int count,
				  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

		int start();
		void stop();

		int queueBuffers(FrameBuffer *input, FrameBuffer *output);

	protected:
		std::string logPrefix() const override;

	private:
		void captureBufferReady(FrameBuffer *buffer);
		void outputBufferReady(FrameBuffer *buffer);

		SimpleConverter *converter_;
		unsigned int index_;
		std::unique_ptr<V4L2M2MDevice> m2m_;

		unsigned int inputBufferCount_;
		unsigned int outputBufferCount_;
	};

	std::string deviceNode_;
	std::unique_ptr<V4L2M2MDevice> m2m_;

	std::vector<Stream> streams_;
	std::map<FrameBuffer *, unsigned int> queue_;
};

} /* namespace libcamera */
