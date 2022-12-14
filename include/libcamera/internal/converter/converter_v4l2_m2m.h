/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright 2022 NXP
 *
 * converter_v4l2_m2m.h - V4l2 M2M Format converter interface
 */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>

#include <libcamera/pixel_format.h>

#include "libcamera/internal/converter.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class Size;
class SizeRange;
struct StreamConfiguration;
class V4L2M2MDevice;

class V4L2M2MConverter : public Converter
{
public:
	V4L2M2MConverter(MediaDevice *media);

	int loadConfiguration([[maybe_unused]] const std::string &filename) { return 0; }
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

private:
	class Stream : protected Loggable
	{
	public:
		Stream(V4L2M2MConverter *converter, unsigned int index);

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

		V4L2M2MConverter *converter_;
		unsigned int index_;
		std::unique_ptr<V4L2M2MDevice> m2m_;

		unsigned int inputBufferCount_;
		unsigned int outputBufferCount_;
	};

	std::unique_ptr<V4L2M2MDevice> m2m_;

	std::vector<Stream> streams_;
	std::map<FrameBuffer *, unsigned int> queue_;
};

} /* namespace libcamera */
