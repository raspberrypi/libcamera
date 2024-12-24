/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright 2022 NXP
 *
 * V4l2 M2M Format converter interface
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
class Stream;
struct StreamConfiguration;
class Rectangle;
class V4L2M2MDevice;

class V4L2M2MConverter : public Converter
{
public:
	V4L2M2MConverter(MediaDevice *media);

	int loadConfiguration([[maybe_unused]] const std::string &filename) override { return 0; }
	bool isValid() const override { return m2m_ != nullptr; }

	std::vector<PixelFormat> formats(PixelFormat input) override;
	SizeRange sizes(const Size &input) override;

	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &pixelFormat, const Size &size) override;

	Size adjustInputSize(const PixelFormat &pixFmt,
			     const Size &size, Alignment align = Alignment::Down) override;
	Size adjustOutputSize(const PixelFormat &pixFmt,
			      const Size &size, Alignment align = Alignment::Down) override;

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>>
		      &outputCfg) override;
	bool isConfigured(const Stream *stream) const override;
	int exportBuffers(const Stream *stream, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start() override;
	void stop() override;

	int validateOutput(StreamConfiguration *cfg, bool *adjusted,
			   Alignment align = Alignment::Down) override;

	int queueBuffers(FrameBuffer *input,
			 const std::map<const Stream *, FrameBuffer *> &outputs) override;

	int setInputCrop(const Stream *stream, Rectangle *rect) override;
	std::pair<Rectangle, Rectangle> inputCropBounds() override { return inputCropBounds_; }
	std::pair<Rectangle, Rectangle> inputCropBounds(const Stream *stream) override;

private:
	class V4L2M2MStream : protected Loggable
	{
	public:
		V4L2M2MStream(V4L2M2MConverter *converter, const Stream *stream);

		bool isValid() const { return m2m_ != nullptr; }

		int configure(const StreamConfiguration &inputCfg,
			      const StreamConfiguration &outputCfg);
		int exportBuffers(unsigned int count,
				  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

		int start();
		void stop();

		int queueBuffers(FrameBuffer *input, FrameBuffer *output);

		int setInputSelection(unsigned int target, Rectangle *rect);
		int getInputSelection(unsigned int target, Rectangle *rect);

		std::pair<Rectangle, Rectangle> inputCropBounds();

	protected:
		std::string logPrefix() const override;

	private:
		void captureBufferReady(FrameBuffer *buffer);
		void outputBufferReady(FrameBuffer *buffer);

		V4L2M2MConverter *converter_;
		const Stream *stream_;
		std::unique_ptr<V4L2M2MDevice> m2m_;

		unsigned int inputBufferCount_;
		unsigned int outputBufferCount_;

		std::pair<Rectangle, Rectangle> inputCropBounds_;
	};

	Size adjustSizes(const Size &size, const std::vector<SizeRange> &ranges,
			 Alignment align);

	std::unique_ptr<V4L2M2MDevice> m2m_;

	std::map<const Stream *, std::unique_ptr<V4L2M2MStream>> streams_;
	std::map<FrameBuffer *, unsigned int> queue_;
	std::pair<Rectangle, Rectangle> inputCropBounds_;
};

} /* namespace libcamera */
