/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * debayering base class
 */

#pragma once

#include <stdint.h>

#include <libcamera/base/log.h>
#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/global_configuration.h"
#include "libcamera/internal/software_isp/benchmark.h"
#include "libcamera/internal/software_isp/debayer_params.h"

namespace libcamera {

class FrameBuffer;

LOG_DECLARE_CATEGORY(Debayer)

class Debayer : public Object
{
public:
	Debayer(const GlobalConfiguration &configuration);
	virtual ~Debayer() = 0;

	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
			      bool ccmEnabled) = 0;

	virtual std::vector<PixelFormat> formats(PixelFormat inputFormat) = 0;

	virtual std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &outputFormat, const Size &size) = 0;

	virtual void process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params) = 0;
	virtual int start() { return 0; }
	virtual void stop() {}

	virtual SizeRange sizes(PixelFormat inputFormat, const Size &inputSize) = 0;

	virtual const SharedFD &getStatsFD() = 0;

	unsigned int frameSize() { return outputConfig_.frameSize; }

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

	struct DebayerInputConfig {
		Size patternSize;
		unsigned int bpp;
		unsigned int stride;
		std::vector<PixelFormat> outputFormats;
	};

	struct DebayerOutputConfig {
		unsigned int bpp;
		unsigned int stride;
		unsigned int frameSize;
	};

	DebayerInputConfig inputConfig_;
	DebayerOutputConfig outputConfig_;
	Size outputSize_;
	PixelFormat inputPixelFormat_;
	PixelFormat outputPixelFormat_;
	bool swapRedBlueGains_;
	Benchmark bench_;

private:
	virtual Size patternSize(PixelFormat inputFormat) = 0;

protected:
	void dmaSyncBegin(std::vector<DmaSyncer> &dmaSyncers, FrameBuffer *input, FrameBuffer *output);
	static bool isStandardBayerOrder(BayerFormat::Order order);
};

} /* namespace libcamera */
