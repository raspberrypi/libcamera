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
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/software_isp/debayer_params.h"

namespace libcamera {

class FrameBuffer;

LOG_DECLARE_CATEGORY(Debayer)

class Debayer
{
public:
	virtual ~Debayer() = 0;

	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
			      bool ccmEnabled) = 0;

	virtual std::vector<PixelFormat> formats(PixelFormat inputFormat) = 0;

	virtual std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &outputFormat, const Size &size) = 0;

	virtual void process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params) = 0;

	virtual SizeRange sizes(PixelFormat inputFormat, const Size &inputSize) = 0;

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

private:
	virtual Size patternSize(PixelFormat inputFormat) = 0;
};

} /* namespace libcamera */
