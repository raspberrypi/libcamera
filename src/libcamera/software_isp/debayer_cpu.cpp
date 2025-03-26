/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023-2025 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * CPU based debayering class
 */

#include "debayer_cpu.h"

#include <algorithm>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>
#include <utility>

#include <linux/dma-buf.h>

#include <libcamera/formats.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

/**
 * \class DebayerCpu
 * \brief Class for debayering on the CPU
 *
 * Implementation for CPU based debayering
 */

/**
 * \brief Constructs a DebayerCpu object
 * \param[in] stats Pointer to the stats object to use
 */
DebayerCpu::DebayerCpu(std::unique_ptr<SwStatsCpu> stats)
	: stats_(std::move(stats))
{
	/*
	 * Reading from uncached buffers may be very slow.
	 * In such a case, it's better to copy input buffer data to normal memory.
	 * But in case of cached buffers, copying the data is unnecessary overhead.
	 * enable_input_memcpy_ makes this behavior configurable.  At the moment, we
	 * always set it to true as the safer choice but this should be changed in
	 * future.
	 */
	enableInputMemcpy_ = true;

	/* Initialize color lookup tables */
	for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
		red_[i] = green_[i] = blue_[i] = i;
		redCcm_[i] = { static_cast<int16_t>(i), 0, 0 };
		greenCcm_[i] = { 0, static_cast<int16_t>(i), 0 };
		blueCcm_[i] = { 0, 0, static_cast<int16_t>(i) };
	}
}

DebayerCpu::~DebayerCpu() = default;

#define DECLARE_SRC_POINTERS(pixel_t)                            \
	const pixel_t *prev = (const pixel_t *)src[0] + xShift_; \
	const pixel_t *curr = (const pixel_t *)src[1] + xShift_; \
	const pixel_t *next = (const pixel_t *)src[2] + xShift_;

#define GAMMA(value) \
	*dst++ = gammaLut_[std::clamp(value, 0, static_cast<int>(gammaLut_.size()) - 1)]

#define STORE_PIXEL(b_, g_, r_)                                        \
	if constexpr (ccmEnabled) {                                    \
		const DebayerParams::CcmColumn &blue = blueCcm_[b_];   \
		const DebayerParams::CcmColumn &green = greenCcm_[g_]; \
		const DebayerParams::CcmColumn &red = redCcm_[r_];     \
		GAMMA(blue.b + green.b + red.b);                       \
		GAMMA(blue.g + green.g + red.g);                       \
		GAMMA(blue.r + green.r + red.r);                       \
	} else {                                                       \
		*dst++ = blue_[b_];                                    \
		*dst++ = green_[g_];                                   \
		*dst++ = red_[r_];                                     \
	}                                                              \
	if constexpr (addAlphaByte)                                    \
		*dst++ = 255;                                          \
	x++;

/*
 * RGR
 * GBG
 * RGR
 */
#define BGGR_BGR888(p, n, div)                                                 \
	STORE_PIXEL(                                                           \
		curr[x] / (div),                                               \
		(prev[x] + curr[x - p] + curr[x + n] + next[x]) / (4 * (div)), \
		(prev[x - p] + prev[x + n] + next[x - p] + next[x + n]) / (4 * (div)))

/*
 * GBG
 * RGR
 * GBG
 */
#define GRBG_BGR888(p, n, div)                     \
	STORE_PIXEL(                               \
		(prev[x] + next[x]) / (2 * (div)), \
		curr[x] / (div),                   \
		(curr[x - p] + curr[x + n]) / (2 * (div)))

/*
 * GRG
 * BGB
 * GRG
 */
#define GBRG_BGR888(p, n, div)                             \
	STORE_PIXEL(                                       \
		(curr[x - p] + curr[x + n]) / (2 * (div)), \
		curr[x] / (div),                           \
		(prev[x] + next[x]) / (2 * (div)))

/*
 * BGB
 * GRG
 * BGB
 */
#define RGGB_BGR888(p, n, div)                                                         \
	STORE_PIXEL(                                                                   \
		(prev[x - p] + prev[x + n] + next[x - p] + next[x + n]) / (4 * (div)), \
		(prev[x] + curr[x - p] + curr[x + n] + next[x]) / (4 * (div)),         \
		curr[x] / (div))

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer8_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < (int)window_.width;) {
		BGGR_BGR888(1, 1, 1)
		GBRG_BGR888(1, 1, 1)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer8_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint8_t)

	for (int x = 0; x < (int)window_.width;) {
		GRBG_BGR888(1, 1, 1)
		RGGB_BGR888(1, 1, 1)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 4 for 10 -> 8 bpp value */
		BGGR_BGR888(1, 1, 4)
		GBRG_BGR888(1, 1, 4)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 4 for 10 -> 8 bpp value */
		GRBG_BGR888(1, 1, 4)
		RGGB_BGR888(1, 1, 4)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 16 for 12 -> 8 bpp value */
		BGGR_BGR888(1, 1, 16)
		GBRG_BGR888(1, 1, 16)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer12_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	DECLARE_SRC_POINTERS(uint16_t)

	for (int x = 0; x < (int)window_.width;) {
		/* divide values by 16 for 12 -> 8 bpp value */
		GRBG_BGR888(1, 1, 16)
		RGGB_BGR888(1, 1, 16)
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_BGBG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	/*
	 * For the first pixel getting a pixel from the previous column uses
	 * x - 2 to skip the 5th byte with least-significant bits for 4 pixels.
	 * Same for last pixel (uses x + 2) and looking at the next column.
	 */
	for (int x = 0; x < widthInBytes;) {
		/* First pixel */
		BGGR_BGR888(2, 1, 1)
		/* Second pixel BGGR -> GBRG */
		GBRG_BGR888(1, 1, 1)
		/* Same thing for third and fourth pixels */
		BGGR_BGR888(1, 1, 1)
		GBRG_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_GRGR_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* First pixel */
		GRBG_BGR888(2, 1, 1)
		/* Second pixel GRBG -> RGGB */
		RGGB_BGR888(1, 1, 1)
		/* Same thing for third and fourth pixels */
		GRBG_BGR888(1, 1, 1)
		RGGB_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_GBGB_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		GBRG_BGR888(2, 1, 1)
		/* Odd pixel GBGR -> BGGR */
		BGGR_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		GBRG_BGR888(1, 1, 1)
		BGGR_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

template<bool addAlphaByte, bool ccmEnabled>
void DebayerCpu::debayer10P_RGRG_BGR888(uint8_t *dst, const uint8_t *src[])
{
	const int widthInBytes = window_.width * 5 / 4;
	const uint8_t *prev = src[0];
	const uint8_t *curr = src[1];
	const uint8_t *next = src[2];

	for (int x = 0; x < widthInBytes;) {
		/* Even pixel */
		RGGB_BGR888(2, 1, 1)
		/* Odd pixel RGGB -> GRBG */
		GRBG_BGR888(1, 1, 1)
		/* Same thing for next 2 pixels */
		RGGB_BGR888(1, 1, 1)
		GRBG_BGR888(1, 2, 1)
		/* Skip 5th src byte with 4 x 2 least-significant-bits */
		x++;
	}
}

static bool isStandardBayerOrder(BayerFormat::Order order)
{
	return order == BayerFormat::BGGR || order == BayerFormat::GBRG ||
	       order == BayerFormat::GRBG || order == BayerFormat::RGGB;
}

/*
 * Setup the Debayer object according to the passed in parameters.
 * Return 0 on success, a negative errno value on failure
 * (unsupported parameters).
 */
int DebayerCpu::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10 || bayerFormat.bitDepth == 12) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = (bayerFormat.bitDepth + 7) & ~7;
		config.patternSize.width = 2;
		config.patternSize.height = 2;
		config.outputFormats = std::vector<PixelFormat>({ formats::RGB888,
								  formats::XRGB8888,
								  formats::ARGB8888,
								  formats::BGR888,
								  formats::XBGR8888,
								  formats::ABGR8888 });
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 10;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.patternSize.height = 2;
		config.outputFormats = std::vector<PixelFormat>({ formats::RGB888,
								  formats::XRGB8888,
								  formats::ARGB8888,
								  formats::BGR888,
								  formats::XBGR8888,
								  formats::ABGR8888 });
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported input format " << inputFormat.toString();
	return -EINVAL;
}

int DebayerCpu::getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config)
{
	if (outputFormat == formats::RGB888 || outputFormat == formats::BGR888) {
		config.bpp = 24;
		return 0;
	}

	if (outputFormat == formats::XRGB8888 || outputFormat == formats::ARGB8888 ||
	    outputFormat == formats::XBGR8888 || outputFormat == formats::ABGR8888) {
		config.bpp = 32;
		return 0;
	}

	LOG(Debayer, Info)
		<< "Unsupported output format " << outputFormat.toString();
	return -EINVAL;
}

/*
 * Check for standard Bayer orders and set xShift_ and swap debayer0/1, so that
 * a single pair of BGGR debayer functions can be used for all 4 standard orders.
 */
int DebayerCpu::setupStandardBayerOrder(BayerFormat::Order order)
{
	switch (order) {
	case BayerFormat::BGGR:
		break;
	case BayerFormat::GBRG:
		xShift_ = 1; /* BGGR -> GBRG */
		break;
	case BayerFormat::GRBG:
		std::swap(debayer0_, debayer1_); /* BGGR -> GRBG */
		break;
	case BayerFormat::RGGB:
		xShift_ = 1; /* BGGR -> GBRG */
		std::swap(debayer0_, debayer1_); /* GBRG -> RGGB */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define SET_DEBAYER_METHODS(method0, method1)                                                                        \
	debayer0_ = addAlphaByte                                                                                     \
			    ? (ccmEnabled ? &DebayerCpu::method0<true, true> : &DebayerCpu::method0<true, false>)    \
			    : (ccmEnabled ? &DebayerCpu::method0<false, true> : &DebayerCpu::method0<false, false>); \
	debayer1_ = addAlphaByte                                                                                     \
			    ? (ccmEnabled ? &DebayerCpu::method1<true, true> : &DebayerCpu::method1<true, false>)    \
			    : (ccmEnabled ? &DebayerCpu::method1<false, true> : &DebayerCpu::method1<false, false>);

int DebayerCpu::setDebayerFunctions(PixelFormat inputFormat,
				    PixelFormat outputFormat,
				    bool ccmEnabled)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);
	bool addAlphaByte = false;

	xShift_ = 0;
	swapRedBlueGains_ = false;

	auto invalidFmt = []() -> int {
		LOG(Debayer, Error) << "Unsupported input output format combination";
		return -EINVAL;
	};

	switch (outputFormat) {
	case formats::XRGB8888:
	case formats::ARGB8888:
		addAlphaByte = true;
		[[fallthrough]];
	case formats::RGB888:
		break;
	case formats::XBGR8888:
	case formats::ABGR8888:
		addAlphaByte = true;
		[[fallthrough]];
	case formats::BGR888:
		/* Swap R and B in bayer order to generate BGR888 instead of RGB888 */
		swapRedBlueGains_ = true;

		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			bayerFormat.order = BayerFormat::RGGB;
			break;
		case BayerFormat::GBRG:
			bayerFormat.order = BayerFormat::GRBG;
			break;
		case BayerFormat::GRBG:
			bayerFormat.order = BayerFormat::GBRG;
			break;
		case BayerFormat::RGGB:
			bayerFormat.order = BayerFormat::BGGR;
			break;
		default:
			return invalidFmt();
		}
		break;
	default:
		return invalidFmt();
	}

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10 || bayerFormat.bitDepth == 12) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		switch (bayerFormat.bitDepth) {
		case 8:
			SET_DEBAYER_METHODS(debayer8_BGBG_BGR888, debayer8_GRGR_BGR888)
			break;
		case 10:
			SET_DEBAYER_METHODS(debayer10_BGBG_BGR888, debayer10_GRGR_BGR888)
			break;
		case 12:
			SET_DEBAYER_METHODS(debayer12_BGBG_BGR888, debayer12_GRGR_BGR888)
			break;
		}
		setupStandardBayerOrder(bayerFormat.order);
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2) {
		switch (bayerFormat.order) {
		case BayerFormat::BGGR:
			SET_DEBAYER_METHODS(debayer10P_BGBG_BGR888, debayer10P_GRGR_BGR888)
			return 0;
		case BayerFormat::GBRG:
			SET_DEBAYER_METHODS(debayer10P_GBGB_BGR888, debayer10P_RGRG_BGR888)
			return 0;
		case BayerFormat::GRBG:
			SET_DEBAYER_METHODS(debayer10P_GRGR_BGR888, debayer10P_BGBG_BGR888)
			return 0;
		case BayerFormat::RGGB:
			SET_DEBAYER_METHODS(debayer10P_RGRG_BGR888, debayer10P_GBGB_BGR888)
			return 0;
		default:
			break;
		}
	}

	return invalidFmt();
}

int DebayerCpu::configure(const StreamConfiguration &inputCfg,
			  const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
			  bool ccmEnabled)
{
	if (getInputConfig(inputCfg.pixelFormat, inputConfig_) != 0)
		return -EINVAL;

	if (stats_->configure(inputCfg) != 0)
		return -EINVAL;

	const Size &statsPatternSize = stats_->patternSize();
	if (inputConfig_.patternSize.width != statsPatternSize.width ||
	    inputConfig_.patternSize.height != statsPatternSize.height) {
		LOG(Debayer, Error)
			<< "mismatching stats and debayer pattern sizes for "
			<< inputCfg.pixelFormat.toString();
		return -EINVAL;
	}

	inputConfig_.stride = inputCfg.stride;

	if (outputCfgs.size() != 1) {
		LOG(Debayer, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	const StreamConfiguration &outputCfg = outputCfgs[0];
	SizeRange outSizeRange = sizes(inputCfg.pixelFormat, inputCfg.size);
	std::tie(outputConfig_.stride, outputConfig_.frameSize) =
		strideAndFrameSize(outputCfg.pixelFormat, outputCfg.size);

	if (!outSizeRange.contains(outputCfg.size) || outputConfig_.stride != outputCfg.stride) {
		LOG(Debayer, Error)
			<< "Invalid output size/stride: "
			<< "\n  " << outputCfg.size << " (" << outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " (" << outputConfig_.stride << ")";
		return -EINVAL;
	}

	int ret = setDebayerFunctions(inputCfg.pixelFormat,
				      outputCfg.pixelFormat,
				      ccmEnabled);
	if (ret != 0)
		return -EINVAL;

	window_.x = ((inputCfg.size.width - outputCfg.size.width) / 2) &
		    ~(inputConfig_.patternSize.width - 1);
	window_.y = ((inputCfg.size.height - outputCfg.size.height) / 2) &
		    ~(inputConfig_.patternSize.height - 1);
	window_.width = outputCfg.size.width;
	window_.height = outputCfg.size.height;

	/* Don't pass x,y since process() already adjusts src before passing it */
	stats_->setWindow(Rectangle(window_.size()));

	/* pad with patternSize.Width on both left and right side */
	lineBufferPadding_ = inputConfig_.patternSize.width * inputConfig_.bpp / 8;
	lineBufferLength_ = window_.width * inputConfig_.bpp / 8 +
			    2 * lineBufferPadding_;

	if (enableInputMemcpy_) {
		for (unsigned int i = 0; i <= inputConfig_.patternSize.height; i++)
			lineBuffers_[i].resize(lineBufferLength_);
	}

	measuredFrames_ = 0;
	frameProcessTime_ = 0;

	return 0;
}

/*
 * Get width and height at which the bayer-pattern repeats.
 * Return pattern-size or an empty Size for an unsupported inputFormat.
 */
Size DebayerCpu::patternSize(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return {};

	return config.patternSize;
}

std::vector<PixelFormat> DebayerCpu::formats(PixelFormat inputFormat)
{
	DebayerCpu::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return std::vector<PixelFormat>();

	return config.outputFormats;
}

std::tuple<unsigned int, unsigned int>
DebayerCpu::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	DebayerCpu::DebayerOutputConfig config;

	if (getOutputConfig(outputFormat, config) != 0)
		return std::make_tuple(0, 0);

	/* round up to multiple of 8 for 64 bits alignment */
	unsigned int stride = (size.width * config.bpp / 8 + 7) & ~7;

	return std::make_tuple(stride, stride * size.height);
}

void DebayerCpu::setupInputMemcpy(const uint8_t *linePointers[])
{
	const unsigned int patternHeight = inputConfig_.patternSize.height;

	if (!enableInputMemcpy_)
		return;

	for (unsigned int i = 0; i < patternHeight; i++) {
		memcpy(lineBuffers_[i].data(),
		       linePointers[i + 1] - lineBufferPadding_,
		       lineBufferLength_);
		linePointers[i + 1] = lineBuffers_[i].data() + lineBufferPadding_;
	}

	/* Point lineBufferIndex_ to first unused lineBuffer */
	lineBufferIndex_ = patternHeight;
}

void DebayerCpu::shiftLinePointers(const uint8_t *linePointers[], const uint8_t *src)
{
	const unsigned int patternHeight = inputConfig_.patternSize.height;

	for (unsigned int i = 0; i < patternHeight; i++)
		linePointers[i] = linePointers[i + 1];

	linePointers[patternHeight] = src +
				      (patternHeight / 2) * (int)inputConfig_.stride;
}

void DebayerCpu::memcpyNextLine(const uint8_t *linePointers[])
{
	const unsigned int patternHeight = inputConfig_.patternSize.height;

	if (!enableInputMemcpy_)
		return;

	memcpy(lineBuffers_[lineBufferIndex_].data(),
	       linePointers[patternHeight] - lineBufferPadding_,
	       lineBufferLength_);
	linePointers[patternHeight] = lineBuffers_[lineBufferIndex_].data() + lineBufferPadding_;

	lineBufferIndex_ = (lineBufferIndex_ + 1) % (patternHeight + 1);
}

void DebayerCpu::process2(const uint8_t *src, uint8_t *dst)
{
	unsigned int yEnd = window_.y + window_.height;
	/* Holds [0] previous- [1] current- [2] next-line */
	const uint8_t *linePointers[3];

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	/* [x] becomes [x - 1] after initial shiftLinePointers() call */
	if (window_.y) {
		linePointers[1] = src - inputConfig_.stride; /* previous-line */
		linePointers[2] = src;
	} else {
		/* window_.y == 0, use the next line as prev line */
		linePointers[1] = src + inputConfig_.stride;
		linePointers[2] = src;
		/* Last 2 lines also need special handling */
		yEnd -= 2;
	}

	setupInputMemcpy(linePointers);

	for (unsigned int y = window_.y; y < yEnd; y += 2) {
		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		stats_->processLine0(y, linePointers);
		(this->*debayer0_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		(this->*debayer1_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;
	}

	if (window_.y == 0) {
		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		stats_->processLine0(yEnd, linePointers);
		(this->*debayer0_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		shiftLinePointers(linePointers, src);
		/* next line may point outside of src, use prev. */
		linePointers[2] = linePointers[0];
		(this->*debayer1_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;
	}
}

void DebayerCpu::process4(const uint8_t *src, uint8_t *dst)
{
	const unsigned int yEnd = window_.y + window_.height;
	/*
	 * This holds pointers to [0] 2-lines-up [1] 1-line-up [2] current-line
	 * [3] 1-line-down [4] 2-lines-down.
	 */
	const uint8_t *linePointers[5];

	/* Adjust src to top left corner of the window */
	src += window_.y * inputConfig_.stride + window_.x * inputConfig_.bpp / 8;

	/* [x] becomes [x - 1] after initial shiftLinePointers() call */
	linePointers[1] = src - 2 * inputConfig_.stride;
	linePointers[2] = src - inputConfig_.stride;
	linePointers[3] = src;
	linePointers[4] = src + inputConfig_.stride;

	setupInputMemcpy(linePointers);

	for (unsigned int y = window_.y; y < yEnd; y += 4) {
		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		stats_->processLine0(y, linePointers);
		(this->*debayer0_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		(this->*debayer1_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		stats_->processLine2(y, linePointers);
		(this->*debayer2_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;

		shiftLinePointers(linePointers, src);
		memcpyNextLine(linePointers);
		(this->*debayer3_)(dst, linePointers);
		src += inputConfig_.stride;
		dst += outputConfig_.stride;
	}
}

namespace {

inline int64_t timeDiff(timespec &after, timespec &before)
{
	return (after.tv_sec - before.tv_sec) * 1000000000LL +
	       (int64_t)after.tv_nsec - (int64_t)before.tv_nsec;
}

} /* namespace */

void DebayerCpu::process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params)
{
	timespec frameStartTime;

	if (measuredFrames_ < DebayerCpu::kLastFrameToMeasure) {
		frameStartTime = {};
		clock_gettime(CLOCK_MONOTONIC_RAW, &frameStartTime);
	}

	std::vector<DmaSyncer> dmaSyncers;
	for (const FrameBuffer::Plane &plane : input->planes())
		dmaSyncers.emplace_back(plane.fd, DmaSyncer::SyncType::Read);

	for (const FrameBuffer::Plane &plane : output->planes())
		dmaSyncers.emplace_back(plane.fd, DmaSyncer::SyncType::Write);

	green_ = params.green;
	greenCcm_ = params.greenCcm;
	if (swapRedBlueGains_) {
		red_ = params.blue;
		blue_ = params.red;
		redCcm_ = params.blueCcm;
		blueCcm_ = params.redCcm;
		for (unsigned int i = 0; i < 256; i++) {
			std::swap(redCcm_[i].r, redCcm_[i].b);
			std::swap(blueCcm_[i].r, blueCcm_[i].b);
		}
	} else {
		red_ = params.red;
		blue_ = params.blue;
		redCcm_ = params.redCcm;
		blueCcm_ = params.blueCcm;
	}
	gammaLut_ = params.gammaLut;

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	MappedFrameBuffer out(output, MappedFrameBuffer::MapFlag::Write);
	if (!in.isValid() || !out.isValid()) {
		LOG(Debayer, Error) << "mmap-ing buffer(s) failed";
		metadata.status = FrameMetadata::FrameError;
		return;
	}

	stats_->startFrame();

	if (inputConfig_.patternSize.height == 2)
		process2(in.planes()[0].data(), out.planes()[0].data());
	else
		process4(in.planes()[0].data(), out.planes()[0].data());

	metadata.planes()[0].bytesused = out.planes()[0].size();

	dmaSyncers.clear();

	/* Measure before emitting signals */
	if (measuredFrames_ < DebayerCpu::kLastFrameToMeasure &&
	    ++measuredFrames_ > DebayerCpu::kFramesToSkip) {
		timespec frameEndTime = {};
		clock_gettime(CLOCK_MONOTONIC_RAW, &frameEndTime);
		frameProcessTime_ += timeDiff(frameEndTime, frameStartTime);
		if (measuredFrames_ == DebayerCpu::kLastFrameToMeasure) {
			const unsigned int measuredFrames = DebayerCpu::kLastFrameToMeasure -
							    DebayerCpu::kFramesToSkip;
			LOG(Debayer, Info)
				<< "Processed " << measuredFrames
				<< " frames in " << frameProcessTime_ / 1000 << "us, "
				<< frameProcessTime_ / (1000 * measuredFrames)
				<< " us/frame";
		}
	}

	/*
	 * Buffer ids are currently not used, so pass zeros as its parameter.
	 *
	 * \todo Pass real bufferId once stats buffer passing is changed.
	 */
	stats_->finishFrame(frame, 0);
	outputBufferReady.emit(output);
	inputBufferReady.emit(input);
}

SizeRange DebayerCpu::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	Size patternSize = this->patternSize(inputFormat);
	unsigned int borderHeight = patternSize.height;

	if (patternSize.isNull())
		return {};

	/* No need for top/bottom border with a pattern height of 2 */
	if (patternSize.height == 2)
		borderHeight = 0;

	/*
	 * For debayer interpolation a border is kept around the entire image
	 * and the minimum output size is pattern-height x pattern-width.
	 */
	if (inputSize.width < (3 * patternSize.width) ||
	    inputSize.height < (2 * borderHeight + patternSize.height)) {
		LOG(Debayer, Warning)
			<< "Input format size too small: " << inputSize.toString();
		return {};
	}

	return SizeRange(Size(patternSize.width, patternSize.height),
			 Size((inputSize.width - 2 * patternSize.width) & ~(patternSize.width - 1),
			      (inputSize.height - 2 * borderHeight) & ~(patternSize.height - 1)),
			 patternSize.width, patternSize.height);
}

} /* namespace libcamera */
