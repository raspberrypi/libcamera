/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023-2025 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * CPU based debayering header
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

#include <libcamera/base/object.h>

#include "libcamera/internal/bayer_format.h"

#include "debayer.h"
#include "swstats_cpu.h"

namespace libcamera {

class DebayerCpu : public Debayer, public Object
{
public:
	DebayerCpu(std::unique_ptr<SwStatsCpu> stats);
	~DebayerCpu();

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
		      bool ccmEnabled);
	Size patternSize(PixelFormat inputFormat);
	std::vector<PixelFormat> formats(PixelFormat input);
	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &outputFormat, const Size &size);
	void process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params);
	SizeRange sizes(PixelFormat inputFormat, const Size &inputSize);

	/**
	 * \brief Get the file descriptor for the statistics
	 *
	 * \return the file descriptor pointing to the statistics
	 */
	const SharedFD &getStatsFD() { return stats_->getStatsFD(); }

	/**
	 * \brief Get the output frame size
	 *
	 * \return The output frame size
	 */
	unsigned int frameSize() { return outputConfig_.frameSize; }

private:
	/**
	 * \brief Called to debayer 1 line of Bayer input data to output format
	 * \param[out] dst Pointer to the start of the output line to write
	 * \param[in] src The input data
	 *
	 * Input data is an array of (patternSize_.height + 1) src
	 * pointers each pointing to a line in the Bayer source. The middle
	 * element of the array will point to the actual line being processed.
	 * Earlier element(s) will point to the previous line(s) and later
	 * element(s) to the next line(s).
	 *
	 * These functions take an array of src pointers, rather than
	 * a single src pointer + a stride for the source, so that when the src
	 * is slow uncached memory it can be copied to faster memory before
	 * debayering. Debayering a standard 2x2 Bayer pattern requires access
	 * to the previous and next src lines for interpolating the missing
	 * colors. To allow copying the src lines only once 3 temporary buffers
	 * each holding a single line are used, re-using the oldest buffer for
	 * the next line and the pointers are swizzled so that:
	 * src[0] = previous-line, src[1] = currrent-line, src[2] = next-line.
	 * This way the 3 pointers passed to the debayer functions form
	 * a sliding window over the src avoiding the need to copy each
	 * line more than once.
	 *
	 * Similarly for bayer patterns which repeat every 4 lines, 5 src
	 * pointers are passed holding: src[0] = 2-lines-up, src[1] = 1-line-up
	 * src[2] = current-line, src[3] = 1-line-down, src[4] = 2-lines-down.
	 */
	using debayerFn = void (DebayerCpu::*)(uint8_t *dst, const uint8_t *src[]);

	/* 8-bit raw bayer format */
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer8_BGBG_BGR888(uint8_t *dst, const uint8_t *src[]);
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer8_GRGR_BGR888(uint8_t *dst, const uint8_t *src[]);
	/* unpacked 10-bit raw bayer format */
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer10_BGBG_BGR888(uint8_t *dst, const uint8_t *src[]);
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer10_GRGR_BGR888(uint8_t *dst, const uint8_t *src[]);
	/* unpacked 12-bit raw bayer format */
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer12_BGBG_BGR888(uint8_t *dst, const uint8_t *src[]);
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer12_GRGR_BGR888(uint8_t *dst, const uint8_t *src[]);
	/* CSI-2 packed 10-bit raw bayer format (all the 4 orders) */
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer10P_BGBG_BGR888(uint8_t *dst, const uint8_t *src[]);
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer10P_GRGR_BGR888(uint8_t *dst, const uint8_t *src[]);
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer10P_GBGB_BGR888(uint8_t *dst, const uint8_t *src[]);
	template<bool addAlphaByte, bool ccmEnabled>
	void debayer10P_RGRG_BGR888(uint8_t *dst, const uint8_t *src[]);

	struct DebayerInputConfig {
		Size patternSize;
		unsigned int bpp; /* Memory used per pixel, not precision */
		unsigned int stride;
		std::vector<PixelFormat> outputFormats;
	};

	struct DebayerOutputConfig {
		unsigned int bpp; /* Memory used per pixel, not precision */
		unsigned int stride;
		unsigned int frameSize;
	};

	int getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config);
	int getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config);
	int setupStandardBayerOrder(BayerFormat::Order order);
	int setDebayerFunctions(PixelFormat inputFormat,
				PixelFormat outputFormat,
				bool ccmEnabled);
	void setupInputMemcpy(const uint8_t *linePointers[]);
	void shiftLinePointers(const uint8_t *linePointers[], const uint8_t *src);
	void memcpyNextLine(const uint8_t *linePointers[]);
	void process2(const uint8_t *src, uint8_t *dst);
	void process4(const uint8_t *src, uint8_t *dst);

	/* Max. supported Bayer pattern height is 4, debayering this requires 5 lines */
	static constexpr unsigned int kMaxLineBuffers = 5;

	DebayerParams::LookupTable red_;
	DebayerParams::LookupTable green_;
	DebayerParams::LookupTable blue_;
	DebayerParams::CcmLookupTable redCcm_;
	DebayerParams::CcmLookupTable greenCcm_;
	DebayerParams::CcmLookupTable blueCcm_;
	DebayerParams::LookupTable gammaLut_;
	debayerFn debayer0_;
	debayerFn debayer1_;
	debayerFn debayer2_;
	debayerFn debayer3_;
	Rectangle window_;
	DebayerInputConfig inputConfig_;
	DebayerOutputConfig outputConfig_;
	std::unique_ptr<SwStatsCpu> stats_;
	std::vector<uint8_t> lineBuffers_[kMaxLineBuffers];
	unsigned int lineBufferLength_;
	unsigned int lineBufferPadding_;
	unsigned int lineBufferIndex_;
	unsigned int xShift_; /* Offset of 0/1 applied to window_.x */
	bool enableInputMemcpy_;
	bool swapRedBlueGains_;
	unsigned int measuredFrames_;
	int64_t frameProcessTime_;
	/* Skip 30 frames for things to stabilize then measure 30 frames */
	static constexpr unsigned int kFramesToSkip = 30;
	static constexpr unsigned int kLastFrameToMeasure = 60;
};

} /* namespace libcamera */
