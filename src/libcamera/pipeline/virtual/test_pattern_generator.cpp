/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Derived class of FrameGenerator for generating test patterns
 */

#include "test_pattern_generator.h"

#include <string.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include <libyuv/convert_from_argb.h>

namespace {

template<size_t SampleSize>
void rotateLeft1Column(const libcamera::Size &size, uint8_t *image)
{
	if (size.width < 2)
		return;

	const size_t stride = size.width * SampleSize;
	uint8_t first[SampleSize];

	for (size_t i = 0; i < size.height; i++, image += stride) {
		memcpy(first, &image[0], SampleSize);
		memmove(&image[0], &image[SampleSize], stride - SampleSize);
		memcpy(&image[stride - SampleSize], first, SampleSize);
	}
}

} /* namespace */

namespace libcamera {

LOG_DECLARE_CATEGORY(Virtual)

static const unsigned int kARGBSize = 4;

int TestPatternGenerator::generateFrame(const Size &size,
					const FrameBuffer *buffer)
{
	MappedFrameBuffer mappedFrameBuffer(buffer,
					    MappedFrameBuffer::MapFlag::Write);

	const auto &planes = mappedFrameBuffer.planes();

	rotateLeft1Column<kARGBSize>(size, template_.get());

	/* Convert the template_ to the frame buffer */
	int ret = libyuv::ARGBToNV12(template_.get(), size.width * kARGBSize,
				     planes[0].begin(), size.width,
				     planes[1].begin(), size.width,
				     size.width, size.height);
	if (ret != 0)
		LOG(Virtual, Error) << "ARGBToNV12() failed with " << ret;

	return ret;
}

void ColorBarsGenerator::configure(const Size &size)
{
	constexpr uint8_t kColorBar[8][3] = {
		/*  R,    G,    B */
		{ 0xff, 0xff, 0xff }, /* White */
		{ 0xff, 0xff, 0x00 }, /* Yellow */
		{ 0x00, 0xff, 0xff }, /* Cyan */
		{ 0x00, 0xff, 0x00 }, /* Green */
		{ 0xff, 0x00, 0xff }, /* Magenta */
		{ 0xff, 0x00, 0x00 }, /* Red */
		{ 0x00, 0x00, 0xff }, /* Blue */
		{ 0x00, 0x00, 0x00 }, /* Black */
	};

	template_ = std::make_unique<uint8_t[]>(
		size.width * size.height * kARGBSize);

	unsigned int colorBarWidth = size.width / std::size(kColorBar);

	uint8_t *buf = template_.get();
	for (size_t h = 0; h < size.height; h++) {
		for (size_t w = 0; w < size.width; w++) {
			/* Repeat when the width is exceed */
			unsigned int index = (w / colorBarWidth) % std::size(kColorBar);

			*buf++ = kColorBar[index][2]; /* B */
			*buf++ = kColorBar[index][1]; /* G */
			*buf++ = kColorBar[index][0]; /* R */
			*buf++ = 0x00; /* A */
		}
	}
}

void DiagonalLinesGenerator::configure(const Size &size)
{
	constexpr uint8_t kColorBar[2][3] = {
		/*  R,    G,    B */
		{ 0xff, 0xff, 0xff }, /* White */
		{ 0x00, 0x00, 0x00 }, /* Black */
	};

	template_ = std::make_unique<uint8_t[]>(
		size.width * size.height * kARGBSize);

	unsigned int lineWidth = size.width / 10;

	uint8_t *buf = template_.get();
	for (size_t h = 0; h < size.height; h++) {
		for (size_t w = 0; w < size.width; w++) {
			/* Repeat when the width is exceed */
			int index = ((w + h) / lineWidth) % 2;

			*buf++ = kColorBar[index][2]; /* B */
			*buf++ = kColorBar[index][1]; /* G */
			*buf++ = kColorBar[index][0]; /* R */
			*buf++ = 0x00; /* A */
		}
	}
}

} /* namespace libcamera */
