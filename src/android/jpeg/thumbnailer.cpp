/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * thumbnailer.cpp - Simple image thumbnailer
 */

#include "thumbnailer.h"

#include <libcamera/base/log.h>

#include <libcamera/formats.h>

#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(Thumbnailer)

Thumbnailer::Thumbnailer()
	: valid_(false)
{
}

void Thumbnailer::configure(const Size &sourceSize, PixelFormat pixelFormat)
{
	sourceSize_ = sourceSize;
	pixelFormat_ = pixelFormat;

	if (pixelFormat_ != formats::NV12) {
		LOG(Thumbnailer, Error)
			<< "Failed to configure: Pixel Format "
			<< pixelFormat_ << " unsupported.";
		return;
	}

	valid_ = true;
}

void Thumbnailer::createThumbnail(const FrameBuffer &source,
				  const Size &targetSize,
				  std::vector<unsigned char> *destination)
{
	MappedFrameBuffer frame(&source, MappedFrameBuffer::MapFlag::Read);
	if (!frame.isValid()) {
		LOG(Thumbnailer, Error)
			<< "Failed to map FrameBuffer : "
			<< strerror(frame.error());
		return;
	}

	if (!valid_) {
		LOG(Thumbnailer, Error) << "Config is unconfigured or invalid.";
		return;
	}

	const unsigned int sw = sourceSize_.width;
	const unsigned int sh = sourceSize_.height;
	const unsigned int tw = targetSize.width;
	const unsigned int th = targetSize.height;

	ASSERT(frame.planes().size() == 2);
	ASSERT(tw % 2 == 0 && th % 2 == 0);

	/* Image scaling block implementing nearest-neighbour algorithm. */
	unsigned char *src = frame.planes()[0].data();
	unsigned char *srcC = frame.planes()[1].data();
	unsigned char *srcCb, *srcCr;
	unsigned char *dstY, *srcY;

	size_t dstSize = (th * tw) + ((th / 2) * tw);
	destination->resize(dstSize);
	unsigned char *dst = destination->data();
	unsigned char *dstC = dst + th * tw;

	for (unsigned int y = 0; y < th; y += 2) {
		unsigned int sourceY = (sh * y + th / 2) / th;

		dstY = dst + y * tw;
		srcY = src + sw * sourceY;
		srcCb = srcC + (sourceY / 2) * sw + 0;
		srcCr = srcC + (sourceY / 2) * sw + 1;

		for (unsigned int x = 0; x < tw; x += 2) {
			unsigned int sourceX = (sw * x + tw / 2) / tw;

			dstY[x] = srcY[sourceX];
			dstY[tw + x] = srcY[sw + sourceX];
			dstY[x + 1] = srcY[sourceX + 1];
			dstY[tw + x + 1] = srcY[sw + sourceX + 1];

			dstC[(y / 2) * tw + x + 0] = srcCb[(sourceX / 2) * 2];
			dstC[(y / 2) * tw + x + 1] = srcCr[(sourceX / 2) * 2];
		}
	}
}
