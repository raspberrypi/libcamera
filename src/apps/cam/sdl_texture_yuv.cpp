/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * SDL YUV Textures
 */

#include "sdl_texture_yuv.h"

#include <assert.h>

using namespace libcamera;

#if SDL_VERSION_ATLEAST(2, 0, 16)
SDLTextureNV::SDLTextureNV(const SDL_Rect &rect, uint32_t pixelFormat, unsigned int stride)
	: SDLTexture(rect, pixelFormat, stride)
{
	assert(pixelFormat == SDL_PIXELFORMAT_NV12 || pixelFormat == SDL_PIXELFORMAT_NV21);
}

void SDLTextureNV::update(libcamera::Span<const libcamera::Span<const uint8_t>> data)
{
	assert(data.size() == 2);
	assert(data[0].size_bytes() == std::size_t(rect_.h) * std::size_t(stride_));
	assert(data[1].size_bytes() == std::size_t(rect_.h) * std::size_t(stride_) / 2);

	SDL_UpdateNVTexture(ptr_, nullptr, data[0].data(), stride_,
			    data[1].data(), stride_);
}
#endif
