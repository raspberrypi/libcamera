/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * SDL YUV Textures
 */

#include "sdl_texture_yuv.h"

using namespace libcamera;

#if SDL_VERSION_ATLEAST(2, 0, 16)
SDLTextureNV12::SDLTextureNV12(const SDL_Rect &rect, unsigned int stride)
	: SDLTexture(rect, SDL_PIXELFORMAT_NV12, stride)
{
}

void SDLTextureNV12::update(libcamera::Span<const libcamera::Span<const uint8_t>> data)
{
	SDL_UpdateNVTexture(ptr_, nullptr, data[0].data(), stride_,
			    data[1].data(), stride_);
}
#endif
