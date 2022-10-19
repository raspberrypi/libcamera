/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_texture_yuv.cpp - SDL YUV Textures
 */

#include "sdl_texture_yuv.h"

using namespace libcamera;

#if SDL_VERSION_ATLEAST(2, 0, 16)
SDLTextureNV12::SDLTextureNV12(const SDL_Rect &rect, unsigned int stride)
	: SDLTexture(rect, SDL_PIXELFORMAT_NV12, stride)
{
}

void SDLTextureNV12::update(const std::vector<libcamera::Span<const uint8_t>> &data)
{
	SDL_UpdateNVTexture(ptr_, &rect_, data[0].data(), stride_,
			    data[1].data(), stride_);
}
#endif

SDLTextureYUYV::SDLTextureYUYV(const SDL_Rect &rect, unsigned int stride)
	: SDLTexture(rect, SDL_PIXELFORMAT_YUY2, stride)
{
}

void SDLTextureYUYV::update(const std::vector<libcamera::Span<const uint8_t>> &data)
{
	SDL_UpdateTexture(ptr_, &rect_, data[0].data(), stride_);
}
