/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_texture_yuv.cpp - SDL Texture YUYV
 */

#include "sdl_texture_yuv.h"

using namespace libcamera;

SDLTextureYUYV::SDLTextureYUYV(const SDL_Rect &rect, unsigned int stride)
	: SDLTexture(rect, SDL_PIXELFORMAT_YUY2, stride)
{
}

void SDLTextureYUYV::update(Span<const uint8_t> data)
{
	SDL_UpdateTexture(ptr_, &rect_, data.data(), pitch_);
}
