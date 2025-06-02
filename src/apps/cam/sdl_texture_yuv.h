/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * SDL YUV Textures
 */

#pragma once

#include "sdl_texture.h"

#if SDL_VERSION_ATLEAST(2, 0, 16)
class SDLTextureNV12 : public SDLTexture
{
public:
	SDLTextureNV12(const SDL_Rect &rect, unsigned int stride);
	void update(libcamera::Span<const libcamera::Span<const uint8_t>> data) override;
};
#endif
