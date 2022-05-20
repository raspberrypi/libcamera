/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_texture.h - SDL Texture
 */

#pragma once

#include <SDL2/SDL.h>

#include "image.h"

class SDLTexture
{
public:
	SDLTexture(const SDL_Rect &rect, SDL_PixelFormatEnum pixelFormat,
		   const int pitch);
	virtual ~SDLTexture();
	int create(SDL_Renderer *renderer);
	virtual void update(const libcamera::Span<uint8_t> &data) = 0;
	SDL_Texture *get() const { return ptr_; }

protected:
	SDL_Texture *ptr_;
	const SDL_Rect rect_;
	const SDL_PixelFormatEnum pixelFormat_;
	const int pitch_;
};
