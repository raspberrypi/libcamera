/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * SDL Texture
 */

#pragma once

#include <libcamera/base/span.h>

#include <SDL2/SDL.h>

#include "../common/image.h"

class SDLTexture
{
public:
	SDLTexture(const SDL_Rect &rect, uint32_t pixelFormat, const int stride);
	virtual ~SDLTexture();
	int create(SDL_Renderer *renderer);
	virtual void update(libcamera::Span<const libcamera::Span<const uint8_t>> data) = 0;
	SDL_Texture *get() const { return ptr_; }

protected:
	SDL_Texture *ptr_;
	const SDL_Rect rect_;
	const uint32_t pixelFormat_;
	const int stride_;
};
