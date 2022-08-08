/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_texture.cpp - SDL Texture
 */

#include "sdl_texture.h"

#include <iostream>

SDLTexture::SDLTexture(const SDL_Rect &rect, uint32_t pixelFormat,
		       const int stride)
	: ptr_(nullptr), rect_(rect), pixelFormat_(pixelFormat), stride_(stride)
{
}

SDLTexture::~SDLTexture()
{
	if (ptr_)
		SDL_DestroyTexture(ptr_);
}

int SDLTexture::create(SDL_Renderer *renderer)
{
	ptr_ = SDL_CreateTexture(renderer, pixelFormat_,
				 SDL_TEXTUREACCESS_STREAMING, rect_.w,
				 rect_.h);
	if (!ptr_) {
		std::cerr << "Failed to create SDL texture: " << SDL_GetError()
			  << std::endl;
		return -ENOMEM;
	}

	return 0;
}
