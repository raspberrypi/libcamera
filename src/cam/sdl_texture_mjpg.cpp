/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_texture_mjpg.cpp - SDL Texture MJPG
 */

#include "sdl_texture_mjpg.h"

#include <SDL2/SDL_image.h>

using namespace libcamera;

SDLTextureMJPG::SDLTextureMJPG(const SDL_Rect &rect)
	: SDLTexture(rect, SDL_PIXELFORMAT_RGB24, 0)
{
}

void SDLTextureMJPG::update(const Span<uint8_t> &data)
{
	SDL_RWops *bufferStream = SDL_RWFromMem(data.data(), data.size());
	SDL_Surface *frame = IMG_Load_RW(bufferStream, 0);
	SDL_UpdateTexture(ptr_, nullptr, frame->pixels, frame->pitch);
	SDL_FreeSurface(frame);
}
