/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * SDL single plane textures
 */

#include "sdl_texture_1plane.h"

#include <assert.h>

void SDLTexture1Plane::update(libcamera::Span<const libcamera::Span<const uint8_t>> data)
{
	assert(data.size() == 1);
	assert(data[0].size_bytes() == std::size_t(rect_.h) * std::size_t(stride_));
	SDL_UpdateTexture(ptr_, nullptr, data[0].data(), stride_);
}
