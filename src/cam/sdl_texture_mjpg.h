/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_texture_mjpg.h - SDL Texture MJPG
 */

#pragma once

#include "sdl_texture.h"

class SDLTextureMJPG : public SDLTexture
{
public:
	SDLTextureMJPG(const SDL_Rect &rect);

	void update(const std::vector<libcamera::Span<const uint8_t>> &data) override;

private:
	int decompress(libcamera::Span<const uint8_t> data);

	std::unique_ptr<unsigned char[]> rgb_;
};
