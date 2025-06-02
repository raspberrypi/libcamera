/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * SDL single plane textures
 */

#pragma once

#include "sdl_texture.h"

class SDLTexture1Plane final : public SDLTexture
{
public:
	using SDLTexture::SDLTexture;

	void update(libcamera::Span<const libcamera::Span<const uint8_t>> data) override;
};
