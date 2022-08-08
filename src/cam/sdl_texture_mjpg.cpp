/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_texture_mjpg.cpp - SDL Texture MJPG
 */

#include "sdl_texture_mjpg.h"

#include <iostream>
#include <setjmp.h>
#include <stdio.h>

#include <jpeglib.h>

using namespace libcamera;

struct JpegErrorManager : public jpeg_error_mgr {
	JpegErrorManager()
	{
		jpeg_std_error(this);
		error_exit = errorExit;
		output_message = outputMessage;
	}

	static void errorExit(j_common_ptr cinfo)
	{
		JpegErrorManager *self =
			static_cast<JpegErrorManager *>(cinfo->err);
		longjmp(self->escape_, 1);
	}

	static void outputMessage([[maybe_unused]] j_common_ptr cinfo)
	{
	}

	jmp_buf escape_;
};

SDLTextureMJPG::SDLTextureMJPG(const SDL_Rect &rect)
	: SDLTexture(rect, SDL_PIXELFORMAT_RGB24, rect.w * 3),
	  rgb_(std::make_unique<unsigned char[]>(stride_ * rect.h))
{
}

int SDLTextureMJPG::decompress(Span<const uint8_t> data)
{
	struct jpeg_decompress_struct cinfo;

	JpegErrorManager errorManager;
	if (setjmp(errorManager.escape_)) {
		/* libjpeg found an error */
		jpeg_destroy_decompress(&cinfo);
		std::cerr << "JPEG decompression error" << std::endl;
		return -EINVAL;
	}

	cinfo.err = &errorManager;
	jpeg_create_decompress(&cinfo);

	jpeg_mem_src(&cinfo, data.data(), data.size());

	jpeg_read_header(&cinfo, TRUE);

	jpeg_start_decompress(&cinfo);

	for (int i = 0; cinfo.output_scanline < cinfo.output_height; ++i) {
		JSAMPROW rowptr = rgb_.get() + i * stride_;
		jpeg_read_scanlines(&cinfo, &rowptr, 1);
	}

	jpeg_finish_decompress(&cinfo);

	jpeg_destroy_decompress(&cinfo);

	return 0;
}

void SDLTextureMJPG::update(const std::vector<libcamera::Span<const uint8_t>> &data)
{
	decompress(data[0]);
	SDL_UpdateTexture(ptr_, nullptr, rgb_.get(), stride_);
}
