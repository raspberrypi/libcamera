/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * sdl_sink.h - SDL Sink
 */

#pragma once

#include <map>
#include <memory>

#include <libcamera/stream.h>

#include <SDL2/SDL.h>

#include "frame_sink.h"

class Image;
class SDLTexture;

class SDLSink : public FrameSink
{
public:
	SDLSink();
	~SDLSink();

	int configure(const libcamera::CameraConfiguration &config) override;
	int start() override;
	int stop() override;
	void mapBuffer(libcamera::FrameBuffer *buffer) override;

	bool processRequest(libcamera::Request *request) override;

private:
	void renderBuffer(libcamera::FrameBuffer *buffer);
	void processSDLEvents();

	std::map<libcamera::FrameBuffer *, std::unique_ptr<Image>>
		mappedBuffers_;

	std::unique_ptr<SDLTexture> texture_;

	SDL_Window *window_;
	SDL_Renderer *renderer_;
	SDL_Rect rect_;
	bool init_;
};
