/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder.h - qcam - Viewfinder base class
 */

#pragma once

#include <QImage>
#include <QList>
#include <QSize>

#include <libcamera/color_space.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>

class Image;

class ViewFinder
{
public:
	virtual ~ViewFinder() = default;

	virtual const QList<libcamera::PixelFormat> &nativeFormats() const = 0;

	virtual int setFormat(const libcamera::PixelFormat &format, const QSize &size,
			      const libcamera::ColorSpace &colorSpace,
			      unsigned int stride) = 0;
	virtual void render(libcamera::FrameBuffer *buffer, Image *image) = 0;
	virtual void stop() = 0;

	virtual QImage getCurrentImage() = 0;
};
