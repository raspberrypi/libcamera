/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder_qt.h - qcam - QPainter-based ViewFinder
 */

#pragma once

#include <QIcon>
#include <QImage>
#include <QList>
#include <QMutex>
#include <QSize>
#include <QWidget>

#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/pixel_format.h>

#include "format_converter.h"
#include "viewfinder.h"

class ViewFinderQt : public QWidget, public ViewFinder
{
	Q_OBJECT

public:
	ViewFinderQt(QWidget *parent);
	~ViewFinderQt();

	const QList<libcamera::PixelFormat> &nativeFormats() const override;

	int setFormat(const libcamera::PixelFormat &format, const QSize &size,
		      const libcamera::ColorSpace &colorSpace,
		      unsigned int stride) override;
	void render(libcamera::FrameBuffer *buffer, Image *image) override;
	void stop() override;

	QImage getCurrentImage() override;

Q_SIGNALS:
	void renderComplete(libcamera::FrameBuffer *buffer);

protected:
	void paintEvent(QPaintEvent *) override;
	QSize sizeHint() const override;

private:
	FormatConverter converter_;

	libcamera::PixelFormat format_;
	QSize size_;

	/* Camera stopped icon */
	QSize vfSize_;
	QIcon icon_;
	QPixmap pixmap_;

	/* Buffer and render image */
	libcamera::FrameBuffer *buffer_;
	QImage image_;
	QMutex mutex_; /* Prevent concurrent access to image_ */
};
