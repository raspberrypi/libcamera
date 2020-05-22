/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder.h - qcam - Viewfinder
 */
#ifndef __QCAM_VIEWFINDER_H__
#define __QCAM_VIEWFINDER_H__

#include <stddef.h>

#include <QIcon>
#include <QList>
#include <QImage>
#include <QMutex>
#include <QSize>
#include <QWidget>

#include <libcamera/buffer.h>
#include <libcamera/pixel_format.h>

#include "format_converter.h"

class QImage;

struct MappedBuffer {
	void *memory;
	size_t size;
};

class ViewFinder : public QWidget
{
	Q_OBJECT

public:
	ViewFinder(QWidget *parent);
	~ViewFinder();

	const QList<libcamera::PixelFormat> &nativeFormats() const;

	int setFormat(const libcamera::PixelFormat &format, const QSize &size);
	void render(libcamera::FrameBuffer *buffer, MappedBuffer *map);
	void stop();

	QImage getCurrentImage();

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

#endif /* __QCAM_VIEWFINDER__ */
