/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder.h - qcam - Viewfinder
 */
#ifndef __QCAM_VIEWFINDER_H__
#define __QCAM_VIEWFINDER_H__

#include <QMutex>
#include <QSize>
#include <QWidget>

#include <libcamera/pixelformats.h>

#include "format_converter.h"

class QImage;

class ViewFinder : public QWidget
{
public:
	ViewFinder(QWidget *parent);
	~ViewFinder();

	int setFormat(const libcamera::PixelFormat &format, const QSize &size);
	void display(const unsigned char *rgb, size_t size);

	QImage getCurrentImage();

protected:
	void paintEvent(QPaintEvent *) override;
	QSize sizeHint() const override;

private:
	FormatConverter converter_;

	libcamera::PixelFormat format_;
	QSize size_;

	QImage *image_;
	QMutex mutex_; /* Prevent concurrent access to image_ */
};

#endif /* __QCAM_VIEWFINDER__ */
