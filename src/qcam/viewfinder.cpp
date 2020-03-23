/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder.cpp - qcam - Viewfinder
 */

#include "viewfinder.h"

#include <QImage>
#include <QImageWriter>
#include <QMutexLocker>
#include <QPainter>
#include <QtDebug>

#include "format_converter.h"

ViewFinder::ViewFinder(QWidget *parent)
	: QWidget(parent), format_(0), image_(nullptr)
{
}

ViewFinder::~ViewFinder()
{
	delete image_;
}

void ViewFinder::render(libcamera::FrameBuffer *buffer, MappedBuffer *map)
{
	if (buffer->planes().size() != 1) {
		qWarning() << "Multi-planar buffers are not supported";
		return;
	}

	QMutexLocker locker(&mutex_);

	/*
	 * \todo We're not supposed to block the pipeline handler thread
	 * for long, implement a better way to save images without
	 * impacting performances.
	 */

	converter_.convert(static_cast<unsigned char *>(map->memory),
			   buffer->metadata().planes[0].bytesused, image_);
	update();

	renderComplete(buffer);
}

QImage ViewFinder::getCurrentImage()
{
	QMutexLocker locker(&mutex_);

	return image_->copy();
}

int ViewFinder::setFormat(const libcamera::PixelFormat &format,
			  const QSize &size)
{
	int ret;

	ret = converter_.configure(format, size);
	if (ret < 0)
		return ret;

	format_ = format;
	size_ = size;

	delete image_;
	image_ = new QImage(size_, QImage::Format_RGB32);

	updateGeometry();
	return 0;
}

void ViewFinder::paintEvent(QPaintEvent *)
{
	QPainter painter(this);
	painter.drawImage(rect(), *image_, image_->rect());
}

QSize ViewFinder::sizeHint() const
{
	return size_.isValid() ? size_ : QSize(640, 480);
}
