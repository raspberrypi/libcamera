/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder.cpp - qcam - Viewfinder
 */

#include "viewfinder.h"

#include <utility>

#include <QImage>
#include <QImageWriter>
#include <QMutexLocker>
#include <QPainter>
#include <QtDebug>

#include "format_converter.h"

ViewFinder::ViewFinder(QWidget *parent)
	: QWidget(parent), buffer_(nullptr)
{
}

ViewFinder::~ViewFinder()
{
}

int ViewFinder::setFormat(const libcamera::PixelFormat &format,
			  const QSize &size)
{
	image_ = QImage();

	/*
	 * If format conversion is needed, configure the converter and allocate
	 * the destination image.
	 */
	if (format != DRM_FORMAT_ARGB8888) {
		int ret = converter_.configure(format, size);
		if (ret < 0)
			return ret;

		image_ = QImage(size, QImage::Format_RGB32);
	}

	format_ = format;
	size_ = size;

	updateGeometry();
	return 0;
}

void ViewFinder::render(libcamera::FrameBuffer *buffer, MappedBuffer *map)
{
	if (buffer->planes().size() != 1) {
		qWarning() << "Multi-planar buffers are not supported";
		return;
	}

	unsigned char *memory = static_cast<unsigned char *>(map->memory);
	size_t size = buffer->metadata().planes[0].bytesused;

	{
		QMutexLocker locker(&mutex_);

		if (format_ == DRM_FORMAT_ARGB8888) {
			/*
			 * If the frame format is identical to the display
			 * format, create a QImage that references the frame
			 * and store a reference to the frame buffer. The
			 * previously stored frame buffer, if any, will be
			 * released.
			 *
			 * \todo Get the stride from the buffer instead of
			 * computing it naively
			 */
			image_ = QImage(memory, size_.width(), size_.height(),
					size / size_.height(), QImage::Format_RGB32);
			std::swap(buffer, buffer_);
		} else {
			/*
			 * Otherwise, convert the format and release the frame
			 * buffer immediately.
			 */
			converter_.convert(memory, size, &image_);
		}
	}

	update();

	if (buffer)
		renderComplete(buffer);
}

void ViewFinder::stop()
{
	image_ = QImage();

	if (buffer_) {
		renderComplete(buffer_);
		buffer_ = nullptr;
	}

	update();
}

QImage ViewFinder::getCurrentImage()
{
	QMutexLocker locker(&mutex_);

	return image_.copy();
}

void ViewFinder::paintEvent(QPaintEvent *)
{
	QPainter painter(this);
	painter.drawImage(rect(), image_, image_.rect());
}

QSize ViewFinder::sizeHint() const
{
	return size_.isValid() ? size_ : QSize(640, 480);
}
