/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder_qt.cpp - qcam - QPainter-based ViewFinder
 */

#include "viewfinder_qt.h"

#include <assert.h>
#include <stdint.h>
#include <utility>

#include <libcamera/formats.h>

#include <QImage>
#include <QImageWriter>
#include <QMap>
#include <QMutexLocker>
#include <QPainter>
#include <QtDebug>

#include "../common/image.h"

#include "format_converter.h"

static const QMap<libcamera::PixelFormat, QImage::Format> nativeFormats
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 2, 0)
	{ libcamera::formats::ABGR8888, QImage::Format_RGBX8888 },
	{ libcamera::formats::XBGR8888, QImage::Format_RGBX8888 },
#endif
	{ libcamera::formats::ARGB8888, QImage::Format_RGB32 },
	{ libcamera::formats::XRGB8888, QImage::Format_RGB32 },
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
	{ libcamera::formats::RGB888, QImage::Format_BGR888 },
#endif
	{ libcamera::formats::BGR888, QImage::Format_RGB888 },
};

ViewFinderQt::ViewFinderQt(QWidget *parent)
	: QWidget(parent), buffer_(nullptr)
{
	icon_ = QIcon(":camera-off.svg");
}

ViewFinderQt::~ViewFinderQt()
{
}

const QList<libcamera::PixelFormat> &ViewFinderQt::nativeFormats() const
{
	static const QList<libcamera::PixelFormat> formats = ::nativeFormats.keys();
	return formats;
}

int ViewFinderQt::setFormat(const libcamera::PixelFormat &format, const QSize &size,
			    [[maybe_unused]] const libcamera::ColorSpace &colorSpace,
			    unsigned int stride)
{
	image_ = QImage();

	/*
	 * If format conversion is needed, configure the converter and allocate
	 * the destination image.
	 */
	if (!::nativeFormats.contains(format)) {
		int ret = converter_.configure(format, size, stride);
		if (ret < 0)
			return ret;

		image_ = QImage(size, QImage::Format_RGB32);

		qInfo() << "Using software format conversion from" << format;
	} else {
		qInfo() << "Zero-copy enabled";
	}

	format_ = format;
	size_ = size;

	updateGeometry();
	return 0;
}

void ViewFinderQt::render(libcamera::FrameBuffer *buffer, Image *image)
{
	size_t size = buffer->metadata().planes()[0].bytesused;

	{
		QMutexLocker locker(&mutex_);

		if (::nativeFormats.contains(format_)) {
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
			assert(buffer->planes().size() == 1);
			image_ = QImage(image->data(0).data(), size_.width(),
					size_.height(), size / size_.height(),
					::nativeFormats[format_]);
			std::swap(buffer, buffer_);
		} else {
			/*
			 * Otherwise, convert the format and release the frame
			 * buffer immediately.
			 */
			converter_.convert(image, size, &image_);
		}
	}

	update();

	if (buffer)
		renderComplete(buffer);
}

void ViewFinderQt::stop()
{
	image_ = QImage();

	if (buffer_) {
		renderComplete(buffer_);
		buffer_ = nullptr;
	}

	update();
}

QImage ViewFinderQt::getCurrentImage()
{
	QMutexLocker locker(&mutex_);

	return image_.copy();
}

void ViewFinderQt::paintEvent(QPaintEvent *)
{
	QPainter painter(this);

	/* If we have an image, draw it. */
	if (!image_.isNull()) {
		painter.drawImage(rect(), image_, image_.rect());
		return;
	}

	/*
	 * Otherwise, draw the camera stopped icon. Render it to the pixmap if
	 * the size has changed.
	 */
	constexpr int margin = 20;

	if (vfSize_ != size() || pixmap_.isNull()) {
		QSize vfSize = size() - QSize{ 2 * margin, 2 * margin };
		QSize pixmapSize{ 1, 1 };
		pixmapSize.scale(vfSize, Qt::KeepAspectRatio);
		pixmap_ = icon_.pixmap(pixmapSize);

		vfSize_ = size();
	}

	QPoint point{ margin, margin };
	if (pixmap_.width() < width() - 2 * margin)
		point.setX((width() - pixmap_.width()) / 2);
	else
		point.setY((height() - pixmap_.height()) / 2);

	painter.setBackgroundMode(Qt::OpaqueMode);
	painter.drawPixmap(point, pixmap_);
}

QSize ViewFinderQt::sizeHint() const
{
	return size_.isValid() ? size_ : QSize(640, 480);
}
