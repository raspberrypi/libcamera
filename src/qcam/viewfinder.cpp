/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder.cpp - qcam - Viewfinder
 */

#include <QImage>
#include <QPainter>

#include "format_converter.h"
#include "viewfinder.h"

ViewFinder::ViewFinder(QWidget *parent)
	: QWidget(parent), format_(0), width_(0), height_(0), image_(nullptr)
{
}

ViewFinder::~ViewFinder()
{
	delete image_;
}

void ViewFinder::display(const unsigned char *raw, size_t size)
{
	converter_.convert(raw, size, image_);
	update();
}

int ViewFinder::setFormat(unsigned int format, unsigned int width,
			  unsigned int height)
{
	int ret;

	ret = converter_.configure(format, width, height);
	if (ret < 0)
		return ret;

	format_ = format;
	width_ = width;
	height_ = height;

	delete image_;
	image_ = new QImage(width, height, QImage::Format_RGB32);

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
	return image_ ? image_->size() : QSize(640, 480);
}
