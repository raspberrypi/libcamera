/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * viewfinder.h - qcam - Viewfinder
 */
#ifndef __QCAM_VIEWFINDER_H__
#define __QCAM_VIEWFINDER_H__

#include <QLabel>

#include "format_converter.h"

class QImage;

class ViewFinder : public QLabel
{
public:
	ViewFinder(QWidget *parent);

	int setFormat(unsigned int format, unsigned int width,
		      unsigned int height);
	void display(const unsigned char *rgb, size_t size);

private:
	unsigned int format_;
	unsigned int width_;
	unsigned int height_;

	FormatConverter converter_;
	QImage *image_;
};

#endif /* __QCAM_VIEWFINDER__ */
