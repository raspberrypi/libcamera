/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * format_convert.h - qcam - Convert buffer to RGB
 */
#ifndef __QCAM_FORMAT_CONVERTER_H__
#define __QCAM_FORMAT_CONVERTER_H__

class FormatConverter
{
public:
	int configure(unsigned int format, unsigned int width,
		      unsigned int height);

	void convert(const unsigned char *src, unsigned char *dst);

private:
	unsigned int width_;
	unsigned int height_;
	unsigned int y_pos_;
	unsigned int cb_pos_;
};

#endif /* __QCAM_FORMAT_CONVERTER_H__ */
