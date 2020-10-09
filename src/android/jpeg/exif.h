/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * exif.h - EXIF tag creator using libexif
 */
#ifndef __ANDROID_JPEG_EXIF_H__
#define __ANDROID_JPEG_EXIF_H__

#include <string>
#include <time.h>

#include <libexif/exif-data.h>

#include <libcamera/geometry.h>
#include <libcamera/span.h>

class Exif
{
public:
	Exif();
	~Exif();

	void setMake(const std::string &make);
	void setModel(const std::string &model);

	void setOrientation(int orientation);
	void setSize(const libcamera::Size &size);
	void setTimestamp(time_t timestamp);

	libcamera::Span<const uint8_t> data() const { return { exifData_, size_ }; }
	[[nodiscard]] int generate();

private:
	ExifEntry *createEntry(ExifIfd ifd, ExifTag tag);
	ExifEntry *createEntry(ExifIfd ifd, ExifTag tag, ExifFormat format,
			       unsigned long components, unsigned int size);

	void setShort(ExifIfd ifd, ExifTag tag, uint16_t item);
	void setLong(ExifIfd ifd, ExifTag tag, uint32_t item);
	void setString(ExifIfd ifd, ExifTag tag, ExifFormat format,
		       const std::string &item);
	void setRational(ExifIfd ifd, ExifTag tag, ExifRational item);

	bool valid_;

	ExifData *data_;
	ExifMem *mem_;
	ExifByteOrder order_;

	unsigned char *exifData_;
	unsigned int size_;
};

#endif /* __ANDROID_JPEG_EXIF_H__ */
