/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * exif.h - EXIF tag creator using libexif
 */

#pragma once

#include <chrono>
#include <string>
#include <time.h>
#include <vector>

#include <libexif/exif-data.h>

#include <libcamera/base/span.h>

#include <libcamera/geometry.h>

class Exif
{
public:
	Exif();
	~Exif();

	enum Compression {
		None = 1,
		JPEG = 6,
	};

	enum Flash {
		/* bit 0 */
		Fired = 0x01,
		/* bits 1 and 2 */
		StrobeDetected = 0x04,
		StrobeNotDetected = 0x06,
		/* bits 3 and 4 */
		ModeCompulsoryFiring = 0x08,
		ModeCompulsorySuppression = 0x10,
		ModeAuto = 0x18,
		/* bit 5 */
		FlashNotPresent = 0x20,
		/* bit 6 */
		RedEye = 0x40,
	};

	enum WhiteBalance {
		Auto = 0,
		Manual = 1,
	};

	enum StringEncoding {
		NoEncoding = 0,
		ASCII = 1,
		Unicode = 2,
	};

	void setMake(const std::string &make);
	void setModel(const std::string &model);

	void setOrientation(int orientation);
	void setSize(const libcamera::Size &size);
	void setThumbnail(std::vector<unsigned char> &&thumbnail,
			  Compression compression);
	void setTimestamp(time_t timestamp, std::chrono::milliseconds msec);

	void setGPSDateTimestamp(time_t timestamp);
	void setGPSLocation(const double *coords);
	void setGPSMethod(const std::string &method);

	void setFocalLength(float length);
	void setExposureTime(uint64_t nsec);
	void setAperture(float size);
	void setISO(uint16_t iso);
	void setFlash(Flash flash);
	void setWhiteBalance(WhiteBalance wb);

	libcamera::Span<const uint8_t> data() const { return { exifData_, size_ }; }
	[[nodiscard]] int generate();

private:
	ExifEntry *createEntry(ExifIfd ifd, ExifTag tag);
	ExifEntry *createEntry(ExifIfd ifd, ExifTag tag, ExifFormat format,
			       unsigned long components, unsigned int size);

	void setByte(ExifIfd ifd, ExifTag tag, uint8_t item);
	void setShort(ExifIfd ifd, ExifTag tag, uint16_t item);
	void setLong(ExifIfd ifd, ExifTag tag, uint32_t item);
	void setString(ExifIfd ifd, ExifTag tag, ExifFormat format,
		       const std::string &item,
		       StringEncoding encoding = NoEncoding);
	void setRational(ExifIfd ifd, ExifTag tag, ExifRational item);
	void setRational(ExifIfd ifd, ExifTag tag,
			 libcamera::Span<const ExifRational> items);

	std::tuple<int, int, int> degreesToDMS(double decimalDegrees);
	void setGPSDMS(ExifIfd ifd, ExifTag tag, int deg, int min, int sec);

	std::u16string utf8ToUtf16(const std::string &str);

	bool valid_;

	ExifData *data_;
	ExifMem *mem_;
	ExifByteOrder order_;

	unsigned char *exifData_;
	unsigned int size_;

	std::vector<unsigned char> thumbnailData_;
};
