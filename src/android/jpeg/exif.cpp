/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * exif.cpp - EXIF tag creation using libexif
 */

#include "exif.h"

#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <tuple>
#include <uchar.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

using namespace libcamera;

LOG_DEFINE_CATEGORY(EXIF)

/*
 * List of EXIF tags that we set directly because they are not supported
 * by libexif version 0.6.21.
 */
enum class _ExifTag {
	OFFSET_TIME              = 0x9010,
	OFFSET_TIME_ORIGINAL     = 0x9011,
	OFFSET_TIME_DIGITIZED    = 0x9012,
};

/*
 * The Exif class should be instantiated and specific properties set
 * through the exposed public API.
 *
 * Once all desired properties have been set, the user shall call
 * generate() to process the entries and generate the Exif data.
 *
 * Calls to generate() must check the return code to determine if any error
 * occurred during the construction of the Exif data, and if successful the
 * data can be obtained using the data() function.
 */
Exif::Exif()
	: valid_(false), data_(nullptr), order_(EXIF_BYTE_ORDER_INTEL),
	  exifData_(0), size_(0)
{
	/* Create an ExifMem allocator to construct entries. */
	mem_ = exif_mem_new_default();
	if (!mem_) {
		LOG(EXIF, Error) << "Failed to allocate ExifMem Allocator";
		return;
	}

	data_ = exif_data_new_mem(mem_);
	if (!data_) {
		LOG(EXIF, Error) << "Failed to allocate an ExifData structure";
		return;
	}

	valid_ = true;

	exif_data_set_option(data_, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
	exif_data_set_data_type(data_, EXIF_DATA_TYPE_COMPRESSED);

	/*
	 * Big-Endian: EXIF_BYTE_ORDER_MOTOROLA
	 * Little Endian: EXIF_BYTE_ORDER_INTEL
	 */
	exif_data_set_byte_order(data_, order_);

	setString(EXIF_IFD_EXIF, EXIF_TAG_EXIF_VERSION,
		  EXIF_FORMAT_UNDEFINED, "0231");

	/* Create the mandatory EXIF fields with default data. */
	exif_data_fix(data_);
}

Exif::~Exif()
{
	if (exifData_)
		free(exifData_);

	if (data_) {
		/*
		 * Reset thumbnail data to avoid getting double-freed by
		 * libexif. It is owned by the caller (i.e. PostProcessorJpeg).
		 */
		data_->data = nullptr;
		data_->size = 0;

		exif_data_unref(data_);
	}

	if (mem_)
		exif_mem_unref(mem_);
}

ExifEntry *Exif::createEntry(ExifIfd ifd, ExifTag tag)
{
	ExifContent *content = data_->ifd[ifd];
	ExifEntry *entry = exif_content_get_entry(content, tag);

	if (entry) {
		exif_entry_ref(entry);
		return entry;
	}

	entry = exif_entry_new_mem(mem_);
	if (!entry) {
		LOG(EXIF, Error) << "Failed to allocated new entry";
		valid_ = false;
		return nullptr;
	}

	exif_content_add_entry(content, entry);
	exif_entry_initialize(entry, tag);

	return entry;
}

ExifEntry *Exif::createEntry(ExifIfd ifd, ExifTag tag, ExifFormat format,
			     unsigned long components, unsigned int size)
{
	ExifContent *content = data_->ifd[ifd];

	/* Replace any existing entry with the same tag. */
	ExifEntry *existing = exif_content_get_entry(content, tag);
	exif_content_remove_entry(content, existing);

	ExifEntry *entry = exif_entry_new_mem(mem_);
	if (!entry) {
		LOG(EXIF, Error) << "Failed to allocated new entry";
		valid_ = false;
		return nullptr;
	}

	void *buffer = exif_mem_alloc(mem_, size);
	if (!buffer) {
		LOG(EXIF, Error) << "Failed to allocate buffer for variable entry";
		exif_mem_unref(mem_);
		valid_ = false;
		return nullptr;
	}

	entry->data = static_cast<unsigned char *>(buffer);
	entry->components = components;
	entry->format = format;
	entry->size = size;
	entry->tag = tag;

	exif_content_add_entry(content, entry);

	return entry;
}

void Exif::setByte(ExifIfd ifd, ExifTag tag, uint8_t item)
{
	ExifEntry *entry = createEntry(ifd, tag, EXIF_FORMAT_BYTE, 1, 1);
	if (!entry)
		return;

	entry->data[0] = item;
	exif_entry_unref(entry);
}

void Exif::setShort(ExifIfd ifd, ExifTag tag, uint16_t item)
{
	ExifEntry *entry = createEntry(ifd, tag);
	if (!entry)
		return;

	exif_set_short(entry->data, order_, item);
	exif_entry_unref(entry);
}

void Exif::setLong(ExifIfd ifd, ExifTag tag, uint32_t item)
{
	ExifEntry *entry = createEntry(ifd, tag);
	if (!entry)
		return;

	exif_set_long(entry->data, order_, item);
	exif_entry_unref(entry);
}

void Exif::setRational(ExifIfd ifd, ExifTag tag, ExifRational item)
{
	setRational(ifd, tag, { &item, 1 });
}

void Exif::setRational(ExifIfd ifd, ExifTag tag, Span<const ExifRational> items)
{
	ExifEntry *entry = createEntry(ifd, tag, EXIF_FORMAT_RATIONAL,
				       items.size(),
				       items.size() * sizeof(ExifRational));
	if (!entry)
		return;

	for (size_t i = 0; i < items.size(); i++)
		exif_set_rational(entry->data + i * sizeof(ExifRational),
				  order_, items[i]);
	exif_entry_unref(entry);
}

static const std::map<Exif::StringEncoding, std::array<uint8_t, 8>> stringEncodingCodes = {
	{ Exif::ASCII,     { 0x41, 0x53, 0x43, 0x49, 0x49, 0x00, 0x00, 0x00 } },
	{ Exif::Unicode,   { 0x55, 0x4e, 0x49, 0x43, 0x4f, 0x44, 0x45, 0x00 } },
};

void Exif::setString(ExifIfd ifd, ExifTag tag, ExifFormat format,
		     const std::string &item, StringEncoding encoding)
{
	std::string ascii;
	size_t length;
	const char *str;
	std::vector<uint8_t> buf;

	if (format == EXIF_FORMAT_ASCII) {
		ascii = utils::toAscii(item);
		str = ascii.c_str();

		/* Pad 1 extra byte to null-terminate the ASCII string. */
		length = ascii.length() + 1;
	} else {
		std::u16string u16str;

		auto encodingString = stringEncodingCodes.find(encoding);
		if (encodingString != stringEncodingCodes.end()) {
			buf = {
				encodingString->second.begin(),
				encodingString->second.end()
			};
		}

		switch (encoding) {
		case Unicode:
			u16str = utf8ToUtf16(item);

			buf.resize(8 + u16str.size() * 2);
			for (size_t i = 0; i < u16str.size(); i++) {
				if (order_ == EXIF_BYTE_ORDER_INTEL) {
					buf[8 + 2 * i] = u16str[i] & 0xff;
					buf[8 + 2 * i + 1] = (u16str[i] >> 8) & 0xff;
				} else {
					buf[8 + 2 * i] = (u16str[i] >> 8) & 0xff;
					buf[8 + 2 * i + 1] = u16str[i] & 0xff;
				}
			}

			break;

		case ASCII:
		case NoEncoding:
			buf.insert(buf.end(), item.begin(), item.end());
			break;
		}

		str = reinterpret_cast<const char *>(buf.data());

		/*
		 * Strings stored in different formats (EXIF_FORMAT_UNDEFINED)
		 * are not null-terminated.
		 */
		length = buf.size();
	}

	ExifEntry *entry = createEntry(ifd, tag, format, length, length);
	if (!entry)
		return;

	memcpy(entry->data, str, length);
	exif_entry_unref(entry);
}

void Exif::setMake(const std::string &make)
{
	setString(EXIF_IFD_0, EXIF_TAG_MAKE, EXIF_FORMAT_ASCII, make);
}

void Exif::setModel(const std::string &model)
{
	setString(EXIF_IFD_0, EXIF_TAG_MODEL, EXIF_FORMAT_ASCII, model);
}

void Exif::setSize(const Size &size)
{
	setLong(EXIF_IFD_EXIF, EXIF_TAG_PIXEL_Y_DIMENSION, size.height);
	setLong(EXIF_IFD_EXIF, EXIF_TAG_PIXEL_X_DIMENSION, size.width);
}

void Exif::setTimestamp(time_t timestamp, std::chrono::milliseconds msec)
{
	struct tm tm;
	localtime_r(&timestamp, &tm);

	char str[20];
	strftime(str, sizeof(str), "%Y:%m:%d %H:%M:%S", &tm);
	std::string ts(str);

	setString(EXIF_IFD_0, EXIF_TAG_DATE_TIME, EXIF_FORMAT_ASCII, ts);
	setString(EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_ORIGINAL, EXIF_FORMAT_ASCII, ts);
	setString(EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_DIGITIZED, EXIF_FORMAT_ASCII, ts);

	/* Query and set timezone information if available. */
	int r = strftime(str, sizeof(str), "%z", &tm);
	if (r <= 0)
		return;

	std::string tz(str);
	tz.insert(3, 1, ':');
	setString(EXIF_IFD_EXIF,
		  static_cast<ExifTag>(_ExifTag::OFFSET_TIME),
		  EXIF_FORMAT_ASCII, tz);
	setString(EXIF_IFD_EXIF,
		  static_cast<ExifTag>(_ExifTag::OFFSET_TIME_ORIGINAL),
		  EXIF_FORMAT_ASCII, tz);
	setString(EXIF_IFD_EXIF,
		  static_cast<ExifTag>(_ExifTag::OFFSET_TIME_DIGITIZED),
		  EXIF_FORMAT_ASCII, tz);

	std::stringstream sstr;
	sstr << std::setfill('0') << std::setw(3) << msec.count();
	std::string subsec = sstr.str();

	setString(EXIF_IFD_EXIF, EXIF_TAG_SUB_SEC_TIME,
		  EXIF_FORMAT_ASCII, subsec);
	setString(EXIF_IFD_EXIF, EXIF_TAG_SUB_SEC_TIME_ORIGINAL,
		  EXIF_FORMAT_ASCII, subsec);
	setString(EXIF_IFD_EXIF, EXIF_TAG_SUB_SEC_TIME_DIGITIZED,
		  EXIF_FORMAT_ASCII, subsec);
}

void Exif::setGPSDateTimestamp(time_t timestamp)
{
	struct tm tm;
	gmtime_r(&timestamp, &tm);

	char str[11];
	strftime(str, sizeof(str), "%Y:%m:%d", &tm);
	std::string tsStr(str);

	setString(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_DATE_STAMP),
		  EXIF_FORMAT_ASCII, tsStr);

	/* Set GPS_TIME_STAMP */
	ExifRational ts[] = {
		{ static_cast<ExifLong>(tm.tm_hour), 1 },
		{ static_cast<ExifLong>(tm.tm_min),  1 },
		{ static_cast<ExifLong>(tm.tm_sec),  1 },
	};

	setRational(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_TIME_STAMP),
		    ts);
}

std::tuple<int, int, int> Exif::degreesToDMS(double decimalDegrees)
{
	int degrees = std::trunc(decimalDegrees);
	double minutes = std::abs((decimalDegrees - degrees) * 60);
	double seconds = (minutes - std::trunc(minutes)) * 60;

	return { degrees, std::trunc(minutes), std::round(seconds) };
}

void Exif::setGPSDMS(ExifIfd ifd, ExifTag tag, int deg, int min, int sec)
{
	ExifRational coords[] = {
		{ static_cast<ExifLong>(deg), 1 },
		{ static_cast<ExifLong>(min), 1 },
		{ static_cast<ExifLong>(sec), 1 },
	};

	setRational(ifd, tag, coords);
}

/*
 * \brief Set GPS location (lat, long, alt)
 * \param[in] coords Pointer to coordinates latitude, longitude, and altitude,
 * first two in degrees, the third in meters
 */
void Exif::setGPSLocation(const double *coords)
{
	int deg, min, sec;

	std::tie<int, int, int>(deg, min, sec) = degreesToDMS(coords[0]);
	setString(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_LATITUDE_REF),
		  EXIF_FORMAT_ASCII, deg >= 0 ? "N" : "S");
	setGPSDMS(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_LATITUDE),
		  std::abs(deg), min, sec);

	std::tie<int, int, int>(deg, min, sec) = degreesToDMS(coords[1]);
	setString(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_LONGITUDE_REF),
		  EXIF_FORMAT_ASCII, deg >= 0 ? "E" : "W");
	setGPSDMS(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_LONGITUDE),
		  std::abs(deg), min, sec);

	setByte(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_ALTITUDE_REF),
		coords[2] >= 0 ? 0 : 1);
	setRational(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_ALTITUDE),
		    ExifRational{ static_cast<ExifLong>(std::abs(coords[2])), 1 });
}

void Exif::setGPSMethod(const std::string &method)
{
	setString(EXIF_IFD_GPS, static_cast<ExifTag>(EXIF_TAG_GPS_PROCESSING_METHOD),
		  EXIF_FORMAT_UNDEFINED, method, NoEncoding);
}

void Exif::setOrientation(int orientation)
{
	int value;
	switch (orientation) {
	case 0:
	default:
		value = 1;
		break;
	case 90:
		value = 6;
		break;
	case 180:
		value = 3;
		break;
	case 270:
		value = 8;
		break;
	}

	setShort(EXIF_IFD_0, EXIF_TAG_ORIENTATION, value);
}

void Exif::setThumbnail(std::vector<unsigned char> &&thumbnail,
			Compression compression)
{
	thumbnailData_ = std::move(thumbnail);

	data_->data = thumbnailData_.data();
	data_->size = thumbnailData_.size();

	setShort(EXIF_IFD_0, EXIF_TAG_COMPRESSION, compression);
}

void Exif::setFocalLength(float length)
{
	ExifRational rational = { static_cast<ExifLong>(length * 1000), 1000 };
	setRational(EXIF_IFD_EXIF, EXIF_TAG_FOCAL_LENGTH, rational);
}

void Exif::setExposureTime(uint64_t nsec)
{
	ExifRational rational = { static_cast<ExifLong>(nsec), 1000000000 };
	setRational(EXIF_IFD_EXIF, EXIF_TAG_EXPOSURE_TIME, rational);
}

void Exif::setAperture(float size)
{
	ExifRational rational = { static_cast<ExifLong>(size * 10000), 10000 };
	setRational(EXIF_IFD_EXIF, EXIF_TAG_FNUMBER, rational);
}

void Exif::setISO(uint16_t iso)
{
	setShort(EXIF_IFD_EXIF, EXIF_TAG_ISO_SPEED_RATINGS, iso);
}

void Exif::setFlash(Flash flash)
{
	setShort(EXIF_IFD_EXIF, EXIF_TAG_FLASH, static_cast<ExifShort>(flash));
}

void Exif::setWhiteBalance(WhiteBalance wb)
{
	setShort(EXIF_IFD_EXIF, EXIF_TAG_WHITE_BALANCE, static_cast<ExifShort>(wb));
}

/**
 * \brief Convert UTF-8 string to UTF-16 string
 * \param[in] str String to convert
 *
 * \return \a str in UTF-16
 */
std::u16string Exif::utf8ToUtf16(const std::string &str)
{
	mbstate_t state{};
	char16_t c16;
	const char *ptr = str.data();
	const char *end = ptr + str.size();

	std::u16string ret;
	while (size_t rc = mbrtoc16(&c16, ptr, end - ptr + 1, &state)) {
		if (rc == static_cast<size_t>(-2) ||
		    rc == static_cast<size_t>(-1))
			break;

		ret.push_back(c16);

		if (rc > 0)
			ptr += rc;
	}

	return ret;
}

[[nodiscard]] int Exif::generate()
{
	if (exifData_) {
		free(exifData_);
		exifData_ = nullptr;
	}

	if (!valid_) {
		LOG(EXIF, Error) << "Generated EXIF data is invalid";
		return -1;
	}

	exif_data_save_data(data_, &exifData_, &size_);

	LOG(EXIF, Debug) << "Created EXIF instance (" << size_ << " bytes)";

	return 0;
}
