/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * exif.cpp - EXIF tag creation using libexif
 */

#include "exif.h"

#include "libcamera/internal/log.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(EXIF)

/*
 * The Exif class should be instantiated and specific properties set
 * through the exposed public API.
 *
 * Once all desired properties have been set, the user shall call
 * generate() to process the entries and generate the Exif data.
 *
 * Calls to generate() must check the return code to determine if any error
 * occurred during the construction of the Exif data, and if successful the
 * data can be obtained using the data() method.
 */
Exif::Exif()
	: valid_(false), data_(nullptr), exifData_(0), size_(0)
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
	exif_data_set_byte_order(data_, EXIF_BYTE_ORDER_INTEL);

	/* Create the mandatory EXIF fields with default data. */
	exif_data_fix(data_);
}

Exif::~Exif()
{
	if (exifData_)
		free(exifData_);

	if (data_)
		exif_data_unref(data_);

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

void Exif::setShort(ExifIfd ifd, ExifTag tag, uint16_t item)
{
	ExifEntry *entry = createEntry(ifd, tag);
	if (!entry)
		return;

	exif_set_short(entry->data, EXIF_BYTE_ORDER_INTEL, item);
	exif_entry_unref(entry);
}

void Exif::setLong(ExifIfd ifd, ExifTag tag, uint32_t item)
{
	ExifEntry *entry = createEntry(ifd, tag);
	if (!entry)
		return;

	exif_set_long(entry->data, EXIF_BYTE_ORDER_INTEL, item);
	exif_entry_unref(entry);
}

void Exif::setRational(ExifIfd ifd, ExifTag tag, ExifRational item)
{
	ExifEntry *entry = createEntry(ifd, tag);
	if (!entry)
		return;

	exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, item);
	exif_entry_unref(entry);
}

void Exif::setString(ExifIfd ifd, ExifTag tag, ExifFormat format, const std::string &item)
{
	/* Pad 1 extra byte for null-terminated string in ASCII format. */
	size_t length = format == EXIF_FORMAT_ASCII ?
			item.length() + 1 : item.length();

	ExifEntry *entry = createEntry(ifd, tag, format, length, length);
	if (!entry)
		return;

	memcpy(entry->data, item.c_str(), length);
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

void Exif::setTimestamp(time_t timestamp)
{
	struct tm tm;
	localtime_r(&timestamp, &tm);

	char str[20];
	strftime(str, sizeof(str), "%Y:%m:%d %H:%M:%S", &tm);
	std::string ts(str);

	setString(EXIF_IFD_0, EXIF_TAG_DATE_TIME, EXIF_FORMAT_ASCII, ts);
	setString(EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_ORIGINAL, EXIF_FORMAT_ASCII, ts);
	setString(EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_DIGITIZED, EXIF_FORMAT_ASCII, ts);
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
		value = 8;
		break;
	case 180:
		value = 3;
		break;
	case 270:
		value = 6;
		break;
	}

	setShort(EXIF_IFD_0, EXIF_TAG_ORIENTATION, value);
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
