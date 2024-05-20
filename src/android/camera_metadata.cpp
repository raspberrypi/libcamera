/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Android Camera Metadata Helper
 */

#include "camera_metadata.h"

#include <libcamera/base/log.h>

using namespace libcamera;

LOG_DEFINE_CATEGORY(CameraMetadata)

CameraMetadata::CameraMetadata()
	: metadata_(nullptr), valid_(false), resized_(false)
{
}

CameraMetadata::CameraMetadata(size_t entryCapacity, size_t dataCapacity)
	: resized_(false)
{
	metadata_ = allocate_camera_metadata(entryCapacity, dataCapacity);
	valid_ = metadata_ != nullptr;
}

CameraMetadata::CameraMetadata(const camera_metadata_t *metadata)
	: resized_(false)
{
	metadata_ = clone_camera_metadata(metadata);
	valid_ = metadata_ != nullptr;
}

CameraMetadata::CameraMetadata(const CameraMetadata &other)
	: CameraMetadata(other.getMetadata())
{
}

CameraMetadata::~CameraMetadata()
{
	if (metadata_)
		free_camera_metadata(metadata_);
}

CameraMetadata &CameraMetadata::operator=(const CameraMetadata &other)
{
	if (this == &other)
		return *this;

	if (metadata_)
		free_camera_metadata(metadata_);

	metadata_ = clone_camera_metadata(other.getMetadata());
	valid_ = metadata_ != nullptr;

	return *this;
}

std::tuple<size_t, size_t> CameraMetadata::usage() const
{
	size_t currentEntryCount = get_camera_metadata_entry_count(metadata_);
	size_t currentDataCount = get_camera_metadata_data_count(metadata_);

	return { currentEntryCount, currentDataCount };
}

bool CameraMetadata::getEntry(uint32_t tag, camera_metadata_ro_entry_t *entry) const
{
	if (find_camera_metadata_ro_entry(metadata_, tag, entry))
		return false;

	return true;
}

/*
 * \brief Resize the metadata container, if necessary
 * \param[in] count Number of entries to add to the container
 * \param[in] size Total size of entries to add, in bytes
 * \return True if resize was successful or unnecessary, false otherwise
 */
bool CameraMetadata::resize(size_t count, size_t size)
{
	if (!valid_)
		return false;

	if (!count && !size)
		return true;

	size_t currentEntryCount = get_camera_metadata_entry_count(metadata_);
	size_t currentEntryCapacity = get_camera_metadata_entry_capacity(metadata_);
	size_t newEntryCapacity = currentEntryCapacity < currentEntryCount + count ?
				  currentEntryCapacity * 2 : currentEntryCapacity;

	size_t currentDataCount = get_camera_metadata_data_count(metadata_);
	size_t currentDataCapacity = get_camera_metadata_data_capacity(metadata_);
	size_t newDataCapacity = currentDataCapacity < currentDataCount + size ?
				 currentDataCapacity * 2 : currentDataCapacity;

	if (newEntryCapacity > currentEntryCapacity ||
	    newDataCapacity > currentDataCapacity) {
		camera_metadata_t *oldMetadata = metadata_;
		metadata_ = allocate_camera_metadata(newEntryCapacity, newDataCapacity);
		if (!metadata_) {
			metadata_ = oldMetadata;
			return false;
		}

		LOG(CameraMetadata, Info)
			<< "Resized: old entry capacity " << currentEntryCapacity
			<< ", old data capacity " << currentDataCapacity
			<< ", new entry capacity " << newEntryCapacity
			<< ", new data capacity " << newDataCapacity;

		append_camera_metadata(metadata_, oldMetadata);
		free_camera_metadata(oldMetadata);

		resized_ = true;
	}

	return true;
}

template<> bool CameraMetadata::entryContains(uint32_t tag, uint8_t value) const
{
	camera_metadata_ro_entry_t entry;
	if (!getEntry(tag, &entry))
		return false;

	for (unsigned int i = 0; i < entry.count; i++) {
		if (entry.data.u8[i] == value)
			return true;
	}

	return false;
}

bool CameraMetadata::hasEntry(uint32_t tag) const
{
	camera_metadata_ro_entry_t entry;
	return getEntry(tag, &entry);
}

bool CameraMetadata::addEntry(uint32_t tag, const void *data, size_t count,
			      size_t elementSize)
{
	if (!valid_)
		return false;

	if (!resize(1, count * elementSize)) {
		LOG(CameraMetadata, Error) << "Failed to resize";
		valid_ = false;
		return false;
	}

	if (!add_camera_metadata_entry(metadata_, tag, data, count))
		return true;

	const char *name = get_camera_metadata_tag_name(tag);
	if (name)
		LOG(CameraMetadata, Error)
			<< "Failed to add tag " << name;
	else
		LOG(CameraMetadata, Error)
			<< "Failed to add unknown tag " << tag;

	valid_ = false;

	return false;
}

bool CameraMetadata::updateEntry(uint32_t tag, const void *data, size_t count,
				 size_t elementSize)
{
	if (!valid_)
		return false;

	camera_metadata_entry_t entry;
	int ret = find_camera_metadata_entry(metadata_, tag, &entry);
	if (ret) {
		const char *name = get_camera_metadata_tag_name(tag);
		LOG(CameraMetadata, Error)
			<< "Failed to update tag "
			<< (name ? name : "<unknown>") << ": not present";
		return false;
	}

	if (camera_metadata_type_size[entry.type] != elementSize) {
		const char *name = get_camera_metadata_tag_name(tag);
		LOG(CameraMetadata, Fatal)
			<< "Invalid element size for tag "
			<< (name ? name : "<unknown>");
		return false;
	}

	size_t oldSize =
		calculate_camera_metadata_entry_data_size(entry.type,
							  entry.count);
	size_t newSize =
		calculate_camera_metadata_entry_data_size(entry.type,
							  count);
	size_t sizeIncrement = newSize - oldSize > 0 ? newSize - oldSize : 0;
	if (!resize(0, sizeIncrement)) {
		LOG(CameraMetadata, Error) << "Failed to resize";
		valid_ = false;
		return false;
	}

	ret = update_camera_metadata_entry(metadata_, entry.index, data,
					   count, nullptr);
	if (!ret)
		return true;

	const char *name = get_camera_metadata_tag_name(tag);
	LOG(CameraMetadata, Error)
		<< "Failed to update tag " << (name ? name : "<unknown>");

	valid_ = false;

	return false;
}

camera_metadata_t *CameraMetadata::getMetadata()
{
	return valid_ ? metadata_ : nullptr;
}

const camera_metadata_t *CameraMetadata::getMetadata() const
{
	return valid_ ? metadata_ : nullptr;
}
