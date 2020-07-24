/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_metadata.cpp - libcamera Android Camera Metadata Helper
 */

#include "camera_metadata.h"

#include "libcamera/internal/log.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(CameraMetadata);

CameraMetadata::CameraMetadata(size_t entryCapacity, size_t dataCapacity)
{
	metadata_ = allocate_camera_metadata(entryCapacity, dataCapacity);
	valid_ = metadata_ != nullptr;
}

CameraMetadata::~CameraMetadata()
{
	if (metadata_)
		free_camera_metadata(metadata_);
}

bool CameraMetadata::addEntry(uint32_t tag, const void *data, size_t count)
{
	if (!valid_)
		return false;

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

bool CameraMetadata::updateEntry(uint32_t tag, const void *data, size_t count)
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

	ret = update_camera_metadata_entry(metadata_, entry.index, data,
					   count, nullptr);
	if (ret) {
		const char *name = get_camera_metadata_tag_name(tag);
		LOG(CameraMetadata, Error)
			<< "Failed to update tag " << (name ? name : "<unknown>");
		return false;
	}

	return true;
}

camera_metadata_t *CameraMetadata::get()
{
	return valid_ ? metadata_ : nullptr;
}

const camera_metadata_t *CameraMetadata::get() const
{
	return valid_ ? metadata_ : nullptr;
}
