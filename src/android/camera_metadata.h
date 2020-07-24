/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_metadata.h - libcamera Android Camera Metadata Helper
 */
#ifndef __ANDROID_CAMERA_METADATA_H__
#define __ANDROID_CAMERA_METADATA_H__

#include <stdint.h>

#include <system/camera_metadata.h>

class CameraMetadata
{
public:
	CameraMetadata(size_t entryCapacity, size_t dataCapacity);
	~CameraMetadata();

	bool isValid() const { return valid_; }
	bool addEntry(uint32_t tag, const void *data, size_t data_count);
	bool updateEntry(uint32_t tag, const void *data, size_t data_count);

	camera_metadata_t *get();
	const camera_metadata_t *get() const;

private:
	camera_metadata_t *metadata_;
	bool valid_;
};

#endif /* __ANDROID_CAMERA_METADATA_H__ */
