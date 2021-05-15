/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_metadata.h - libcamera Android Camera Metadata Helper
 */
#ifndef __ANDROID_CAMERA_METADATA_H__
#define __ANDROID_CAMERA_METADATA_H__

#include <stdint.h>
#include <vector>

#include <system/camera_metadata.h>

class CameraMetadata
{
public:
	CameraMetadata();
	CameraMetadata(size_t entryCapacity, size_t dataCapacity);
	CameraMetadata(const camera_metadata_t *metadata);
	CameraMetadata(const CameraMetadata &other);
	~CameraMetadata();

	CameraMetadata &operator=(const CameraMetadata &other);

	bool isValid() const { return valid_; }
	bool getEntry(uint32_t tag, camera_metadata_ro_entry_t *entry) const;

	template<typename T,
		 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
	bool addEntry(uint32_t tag, const T &data)
	{
		return addEntry(tag, &data, 1, sizeof(T));
	}

	template<typename T, size_t size>
	bool addEntry(uint32_t tag, const T (&data)[size])
	{
		return addEntry(tag, data, size, sizeof(T));
	}

	template<typename S,
		 typename T = typename S::value_type>
	bool addEntry(uint32_t tag, const S &data)
	{
		return addEntry(tag, data.data(), data.size(), sizeof(T));
	}

	template<typename T>
	bool addEntry(uint32_t tag, const T *data, size_t count)
	{
		return addEntry(tag, data, count, sizeof(T));
	}

	template<typename T>
	bool updateEntry(uint32_t tag, const T &data)
	{
		return updateEntry(tag, &data, 1);
	}

	template<typename T, size_t size>
	bool updateEntry(uint32_t tag, const T (&data)[size])
	{
		return updateEntry(tag, data, size, sizeof(T));
	}

	template<typename S,
		 typename T = typename S::value_type>
	bool updateEntry(uint32_t tag, const S &data)
	{
		return updateEntry(tag, data.data(), data.size());
	}

	bool updateEntry(uint32_t tag, const void *data, size_t count);

	camera_metadata_t *get();
	const camera_metadata_t *get() const;

private:
	bool resize(size_t count, size_t size);
	bool addEntry(uint32_t tag, const void *data, size_t count,
		      size_t elementSize);

	camera_metadata_t *metadata_;
	bool valid_;
};

#endif /* __ANDROID_CAMERA_METADATA_H__ */
