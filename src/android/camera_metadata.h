/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Android Camera Metadata Helper
 */

#pragma once

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

	std::tuple<size_t, size_t> usage() const;
	bool resized() const { return resized_; }

	bool isValid() const { return valid_; }
	bool getEntry(uint32_t tag, camera_metadata_ro_entry_t *entry) const;

	template<typename T> bool entryContains(uint32_t tag, T value) const;

	bool hasEntry(uint32_t tag) const;

	template<typename T,
		 std::enable_if_t<std::is_arithmetic_v<T> ||
				  std::is_enum_v<T>> * = nullptr>
	bool setEntry(uint32_t tag, const T &data)
	{
		if (hasEntry(tag))
			return updateEntry(tag, &data, 1, sizeof(T));
		else
			return addEntry(tag, &data, 1, sizeof(T));
	}

	template<typename T,
		 std::enable_if_t<std::is_arithmetic_v<T> ||
				  std::is_enum_v<T>> * = nullptr>
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
		return updateEntry(tag, &data, 1, sizeof(T));
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
		return updateEntry(tag, data.data(), data.size(), sizeof(T));
	}

	template<typename T>
	bool updateEntry(uint32_t tag, const T *data, size_t count)
	{
		return updateEntry(tag, data, count, sizeof(T));
	}

	camera_metadata_t *getMetadata();
	const camera_metadata_t *getMetadata() const;

private:
	bool resize(size_t count, size_t size);
	bool addEntry(uint32_t tag, const void *data, size_t count,
		      size_t elementSize);
	bool updateEntry(uint32_t tag, const void *data, size_t count,
			 size_t elementSize);

	camera_metadata_t *metadata_;
	bool valid_;
	bool resized_;
};
