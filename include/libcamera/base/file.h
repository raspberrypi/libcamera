/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * File I/O operations
 */

#pragma once

#include <map>
#include <stdint.h>
#include <string>
#include <sys/types.h>

#include <libcamera/base/private.h>

#include <libcamera/base/class.h>
#include <libcamera/base/flags.h>
#include <libcamera/base/span.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

class File
{
public:
	enum class MapFlag {
		NoOption = 0,
		Private = (1 << 0),
	};

	using MapFlags = Flags<MapFlag>;

	enum class OpenModeFlag {
		NotOpen = 0,
		ReadOnly = (1 << 0),
		WriteOnly = (1 << 1),
		ReadWrite = ReadOnly | WriteOnly,
	};

	using OpenMode = Flags<OpenModeFlag>;

	File(const std::string &name);
	File();
	~File();

	const std::string &fileName() const { return name_; }
	void setFileName(const std::string &name);
	bool exists() const;

	bool open(OpenMode mode);
	bool isOpen() const { return fd_.isValid(); }
	OpenMode openMode() const { return mode_; }
	void close();

	int error() const { return error_; }
	ssize_t size() const;

	off_t pos() const;
	off_t seek(off_t pos);

	ssize_t read(const Span<uint8_t> &data);
	ssize_t write(const Span<const uint8_t> &data);

	Span<uint8_t> map(off_t offset = 0, ssize_t size = -1,
			  MapFlags flags = MapFlag::NoOption);
	bool unmap(uint8_t *addr);

	static bool exists(const std::string &name);

private:
	LIBCAMERA_DISABLE_COPY(File)

	void unmapAll();

	std::string name_;
	UniqueFD fd_;
	OpenMode mode_;

	int error_;
	std::map<void *, size_t> maps_;
};

LIBCAMERA_FLAGS_ENABLE_OPERATORS(File::MapFlag)
LIBCAMERA_FLAGS_ENABLE_OPERATORS(File::OpenModeFlag)

} /* namespace libcamera */
