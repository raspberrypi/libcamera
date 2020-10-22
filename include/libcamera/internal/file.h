/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * file.h - File I/O operations
 */
#ifndef __LIBCAMERA_INTERNAL_FILE_H__
#define __LIBCAMERA_INTERNAL_FILE_H__

#include <map>
#include <string>
#include <sys/types.h>

#include <libcamera/class.h>
#include <libcamera/span.h>

namespace libcamera {

class File
{
public:
	enum MapFlag {
		MapNoOption = 0,
		MapPrivate = (1 << 0),
	};

	enum OpenMode {
		NotOpen = 0,
		ReadOnly = (1 << 0),
		WriteOnly = (1 << 1),
		ReadWrite = ReadOnly | WriteOnly,
	};

	File(const std::string &name);
	File();
	~File();

	const std::string &fileName() const { return name_; }
	void setFileName(const std::string &name);
	bool exists() const;

	bool open(OpenMode mode);
	bool isOpen() const { return fd_ != -1; }
	OpenMode openMode() const { return mode_; }
	void close();

	int error() const { return error_; }
	ssize_t size() const;

	off_t pos() const;
	off_t seek(off_t pos);

	ssize_t read(const Span<uint8_t> &data);
	ssize_t write(const Span<const uint8_t> &data);

	Span<uint8_t> map(off_t offset = 0, ssize_t size = -1,
			  MapFlag flags = MapNoOption);
	bool unmap(uint8_t *addr);

	static bool exists(const std::string &name);

private:
	LIBCAMERA_DISABLE_COPY(File)

	void unmapAll();

	std::string name_;
	int fd_;
	OpenMode mode_;

	int error_;
	std::map<void *, size_t> maps_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_FILE_H__ */
