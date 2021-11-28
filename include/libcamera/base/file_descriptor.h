/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * file_descriptor.h - File descriptor wrapper
 */

#pragma once

#include <memory>
#include <sys/types.h>

namespace libcamera {

class FileDescriptor final
{
public:
	explicit FileDescriptor(const int &fd = -1);
	explicit FileDescriptor(int &&fd);
	FileDescriptor(const FileDescriptor &other);
	FileDescriptor(FileDescriptor &&other);
	~FileDescriptor();

	FileDescriptor &operator=(const FileDescriptor &other);
	FileDescriptor &operator=(FileDescriptor &&other);

	bool isValid() const { return fd_ != nullptr; }
	int fd() const { return fd_ ? fd_->fd() : -1; }
	FileDescriptor dup() const;

	ino_t inode() const;

private:
	class Descriptor
	{
	public:
		Descriptor(int fd, bool duplicate);
		~Descriptor();

		int fd() const { return fd_; }

	private:
		int fd_;
	};

	std::shared_ptr<Descriptor> fd_;
};

} /* namespace libcamera */
