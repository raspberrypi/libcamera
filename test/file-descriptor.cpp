/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * file_descriptor.cpp - FileDescriptor test
 */

#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/file_descriptor.h>

#include "libcamera/internal/utils.h"

#include "test.h"

using namespace libcamera;
using namespace std;

class FileDescriptorTest : public Test
{
protected:
	int init()
	{
		desc1_ = nullptr;
		desc2_ = nullptr;

		fd_ = open("/tmp", O_TMPFILE | O_RDWR, S_IRUSR | S_IWUSR);
		if (fd_ < 0)
			return TestFail;

		/* Cache inode number of temp file. */
		struct stat s;
		if (fstat(fd_, &s))
			return TestFail;

		inodeNr_ = s.st_ino;

		return 0;
	}

	int run()
	{
		/* Test creating empty FileDescriptor. */
		desc1_ = new FileDescriptor();

		if (desc1_->fd() != -1) {
			std::cout << "Failed fd numerical check (default constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;

		/*
		 * Test creating FileDescriptor by copying numerical file
		 * descriptor.
		 */
		desc1_ = new FileDescriptor(fd_);
		if (desc1_->fd() == fd_) {
			std::cout << "Failed fd numerical check (lvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_) || !isValidFd(desc1_->fd())) {
			std::cout << "Failed fd validity after construction (lvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		int fd = desc1_->fd();

		delete desc1_;
		desc1_ = nullptr;

		if (!isValidFd(fd_) || isValidFd(fd)) {
			std::cout << "Failed fd validity after destruction (lvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		/*
		 * Test creating FileDescriptor by taking ownership of
		 * numerical file descriptor.
		 */
		int dupFd = dup(fd_);
		int dupFdCopy = dupFd;

		desc1_ = new FileDescriptor(std::move(dupFd));
		if (desc1_->fd() != dupFdCopy) {
			std::cout << "Failed fd numerical check (rvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		if (dupFd != -1 || !isValidFd(fd_) || !isValidFd(desc1_->fd())) {
			std::cout << "Failed fd validity after construction (rvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		fd = desc1_->fd();

		delete desc1_;
		desc1_ = nullptr;

		if (!isValidFd(fd_) || isValidFd(fd)) {
			std::cout << "Failed fd validity after destruction (rvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		/* Test creating FileDescriptor from other FileDescriptor. */
		desc1_ = new FileDescriptor(fd_);
		desc2_ = new FileDescriptor(*desc1_);

		if (desc1_->fd() == fd_ || desc2_->fd() == fd_ || desc1_->fd() != desc2_->fd()) {
			std::cout << "Failed fd numerical check (copy constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc1_->fd()) || !isValidFd(desc2_->fd())) {
			std::cout << "Failed fd validity after construction (copy constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;

		if (!isValidFd(desc2_->fd())) {
			std::cout << "Failed fd validity after destruction (copy constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc2_;
		desc2_ = nullptr;

		/* Test creating FileDescriptor by taking over other FileDescriptor. */
		desc1_ = new FileDescriptor(fd_);
		fd = desc1_->fd();
		desc2_ = new FileDescriptor(std::move(*desc1_));

		if (desc1_->fd() != -1 || desc2_->fd() != fd) {
			std::cout << "Failed fd numerical check (move constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc2_->fd())) {
			std::cout << "Failed fd validity after construction (move constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;
		delete desc2_;
		desc2_ = nullptr;

		/* Test creating FileDescriptor by copy assignment. */
		desc1_ = new FileDescriptor();
		desc2_ = new FileDescriptor(fd_);

		fd = desc2_->fd();
		*desc1_ = *desc2_;

		if (desc1_->fd() != fd || desc2_->fd() != fd) {
			std::cout << "Failed fd numerical check (copy assignment)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc1_->fd()) || !isValidFd(desc2_->fd())) {
			std::cout << "Failed fd validity after construction (copy assignment)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;
		delete desc2_;
		desc2_ = nullptr;

		/* Test creating FileDescriptor by move assignment. */
		desc1_ = new FileDescriptor();
		desc2_ = new FileDescriptor(fd_);

		fd = desc2_->fd();
		*desc1_ = std::move(*desc2_);

		if (desc1_->fd() != fd || desc2_->fd() != -1) {
			std::cout << "Failed fd numerical check (move assignment)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc1_->fd())) {
			std::cout << "Failed fd validity after construction (move assignment)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;
		delete desc2_;
		desc2_ = nullptr;

		return TestPass;
	}

	void cleanup()
	{
		delete desc2_;
		delete desc1_;

		if (fd_ > 0)
			close(fd_);
	}

private:
	bool isValidFd(int fd)
	{
		struct stat s;
		if (fstat(fd, &s))
			return false;

		/* Check that inode number matches cached temp file. */
		return s.st_ino == inodeNr_;
	}

	int fd_;
	ino_t inodeNr_;
	FileDescriptor *desc1_, *desc2_;
};

TEST_REGISTER(FileDescriptorTest)
