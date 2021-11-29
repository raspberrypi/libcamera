/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * shared_fd.cpp - SharedFD test
 */

#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/shared_fd.h>
#include <libcamera/base/utils.h>

#include "test.h"

using namespace libcamera;
using namespace std;

class SharedFDTest : public Test
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
		/* Test creating empty SharedFD. */
		desc1_ = new SharedFD();

		if (desc1_->get() != -1) {
			std::cout << "Failed fd numerical check (default constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;

		/*
		 * Test creating SharedFD by copying numerical file
		 * descriptor.
		 */
		desc1_ = new SharedFD(fd_);
		if (desc1_->get() == fd_) {
			std::cout << "Failed fd numerical check (lvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_) || !isValidFd(desc1_->get())) {
			std::cout << "Failed fd validity after construction (lvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		int fd = desc1_->get();

		delete desc1_;
		desc1_ = nullptr;

		if (!isValidFd(fd_) || isValidFd(fd)) {
			std::cout << "Failed fd validity after destruction (lvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		/*
		 * Test creating SharedFD by taking ownership of
		 * numerical file descriptor.
		 */
		int dupFd = dup(fd_);
		int dupFdCopy = dupFd;

		desc1_ = new SharedFD(std::move(dupFd));
		if (desc1_->get() != dupFdCopy) {
			std::cout << "Failed fd numerical check (rvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		if (dupFd != -1 || !isValidFd(fd_) || !isValidFd(desc1_->get())) {
			std::cout << "Failed fd validity after construction (rvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		fd = desc1_->get();

		delete desc1_;
		desc1_ = nullptr;

		if (!isValidFd(fd_) || isValidFd(fd)) {
			std::cout << "Failed fd validity after destruction (rvalue ref constructor)"
				  << std::endl;
			return TestFail;
		}

		/* Test creating SharedFD from other SharedFD. */
		desc1_ = new SharedFD(fd_);
		desc2_ = new SharedFD(*desc1_);

		if (desc1_->get() == fd_ || desc2_->get() == fd_ ||
		    desc1_->get() != desc2_->get()) {
			std::cout << "Failed fd numerical check (copy constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc1_->get()) || !isValidFd(desc2_->get())) {
			std::cout << "Failed fd validity after construction (copy constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;

		if (!isValidFd(desc2_->get())) {
			std::cout << "Failed fd validity after destruction (copy constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc2_;
		desc2_ = nullptr;

		/* Test creating SharedFD by taking over other SharedFD. */
		desc1_ = new SharedFD(fd_);
		fd = desc1_->get();
		desc2_ = new SharedFD(std::move(*desc1_));

		if (desc1_->get() != -1 || desc2_->get() != fd) {
			std::cout << "Failed fd numerical check (move constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc2_->get())) {
			std::cout << "Failed fd validity after construction (move constructor)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;
		delete desc2_;
		desc2_ = nullptr;

		/* Test creating SharedFD by copy assignment. */
		desc1_ = new SharedFD();
		desc2_ = new SharedFD(fd_);

		fd = desc2_->get();
		*desc1_ = *desc2_;

		if (desc1_->get() != fd || desc2_->get() != fd) {
			std::cout << "Failed fd numerical check (copy assignment)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc1_->get()) || !isValidFd(desc2_->get())) {
			std::cout << "Failed fd validity after construction (copy assignment)"
				  << std::endl;
			return TestFail;
		}

		delete desc1_;
		desc1_ = nullptr;
		delete desc2_;
		desc2_ = nullptr;

		/* Test creating SharedFD by move assignment. */
		desc1_ = new SharedFD();
		desc2_ = new SharedFD(fd_);

		fd = desc2_->get();
		*desc1_ = std::move(*desc2_);

		if (desc1_->get() != fd || desc2_->get() != -1) {
			std::cout << "Failed fd numerical check (move assignment)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(desc1_->get())) {
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
	SharedFD *desc1_, *desc2_;
};

TEST_REGISTER(SharedFDTest)
