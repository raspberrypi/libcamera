/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * unique-fd.cpp - UniqueFD test
 */

#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/unique_fd.h>
#include <libcamera/base/utils.h>

#include "test.h"

using namespace libcamera;
using namespace std;

class UniqueFDTest : public Test
{
protected:
	int init() override
	{
		return createFd();
	}

	int run() override
	{
		/* Test creating empty UniqueFD. */
		UniqueFD fd;

		if (fd.isValid() || fd.get() != -1) {
			std::cout << "Failed fd check (default constructor)"
				  << std::endl;
			return TestFail;
		}

		/* Test creating UniqueFD from numerical file descriptor. */
		UniqueFD fd2(fd_);
		if (!fd2.isValid() || fd2.get() != fd_) {
			std::cout << "Failed fd check (fd constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_)) {
			std::cout << "Failed fd validity (fd constructor)"
				  << std::endl;
			return TestFail;
		}

		/* Test move constructor. */
		UniqueFD fd3(std::move(fd2));
		if (!fd3.isValid() || fd3.get() != fd_) {
			std::cout << "Failed fd check (move constructor)"
				  << std::endl;
			return TestFail;
		}

		if (fd2.isValid() || fd2.get() != -1) {
			std::cout << "Failed moved fd check (move constructor)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_)) {
			std::cout << "Failed fd validity (move constructor)"
				  << std::endl;
			return TestFail;
		}

		/* Test move assignment operator. */
		fd = std::move(fd3);
		if (!fd.isValid() || fd.get() != fd_) {
			std::cout << "Failed fd check (move assignment)"
				  << std::endl;
			return TestFail;
		}

		if (fd3.isValid() || fd3.get() != -1) {
			std::cout << "Failed moved fd check (move assignment)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_)) {
			std::cout << "Failed fd validity (move assignment)"
				  << std::endl;
			return TestFail;
		}

		/* Test swapping. */
		fd2.swap(fd);
		if (!fd2.isValid() || fd2.get() != fd_) {
			std::cout << "Failed fd check (swap)"
				  << std::endl;
			return TestFail;
		}

		if (fd.isValid() || fd.get() != -1) {
			std::cout << "Failed swapped fd check (swap)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_)) {
			std::cout << "Failed fd validity (swap)"
				  << std::endl;
			return TestFail;
		}

		/* Test release. */
		int numFd = fd2.release();
		if (fd2.isValid() || fd2.get() != -1) {
			std::cout << "Failed fd check (release)"
				  << std::endl;
			return TestFail;
		}

		if (numFd != fd_) {
			std::cout << "Failed released fd check (release)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_)) {
			std::cout << "Failed fd validity (release)"
				  << std::endl;
			return TestFail;
		}

		/* Test reset assignment. */
		fd.reset(numFd);
		if (!fd.isValid() || fd.get() != fd_) {
			std::cout << "Failed fd check (reset assignment)"
				  << std::endl;
			return TestFail;
		}

		if (!isValidFd(fd_)) {
			std::cout << "Failed fd validity (reset assignment)"
				  << std::endl;
			return TestFail;
		}

		/* Test reset destruction. */
		fd.reset();
		if (fd.isValid() || fd.get() != -1) {
			std::cout << "Failed fd check (reset destruction)"
				  << std::endl;
			return TestFail;
		}

		if (isValidFd(fd_)) {
			std::cout << "Failed fd validity (reset destruction)"
				  << std::endl;
			return TestFail;
		}

		/* Test destruction. */
		if (createFd() == TestFail) {
			std::cout << "Failed to recreate test fd"
				  << std::endl;
			return TestFail;
		}

		{
			UniqueFD fd4(fd_);
		}

		if (isValidFd(fd_)) {
			std::cout << "Failed fd validity (destruction)"
				  << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup() override
	{
		if (fd_ > 0)
			close(fd_);
	}

private:
	int createFd()
	{
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
};

TEST_REGISTER(UniqueFDTest)
