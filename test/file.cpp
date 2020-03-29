/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * file.cpp - File I/O operations tests
 */

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "file.h"
#include "test.h"

using namespace std;
using namespace libcamera;

class FileTest : public Test
{
protected:
	int init()
	{
		fileName_ = "/tmp/libcamera.test.XXXXXX";
		int fd = mkstemp(&fileName_.front());
		if (fd == -1)
			return TestFail;

		write(fd, "libcamera", 9);
		close(fd);

		return TestPass;
	}

	int run()
	{
		/* Test static functions. */
		if (!File::exists("/dev/null")) {
			cerr << "Valid file not found" << endl;
			return TestFail;
		}

		if (File::exists("/dev/null/invalid")) {
			cerr << "Invalid file should not exist" << endl;
			return TestFail;
		}

		/* Test unnamed file. */
		File file;

		if (!file.fileName().empty()) {
			cerr << "Unnamed file has non-empty file name" << endl;
			return TestFail;
		}

		if (file.exists()) {
			cerr << "Unnamed file exists" << endl;
			return TestFail;
		}

		if (file.isOpen()) {
			cerr << "File is open after construction" << endl;
			return TestFail;
		}

		if (file.openMode() != File::NotOpen) {
			cerr << "File has invalid open mode after construction"
			     << endl;
			return TestFail;
		}

		if (file.size() >= 0) {
			cerr << "Unnamed file has a size" << endl;
			return TestFail;
		}

		if (file.open(File::ReadWrite)) {
			cerr << "Opening unnamed file succeeded" << endl;
			return TestFail;
		}

		if (file.error() == 0) {
			cerr << "Open failure didn't set error" << endl;
			return TestFail;
		}

		/* Test named file referring to an invalid file. */
		file.setFileName("/dev/null/invalid");

		if (file.fileName() != "/dev/null/invalid") {
			cerr << "File reports incorrect file name" << endl;
			return TestFail;
		}

		if (file.exists()) {
			cerr << "Invalid file exists" << endl;
			return TestFail;
		}

		if (file.isOpen()) {
			cerr << "Invalid file is open after construction" << endl;
			return TestFail;
		}

		if (file.openMode() != File::NotOpen) {
			cerr << "Invalid file has invalid open mode after construction"
			     << endl;
			return TestFail;
		}

		if (file.size() >= 0) {
			cerr << "Invalid file has a size" << endl;
			return TestFail;
		}

		if (file.open(File::ReadWrite)) {
			cerr << "Opening invalid file succeeded" << endl;
			return TestFail;
		}

		/* Test named file referring to a valid file. */
		file.setFileName("/dev/null");

		if (!file.exists()) {
			cerr << "Valid file does not exist" << endl;
			return TestFail;
		}

		if (file.isOpen()) {
			cerr << "Valid file is open after construction" << endl;
			return TestFail;
		}

		if (file.openMode() != File::NotOpen) {
			cerr << "Valid file has invalid open mode after construction"
			     << endl;
			return TestFail;
		}

		if (file.size() >= 0) {
			cerr << "Invalid file has a size" << endl;
			return TestFail;
		}

		/* Test open and close. */
		if (!file.open(File::ReadWrite)) {
			cerr << "Opening file failed" << endl;
			return TestFail;
		}

		if (!file.isOpen()) {
			cerr << "Open file reported as closed" << endl;
			return TestFail;
		}

		if (file.openMode() != File::ReadWrite) {
			cerr << "Open file has invalid open mode" << endl;
			return TestFail;
		}

		file.close();

		if (file.isOpen()) {
			cerr << "Closed file reported as open" << endl;
			return TestFail;
		}

		if (file.openMode() != File::NotOpen) {
			cerr << "Closed file has invalid open mode" << endl;
			return TestFail;
		}

		/* Test size(). */
		file.setFileName("/proc/self/exe");

		if (file.size() >= 0) {
			cerr << "File has valid size before open" << endl;
			return TestFail;
		}

		file.open(File::ReadOnly);

		ssize_t size = file.size();
		if (size <= 0) {
			cerr << "File has invalid size after open" << endl;
			return TestFail;
		}

		/* Test mapping and unmapping. */
		Span<uint8_t> data = file.map();
		if (data.empty()) {
			cerr << "Mapping of complete file failed" << endl;
			return TestFail;
		}

		if (data.size() != static_cast<size_t>(size)) {
			cerr << "Mapping  of complete file has invalid size" << endl;
			return TestFail;
		}

		if (!file.unmap(data.data())) {
			cerr << "Unmapping of complete file failed" << endl;
			return TestFail;
		}

		data = file.map(4096, 8192);
		if (data.empty()) {
			cerr << "Mapping of file region failed" << endl;
			return TestFail;
		}

		if (data.size() != 8192) {
			cerr << "Mapping of file region has invalid size" << endl;
			return TestFail;
		}

		if (!file.unmap(data.data())) {
			cerr << "Unmapping of file region failed" << endl;
			return TestFail;
		}

		file.close();

		/* Test private mapping. */
		file.setFileName(fileName_);
		file.open(File::ReadWrite);

		data = file.map(0, -1, File::MapPrivate);
		if (data.empty()) {
			cerr << "Private mapping failed" << endl;
			return TestFail;
		}

		std::string str{ reinterpret_cast<char *>(data.data()), data.size() };
		if (str != "libcamera") {
			cerr << "Invalid contents of private mapping" << endl;
			return TestFail;
		}

		memcpy(data.data(), "LIBCAMERA", 9);

		if (!file.unmap(data.data())) {
			cerr << "Private unmapping failed" << endl;
			return TestFail;
		}

		data = file.map();

		str = { reinterpret_cast<char *>(data.data()), data.size() };
		if (str != "libcamera") {
			cerr << "Private mapping changed file contents" << endl;
			return TestFail;
		}

		/* Test shared mapping. */
		data = file.map();
		if (data.empty()) {
			cerr << "Shared mapping failed" << endl;
			return TestFail;
		}

		memcpy(data.data(), "LIBCAMERA", 9);

		if (!file.unmap(data.data())) {
			cerr << "Shared unmapping failed" << endl;
			return TestFail;
		}

		data = file.map();

		str = { reinterpret_cast<char *>(data.data()), data.size() };
		if (str != "LIBCAMERA") {
			cerr << "Shared mapping failed to change file contents"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		unlink(fileName_.c_str());
	}

private:
	std::string fileName_;
};

TEST_REGISTER(FileTest)
