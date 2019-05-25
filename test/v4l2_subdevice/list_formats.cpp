/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 Subdevice format handling test
 */

#include <iomanip>
#include <iostream>
#include <vector>

#include <libcamera/geometry.h>

#include "v4l2_subdevice.h"
#include "v4l2_subdevice_test.h"

using namespace std;
using namespace libcamera;

/* List image formats on the "Scaler" subdevice of vimc media device.  */

class ListFormatsTest : public V4L2SubdeviceTest
{
protected:
	int run() override;

private:
	void printFormats(unsigned int pad, unsigned code,
			  const std::vector<SizeRange> &sizes);
};

void ListFormatsTest::printFormats(unsigned int pad,
				   unsigned int code,
				   const std::vector<SizeRange> &sizes)
{
	cout << "Enumerate formats on pad " << pad << endl;
	for (const SizeRange &size : sizes) {
		cout << "	mbus code: 0x" << setfill('0') << setw(4)
		     << hex << code << endl;
		cout << "	min width: " << dec << size.min.width << endl;
		cout << "	min height: " << dec << size.min.height << endl;
		cout << "	max width: " << dec << size.max.width << endl;
		cout << "	max height: " << dec << size.max.height << endl;
	}
}

int ListFormatsTest::run()
{
	/* List all formats available on existing "Scaler" pads. */
	std::map<unsigned int, std::vector<SizeRange>> formats;

	formats = scaler_->formats(0);
	if (formats.empty()) {
		cerr << "Failed to list formats on pad 0 of subdevice "
		     << scaler_->entity()->name() << endl;
		return TestFail;
	}
	for (auto it = formats.begin(); it != formats.end(); ++it)
		printFormats(0, it->first, it->second);

	formats = scaler_->formats(1);
	if (formats.empty()) {
		cerr << "Failed to list formats on pad 1 of subdevice "
		     << scaler_->entity()->name() << endl;
		return TestFail;
	}
	for (auto it = formats.begin(); it != formats.end(); ++it)
		printFormats(1, it->first, it->second);

	/* List format on a non-existing pad, format vector shall be empty. */
	formats = scaler_->formats(2);
	if (!formats.empty()) {
		cerr << "Listing formats on non-existing pad 2 of subdevice "
		     << scaler_->entity()->name()
		     << " should return an empty format list" << endl;
		return TestFail;
	}

	return TestPass;
}

TEST_REGISTER(ListFormatsTest);
