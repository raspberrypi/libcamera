/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 Subdevice format handling test
 */

#include <iostream>
#include <vector>

#include <libcamera/geometry.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/v4l2_subdevice.h"

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
		cout << "	mbus code: " << utils::hex(code, 4) << endl;
		cout << "	min width: " << dec << size.min.width << endl;
		cout << "	min height: " << dec << size.min.height << endl;
		cout << "	max width: " << dec << size.max.width << endl;
		cout << "	max height: " << dec << size.max.height << endl;
	}
}

int ListFormatsTest::run()
{
	/* List all formats available on existing "Scaler" pads. */
	V4L2Subdevice::Formats formats;

	formats = scaler_->formats(0);
	if (formats.empty()) {
		cerr << "Failed to list formats on pad 0 of subdevice "
		     << scaler_->entity()->name() << endl;
		return TestFail;
	}
	for (unsigned int code : utils::map_keys(formats))
		printFormats(0, code, formats[code]);

	formats = scaler_->formats(1);
	if (formats.empty()) {
		cerr << "Failed to list formats on pad 1 of subdevice "
		     << scaler_->entity()->name() << endl;
		return TestFail;
	}
	for (unsigned int code : utils::map_keys(formats))
		printFormats(1, code, formats[code]);

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

TEST_REGISTER(ListFormatsTest)
