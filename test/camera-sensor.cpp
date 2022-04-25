/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera-sensor.cpp - Camera sensor tests
 */

#include <algorithm>
#include <iostream>

#include <linux/media-bus-format.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/camera_lens.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "test.h"

using namespace std;
using namespace libcamera;

class CameraSensorTest : public Test
{
protected:
	int init()
	{
		enumerator_ = DeviceEnumerator::create();
		if (!enumerator_) {
			cerr << "Failed to create device enumerator" << endl;
			return TestFail;
		}

		if (enumerator_->enumerate()) {
			cerr << "Failed to enumerate media devices" << endl;
			return TestFail;
		}

		DeviceMatch dm("vimc");
		media_ = enumerator_->search(dm);
		if (!media_) {
			cerr << "Unable to find \'vimc\' media device node" << endl;
			return TestSkip;
		}

		MediaEntity *entity = media_->getEntityByName("Sensor A");
		if (!entity) {
			cerr << "Unable to find media entity 'Sensor A'" << endl;
			return TestFail;
		}

		sensor_ = new CameraSensor(entity);
		if (sensor_->init() < 0) {
			cerr << "Unable to initialise camera sensor" << endl;
			return TestFail;
		}

		lens_ = sensor_->focusLens();
		if (lens_)
			cout << "Found lens controller" << endl;

		return TestPass;
	}

	int run()
	{
		if (sensor_->model() != "Sensor A") {
			cerr << "Incorrect sensor model '" << sensor_->model()
			     << "'" << endl;
			return TestFail;
		}

		const std::vector<unsigned int> &codes = sensor_->mbusCodes();
		auto iter = std::find(codes.begin(), codes.end(),
				      MEDIA_BUS_FMT_ARGB8888_1X32);
		if (iter == codes.end()) {
			cerr << "Sensor doesn't support ARGB8888_1X32" << endl;
			return TestFail;
		}

		const std::vector<Size> &sizes = sensor_->sizes(*iter);
		auto iter2 = std::find(sizes.begin(), sizes.end(),
				       Size(4096, 2160));
		if (iter2 == sizes.end()) {
			cerr << "Sensor doesn't support 4096x2160" << endl;
			return TestFail;
		}

		const Size &resolution = sensor_->resolution();
		if (resolution != Size(4096, 2160)) {
			cerr << "Incorrect sensor resolution " << resolution << endl;
			return TestFail;
		}

		/* Use an invalid format and make sure it's not selected. */
		V4L2SubdeviceFormat format = sensor_->getFormat({ 0xdeadbeef,
								  MEDIA_BUS_FMT_SBGGR10_1X10,
								  MEDIA_BUS_FMT_BGR888_1X24 },
								Size(1024, 768));
		if (format.mbus_code != MEDIA_BUS_FMT_SBGGR10_1X10 ||
		    format.size != Size(4096, 2160)) {
			cerr << "Failed to get a suitable format, expected 4096x2160-0x"
			     << utils::hex(MEDIA_BUS_FMT_SBGGR10_1X10)
			     << ", got " << format << endl;
			return TestFail;
		}

		if (lens_ && lens_->setFocusPosition(10)) {
			cerr << "Failed to set lens focus position" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		delete sensor_;
	}

private:
	std::unique_ptr<DeviceEnumerator> enumerator_;
	std::shared_ptr<MediaDevice> media_;
	CameraSensor *sensor_;
	CameraLens *lens_;
};

TEST_REGISTER(CameraSensorTest)
