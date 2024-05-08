/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * V4L2 device controls handling test
 */

#include <algorithm>
#include <array>
#include <iostream>
#include <limits.h>

#include "libcamera/internal/v4l2_videodevice.h"

#include "v4l2_videodevice_test.h"

/* These come from the vivid driver. */
#define VIVID_CID_CUSTOM_BASE		(V4L2_CID_USER_BASE | 0xf000)
#define VIVID_CID_INTEGER64		(VIVID_CID_CUSTOM_BASE + 3)
#define VIVID_CID_U8_4D_ARRAY		(VIVID_CID_CUSTOM_BASE + 10)

/* Helper for VIVID_CID_U8_4D_ARRAY control array size: not from kernel. */
#define VIVID_CID_U8_ARRAY_SIZE		(2 * 3 * 4 * 5)

using namespace std;
using namespace libcamera;

class V4L2ControlTest : public V4L2VideoDeviceTest
{
public:
	V4L2ControlTest()
		: V4L2VideoDeviceTest("vivid", "vivid-000-vid-cap")
	{
	}

protected:
	int run()
	{
		const ControlInfoMap &infoMap = capture_->controls();

		/* Test control enumeration. */
		if (infoMap.empty()) {
			cerr << "Failed to enumerate controls" << endl;
			return TestFail;
		}

		if (infoMap.find(V4L2_CID_BRIGHTNESS) == infoMap.end() ||
		    infoMap.find(V4L2_CID_CONTRAST) == infoMap.end() ||
		    infoMap.find(V4L2_CID_SATURATION) == infoMap.end() ||
		    infoMap.find(VIVID_CID_INTEGER64) == infoMap.end() ||
		    infoMap.find(VIVID_CID_U8_4D_ARRAY) == infoMap.end()) {
			cerr << "Missing controls" << endl;
			return TestFail;
		}

		const ControlInfo &brightness = infoMap.find(V4L2_CID_BRIGHTNESS)->second;
		const ControlInfo &contrast = infoMap.find(V4L2_CID_CONTRAST)->second;
		const ControlInfo &saturation = infoMap.find(V4L2_CID_SATURATION)->second;
		const ControlInfo &int64 = infoMap.find(VIVID_CID_INTEGER64)->second;
		const ControlInfo &u8 = infoMap.find(VIVID_CID_U8_4D_ARRAY)->second;

		/* Test getting controls. */
		ControlList ctrls = capture_->getControls({ V4L2_CID_BRIGHTNESS,
							    V4L2_CID_CONTRAST,
							    V4L2_CID_SATURATION,
							    VIVID_CID_INTEGER64,
							    VIVID_CID_U8_4D_ARRAY });
		if (ctrls.empty()) {
			cerr << "Failed to get controls" << endl;
			return TestFail;
		}

		if (ctrls.infoMap() != &infoMap) {
			cerr << "Incorrect infoMap for retrieved controls" << endl;
			return TestFail;
		}

		if (ctrls.get(V4L2_CID_BRIGHTNESS).get<int32_t>() == -1 ||
		    ctrls.get(V4L2_CID_CONTRAST).get<int32_t>() == -1 ||
		    ctrls.get(V4L2_CID_SATURATION).get<int32_t>() == -1) {
			cerr << "Incorrect value for retrieved controls" << endl;
			return TestFail;
		}

		/*
		 * The VIVID_CID_INTEGER64 control can take any value, just test
		 * that its value can be retrieved and has the right type.
		 */
		ctrls.get(VIVID_CID_INTEGER64).get<int64_t>();

		uint8_t u8Min = u8.min().get<uint8_t>();
		uint8_t u8Max = u8.max().get<uint8_t>();

		Span<const uint8_t> u8Span = ctrls.get(VIVID_CID_U8_4D_ARRAY).get<Span<const uint8_t>>();
		bool valid = std::all_of(u8Span.begin(), u8Span.end(),
					 [&](uint8_t v) { return v >= u8Min && v <= u8Max; });
		if (!valid) {
			cerr << "Incorrect value for retrieved array control"
			     << endl;
			return TestFail;
		}

		/* Test setting controls. */
		ctrls.set(V4L2_CID_BRIGHTNESS, brightness.min());
		ctrls.set(V4L2_CID_CONTRAST, contrast.max());
		ctrls.set(V4L2_CID_SATURATION, saturation.min());
		ctrls.set(VIVID_CID_INTEGER64, int64.min());

		std::array<uint8_t, VIVID_CID_U8_ARRAY_SIZE> u8Values;
		std::fill(u8Values.begin(), u8Values.end(), u8.min().get<uint8_t>());
		ctrls.set(VIVID_CID_U8_4D_ARRAY, Span<const uint8_t>(u8Values));

		int ret = capture_->setControls(&ctrls);
		if (ret) {
			cerr << "Failed to set controls" << endl;
			return TestFail;
		}

		/* Test setting controls outside of range. */
		ctrls.set(V4L2_CID_BRIGHTNESS, brightness.min().get<int32_t>() - 1);
		ctrls.set(V4L2_CID_CONTRAST, contrast.max().get<int32_t>() + 1);
		ctrls.set(V4L2_CID_SATURATION, saturation.min().get<int32_t>() + 1);

		ret = capture_->setControls(&ctrls);
		if (ret) {
			cerr << "Failed to set controls (out of range)" << endl;
			return TestFail;
		}

		if (ctrls.get(V4L2_CID_BRIGHTNESS) != brightness.min() ||
		    ctrls.get(V4L2_CID_CONTRAST) != contrast.max() ||
		    ctrls.get(V4L2_CID_SATURATION) != saturation.min().get<int32_t>() + 1) {
			cerr << "Controls not updated when set" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(V4L2ControlTest)
