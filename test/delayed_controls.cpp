/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * delayed_controls.cpp - libcamera delayed controls test
 */

#include <iostream>

#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "test.h"

using namespace std;
using namespace libcamera;

class DelayedControlsTest : public Test
{
public:
	DelayedControlsTest()
	{
	}

protected:
	int init() override
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

		DeviceMatch dm("vivid");
		dm.add("vivid-000-vid-cap");

		media_ = enumerator_->search(dm);
		if (!media_) {
			cerr << "vivid video device found" << endl;
			return TestSkip;
		}

		dev_ = V4L2VideoDevice::fromEntityName(media_.get(), "vivid-000-vid-cap");
		if (dev_->open()) {
			cerr << "Failed to open video device" << endl;
			return TestFail;
		}

		const ControlInfoMap &infoMap = dev_->controls();

		/* Make sure the controls we require are present. */
		if (infoMap.empty()) {
			cerr << "Failed to enumerate controls" << endl;
			return TestFail;
		}

		if (infoMap.find(V4L2_CID_BRIGHTNESS) == infoMap.end() ||
		    infoMap.find(V4L2_CID_CONTRAST) == infoMap.end()) {
			cerr << "Missing controls" << endl;
			return TestFail;
		}

		return TestPass;
	}

	int singleControlNoDelay()
	{
		std::unordered_map<uint32_t, DelayedControls::ControlParams> delays = {
			{ V4L2_CID_BRIGHTNESS, { 0, false } },
		};
		std::unique_ptr<DelayedControls> delayed =
			std::make_unique<DelayedControls>(dev_.get(), delays);
		ControlList ctrls;

		/* Reset control to value not used in test. */
		ctrls.set(V4L2_CID_BRIGHTNESS, 1);
		dev_->setControls(&ctrls);
		delayed->reset();

		/* Trigger the first frame start event */
		delayed->applyControls(0);

		/* Test control without delay are set at once. */
		for (unsigned int i = 1; i < 100; i++) {
			int32_t value = 100 + i;

			ctrls.set(V4L2_CID_BRIGHTNESS, value);
			delayed->push(ctrls);

			delayed->applyControls(i);

			ControlList result = delayed->get(i);
			int32_t brightness = result.get(V4L2_CID_BRIGHTNESS).get<int32_t>();
			if (brightness != value) {
				cerr << "Failed single control without delay"
				     << " frame " << i
				     << " expected " << value
				     << " got " << brightness
				     << endl;
				return TestFail;
			}
		}

		return TestPass;
	}

	int singleControlWithDelay()
	{
		std::unordered_map<uint32_t, DelayedControls::ControlParams> delays = {
			{ V4L2_CID_BRIGHTNESS, { 1, false } },
		};
		std::unique_ptr<DelayedControls> delayed =
			std::make_unique<DelayedControls>(dev_.get(), delays);
		ControlList ctrls;

		/* Reset control to value that will be first in test. */
		int32_t expected = 4;
		ctrls.set(V4L2_CID_BRIGHTNESS, expected);
		dev_->setControls(&ctrls);
		delayed->reset();

		/* Trigger the first frame start event */
		delayed->applyControls(0);

		/* Test single control with delay. */
		for (unsigned int i = 1; i < 100; i++) {
			int32_t value = 10 + i;

			ctrls.set(V4L2_CID_BRIGHTNESS, value);
			delayed->push(ctrls);

			delayed->applyControls(i);

			ControlList result = delayed->get(i);
			int32_t brightness = result.get(V4L2_CID_BRIGHTNESS).get<int32_t>();
			if (brightness != expected) {
				cerr << "Failed single control with delay"
				     << " frame " << i
				     << " expected " << expected
				     << " got " << brightness
				     << endl;
				return TestFail;
			}

			expected = value;
		}

		return TestPass;
	}

	int dualControlsWithDelay()
	{
		static const unsigned int maxDelay = 2;

		std::unordered_map<uint32_t, DelayedControls::ControlParams> delays = {
			{ V4L2_CID_BRIGHTNESS, { 1, false } },
			{ V4L2_CID_CONTRAST, { maxDelay, false } },
		};
		std::unique_ptr<DelayedControls> delayed =
			std::make_unique<DelayedControls>(dev_.get(), delays);
		ControlList ctrls;

		/* Reset control to value that will be first two frames in test. */
		int32_t expected = 200;
		ctrls.set(V4L2_CID_BRIGHTNESS, expected);
		ctrls.set(V4L2_CID_CONTRAST, expected + 1);
		dev_->setControls(&ctrls);
		delayed->reset();

		/* Trigger the first frame start event */
		delayed->applyControls(0);

		/* Test dual control with delay. */
		for (unsigned int i = 1; i < 100; i++) {
			int32_t value = 10 + i;

			ctrls.set(V4L2_CID_BRIGHTNESS, value);
			ctrls.set(V4L2_CID_CONTRAST, value + 1);
			delayed->push(ctrls);

			delayed->applyControls(i);

			ControlList result = delayed->get(i);
			int32_t brightness = result.get(V4L2_CID_BRIGHTNESS).get<int32_t>();
			int32_t contrast = result.get(V4L2_CID_CONTRAST).get<int32_t>();
			if (brightness != expected || contrast != expected + 1) {
				cerr << "Failed dual controls"
				     << " frame " << i
				     << " brightness " << brightness
				     << " contrast " << contrast
				     << " expected " << expected
				     << endl;
				return TestFail;
			}

			expected = i < maxDelay ? expected : value - 1;
		}

		return TestPass;
	}

	int dualControlsMultiQueue()
	{
		static const unsigned int maxDelay = 2;

		std::unordered_map<uint32_t, DelayedControls::ControlParams> delays = {
			{ V4L2_CID_BRIGHTNESS, { 1, false } },
			{ V4L2_CID_CONTRAST, { maxDelay, false } }
		};
		std::unique_ptr<DelayedControls> delayed =
			std::make_unique<DelayedControls>(dev_.get(), delays);
		ControlList ctrls;

		/* Reset control to value that will be first two frames in test. */
		int32_t expected = 100;
		ctrls.set(V4L2_CID_BRIGHTNESS, expected);
		ctrls.set(V4L2_CID_CONTRAST, expected);
		dev_->setControls(&ctrls);
		delayed->reset();

		/* Trigger the first frame start event */
		delayed->applyControls(0);

		/*
		 * Queue all controls before any fake frame start. Note we
		 * can't queue up more then the delayed controls history size
		 * which is 16. Where one spot is used by the reset control.
		 */
		for (unsigned int i = 0; i < 15; i++) {
			int32_t value = 10 + i;

			ctrls.set(V4L2_CID_BRIGHTNESS, value);
			ctrls.set(V4L2_CID_CONTRAST, value);
			delayed->push(ctrls);
		}

		/* Process all queued controls. */
		for (unsigned int i = 1; i < 16; i++) {
			int32_t value = 10 + i - 1;

			delayed->applyControls(i);

			ControlList result = delayed->get(i);

			int32_t brightness = result.get(V4L2_CID_BRIGHTNESS).get<int32_t>();
			int32_t contrast = result.get(V4L2_CID_CONTRAST).get<int32_t>();
			if (brightness != expected || contrast != expected) {
				cerr << "Failed multi queue"
				     << " frame " << i
				     << " brightness " << brightness
				     << " contrast " << contrast
				     << " expected " << expected
				     << endl;
				return TestFail;
			}

			expected = i < maxDelay ? expected : value - 1;
		}

		return TestPass;
	}

	int run() override
	{
		int ret;

		/* Test single control without delay. */
		ret = singleControlNoDelay();
		if (ret)
			return ret;

		/* Test single control with delay. */
		ret = singleControlWithDelay();
		if (ret)
			return ret;

		/* Test dual controls with different delays. */
		ret = dualControlsWithDelay();
		if (ret)
			return ret;

		/* Test control values produced faster than consumed. */
		ret = dualControlsMultiQueue();
		if (ret)
			return ret;

		return TestPass;
	}

private:
	std::unique_ptr<DeviceEnumerator> enumerator_;
	std::shared_ptr<MediaDevice> media_;
	std::unique_ptr<V4L2VideoDevice> dev_;
};

TEST_REGISTER(DelayedControlsTest)
