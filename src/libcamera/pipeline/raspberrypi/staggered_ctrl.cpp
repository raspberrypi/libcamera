/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * staggered_ctrl.cpp - Helper for writing staggered ctrls to a V4L2 device.
 */

#include "staggered_ctrl.h"

#include <algorithm>

#include <libcamera/controls.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/utils.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(RPI_S_W);

namespace RPi {

void StaggeredCtrl::init(V4L2VideoDevice *dev,
	  std::initializer_list<std::pair<const uint32_t, uint8_t>> delayList)
{
	std::lock_guard<std::mutex> lock(lock_);

	dev_ = dev;
	delay_ = delayList;
	ctrl_.clear();

	/* Find the largest delay across all controls. */
	maxDelay_ = 0;
	for (auto const &p : delay_) {
		LOG(RPI_S_W, Info) << "Init ctrl "
				   << utils::hex(p.first) << " with delay "
				   << static_cast<int>(p.second);
		maxDelay_ = std::max(maxDelay_, p.second);
	}

	init_ = true;
}

void StaggeredCtrl::reset()
{
	std::lock_guard<std::mutex> lock(lock_);

	int lastSetCount = setCount_;
	std::unordered_map<uint32_t, int32_t> lastVal;

	/* Reset the counters. */
	setCount_ = getCount_ = 0;

	/* Look for the last set values. */
	for (auto const &c : ctrl_)
		lastVal[c.first] = c.second[lastSetCount].value;

	/* Apply the last set values as the next to be applied. */
	ctrl_.clear();
	for (auto &c : lastVal)
		ctrl_[c.first][setCount_] = CtrlInfo(c.second);
}

bool StaggeredCtrl::set(uint32_t ctrl, int32_t value)
{
	std::lock_guard<std::mutex> lock(lock_);

	/* Can we find this ctrl as one that is registered? */
	if (delay_.find(ctrl) == delay_.end())
		return false;

	ctrl_[ctrl][setCount_].value = value;
	ctrl_[ctrl][setCount_].updated = true;

	return true;
}

bool StaggeredCtrl::set(std::initializer_list<std::pair<const uint32_t, int32_t>> ctrlList)
{
	std::lock_guard<std::mutex> lock(lock_);

	for (auto const &p : ctrlList) {
		/* Can we find this ctrl? */
		if (delay_.find(p.first) == delay_.end())
			return false;

		ctrl_[p.first][setCount_] = CtrlInfo(p.second);
	}

	return true;
}

bool StaggeredCtrl::set(const ControlList &controls)
{
	std::lock_guard<std::mutex> lock(lock_);

	for (auto const &p : controls) {
		/* Can we find this ctrl? */
		if (delay_.find(p.first) == delay_.end())
			return false;

		ctrl_[p.first][setCount_] = CtrlInfo(p.second.get<int32_t>());
		LOG(RPI_S_W, Debug) << "Setting ctrl "
				    << utils::hex(p.first) << " to "
				    << ctrl_[p.first][setCount_].value
				    << " at index "
				    << setCount_;
	}

	return true;
}

int StaggeredCtrl::write()
{
	std::lock_guard<std::mutex> lock(lock_);
	ControlList controls(dev_->controls());

	for (auto &p : ctrl_) {
		int delayDiff = maxDelay_ - delay_[p.first];
		int index = std::max<int>(0, setCount_ - delayDiff);

		if (p.second[index].updated) {
			/* We need to write this value out. */
			controls.set(p.first, p.second[index].value);
			p.second[index].updated = false;
			LOG(RPI_S_W, Debug) << "Writing ctrl "
					    << utils::hex(p.first) << " to "
					    << p.second[index].value
					    << " at index "
					    << index;
		}
	}

	nextFrame();
	return dev_->setControls(&controls);
}

void StaggeredCtrl::get(std::unordered_map<uint32_t, int32_t> &ctrl, uint8_t offset)
{
	std::lock_guard<std::mutex> lock(lock_);

	/* Account for the offset to reset the getCounter. */
	getCount_ += offset + 1;

	ctrl.clear();
	for (auto &p : ctrl_) {
		int index = std::max<int>(0, getCount_ - maxDelay_);
		ctrl[p.first] = p.second[index].value;
		LOG(RPI_S_W, Debug) << "Getting ctrl "
				    << utils::hex(p.first) << " to "
				    << p.second[index].value
				    << " at index "
				    << index;
	}
}

void StaggeredCtrl::nextFrame()
{
	/* Advance the control history to the next frame */
	int prevCount = setCount_;
	setCount_++;

	LOG(RPI_S_W, Debug) << "Next frame, set index is " << setCount_;

	for (auto &p : ctrl_) {
		p.second[setCount_].value = p.second[prevCount].value;
		p.second[setCount_].updated = false;
	}
}

} /* namespace RPi */

} /* namespace libcamera */
