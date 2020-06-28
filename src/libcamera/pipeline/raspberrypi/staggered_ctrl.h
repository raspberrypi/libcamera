/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * staggered_ctrl.h - Helper for writing staggered ctrls to a V4L2 device.
 */
#ifndef __LIBCAMERA_PIPELINE_RASPBERRYPI_STAGGERED_CTRL_H__
#define __LIBCAMERA_PIPELINE_RASPBERRYPI_STAGGERED_CTRL_H__

#include <array>
#include <initializer_list>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace libcamera {

class ControlList;
class V4L2VideoDevice;

namespace RPi {

class StaggeredCtrl
{
public:
	StaggeredCtrl()
		: init_(false), setCount_(0), getCount_(0), maxDelay_(0)
	{
	}

	operator bool() const
	{
		return init_;
	}

	void init(V4L2VideoDevice *dev,
		  std::initializer_list<std::pair<const uint32_t, uint8_t>> delayList);
	void reset();

	void get(std::unordered_map<uint32_t, int32_t> &ctrl, uint8_t offset = 0);

	bool set(uint32_t ctrl, int32_t value);
	bool set(std::initializer_list<std::pair<const uint32_t, int32_t>> ctrlList);
	bool set(const ControlList &controls);

	int write();

private:
	void nextFrame();

	/* listSize must be a power of 2. */
	static constexpr int listSize = (1 << 4);
	struct CtrlInfo {
		CtrlInfo()
			: value(0), updated(false)
		{
		}

		CtrlInfo(int32_t value_)
			: value(value_), updated(true)
		{
		}

		int32_t value;
		bool updated;
	};

	class CircularArray : public std::array<CtrlInfo, listSize>
	{
	public:
		CtrlInfo &operator[](int index)
		{
			return std::array<CtrlInfo, listSize>::operator[](index & (listSize - 1));
		}

		const CtrlInfo &operator[](int index) const
		{
			return std::array<CtrlInfo, listSize>::operator[](index & (listSize - 1));
		}
	};

	bool init_;
	uint32_t setCount_;
	uint32_t getCount_;
	uint8_t maxDelay_;
	V4L2VideoDevice *dev_;
	std::unordered_map<uint32_t, uint8_t> delay_;
	std::unordered_map<uint32_t, CircularArray> ctrl_;
	std::mutex lock_;
};

} /* namespace RPi */

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_RASPBERRYPI_STAGGERED_CTRL_H__ */
