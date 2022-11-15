/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * delayed_controls.h - Helper to deal with controls that take effect with a delay
 *
 * Note: This has been forked from the libcamera core implementation.
 */

#pragma once

#include <stdint.h>
#include <unordered_map>
#include <utility>

#include <libcamera/controls.h>

namespace libcamera {

class V4L2Device;

namespace RPi {

class DelayedControls
{
public:
	struct ControlParams {
		unsigned int delay;
		bool priorityWrite;
	};

	DelayedControls(V4L2Device *device,
			const std::unordered_map<uint32_t, ControlParams> &controlParams);

	void reset(unsigned int cookie);

	bool push(const ControlList &controls, unsigned int cookie);
	std::pair<ControlList, unsigned int> get(uint32_t sequence);

	void applyControls(uint32_t sequence);

private:
	class Info : public ControlValue
	{
	public:
		Info()
			: updated(false)
		{
		}

		Info(const ControlValue &v, bool updated_ = true)
			: ControlValue(v), updated(updated_)
		{
		}

		bool updated;
	};

	static constexpr int listSize = 16;
	template<typename T>
	class RingBuffer : public std::array<T, listSize>
	{
	public:
		T &operator[](unsigned int index)
		{
			return std::array<T, listSize>::operator[](index % listSize);
		}

		const T &operator[](unsigned int index) const
		{
			return std::array<T, listSize>::operator[](index % listSize);
		}
	};

	V4L2Device *device_;
	std::unordered_map<const ControlId *, ControlParams> controlParams_;
	unsigned int maxDelay_;

	uint32_t queueCount_;
	uint32_t writeCount_;
	std::unordered_map<const ControlId *, RingBuffer<Info>> values_;
	RingBuffer<unsigned int> cookies_;
};

} /* namespace RPi */

} /* namespace libcamera */
