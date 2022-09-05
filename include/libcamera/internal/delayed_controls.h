/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * delayed_controls.h - Helper to deal with controls that take effect with a delay
 */

#pragma once

#include <stdint.h>
#include <unordered_map>
#include <utility>

#include <libcamera/controls.h>

namespace libcamera {

class V4L2Device;

class DelayedControls
{
public:
	struct ControlParams {
		unsigned int delay;
		bool priorityWrite;
	};

	DelayedControls(V4L2Device *device,
			const std::unordered_map<uint32_t, ControlParams> &controlParams);

	void reset(unsigned int cookie = 0);

	bool push(const ControlList &controls, unsigned int cookie = 0);
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

	/* \todo Make the listSize configurable at instance creation time. */
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
	/* \todo Evaluate if we should index on ControlId * or unsigned int */
	std::unordered_map<const ControlId *, ControlParams> controlParams_;
	unsigned int maxDelay_;

	uint32_t queueCount_;
	uint32_t writeCount_;
	/* \todo Evaluate if we should index on ControlId * or unsigned int */
	std::unordered_map<const ControlId *, RingBuffer<Info>> values_;
	RingBuffer<unsigned int> cookies_;
};

} /* namespace libcamera */
