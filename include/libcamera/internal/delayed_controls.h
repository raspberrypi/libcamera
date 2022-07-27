/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * delayed_controls.h - Helper to deal with controls that take effect with a delay
 */

#pragma once

#include <stdint.h>
#include <unordered_map>

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

	void reset();

	bool push(const ControlList &controls);
	ControlList get(uint32_t sequence);

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
	class ControlRingBuffer : public std::array<Info, listSize>
	{
	public:
		Info &operator[](unsigned int index)
		{
			return std::array<Info, listSize>::operator[](index % listSize);
		}

		const Info &operator[](unsigned int index) const
		{
			return std::array<Info, listSize>::operator[](index % listSize);
		}
	};

	V4L2Device *device_;
	/* \todo Evaluate if we should index on ControlId * or unsigned int */
	std::unordered_map<const ControlId *, ControlParams> controlParams_;
	unsigned int maxDelay_;

	uint32_t queueCount_;
	uint32_t writeCount_;
	/* \todo Evaluate if we should index on ControlId * or unsigned int */
	std::unordered_map<const ControlId *, ControlRingBuffer> values_;
};

} /* namespace libcamera */
