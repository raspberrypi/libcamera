/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * delayed_controls.h - Helper to deal with controls that take effect with a delay
 */

#include "libcamera/internal/delayed_controls.h"

#include <libcamera/controls.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/v4l2_device.h"

/**
 * \file delayed_controls.h
 * \brief Helper to deal with controls that take effect with a delay
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(DelayedControls)

/**
 * \class DelayedControls
 * \brief Helper to deal with controls that take effect with a delay
 *
 * Some sensor controls take effect with a delay as the sensor needs time to
 * adjust, for example exposure and analog gain. This is a helper class to deal
 * with such controls and the intended users are pipeline handlers.
 *
 * The idea is to extend the concept of the buffer depth of a pipeline the
 * application needs to maintain to also cover controls. Just as with buffer
 * depth if the application keeps the number of requests queued above the
 * control depth the controls are guaranteed to take effect for the correct
 * request. The control depth is determined by the control with the greatest
 * delay.
 */

/**
 * \brief Construct a DelayedControls instance
 * \param[in] device The V4L2 device the controls have to be applied to
 * \param[in] delays Map of the numerical V4L2 control ids to their associated
 * delays (in frames)
 *
 * Only controls specified in \a delays are handled. If it's desired to mix
 * delayed controls and controls that take effect immediately the immediate
 * controls must be listed in the \a delays map with a delay value of 0.
 */
DelayedControls::DelayedControls(V4L2Device *device,
				 const std::unordered_map<uint32_t, unsigned int> &delays)
	: device_(device), maxDelay_(0)
{
	const ControlInfoMap &controls = device_->controls();

	/*
	 * Create a map of control ids to delays for controls exposed by the
	 * device.
	 */
	for (auto const &delay : delays) {
		auto it = controls.find(delay.first);
		if (it == controls.end()) {
			LOG(DelayedControls, Error)
				<< "Delay request for control id "
				<< utils::hex(delay.first)
				<< " but control is not exposed by device "
				<< device_->deviceNode();
			continue;
		}

		const ControlId *id = it->first;

		delays_[id] = delay.second;

		LOG(DelayedControls, Debug)
			<< "Set a delay of " << delays_[id]
			<< " for " << id->name();

		maxDelay_ = std::max(maxDelay_, delays_[id]);
	}

	reset();
}

/**
 * \brief Reset state machine
 *
 * Resets the state machine to a starting position based on control values
 * retrieved from the device.
 */
void DelayedControls::reset()
{
	running_ = false;
	firstSequence_ = 0;
	queueCount_ = 1;
	writeCount_ = 0;

	/* Retrieve control as reported by the device. */
	std::vector<uint32_t> ids;
	for (auto const &delay : delays_)
		ids.push_back(delay.first->id());

	ControlList controls = device_->getControls(ids);

	/* Seed the control queue with the controls reported by the device. */
	values_.clear();
	for (const auto &ctrl : controls) {
		const ControlId *id = device_->controls().idmap().at(ctrl.first);
		values_[id][0] = Info(ctrl.second);
	}
}

/**
 * \brief Push a set of controls on the queue
 * \param[in] controls List of controls to add to the device queue
 *
 * Push a set of controls to the control queue. This increases the control queue
 * depth by one.
 *
 * \returns true if \a controls are accepted, or false otherwise
 */
bool DelayedControls::push(const ControlList &controls)
{
	/* Copy state from previous frame. */
	for (auto &ctrl : values_) {
		Info &info = ctrl.second[queueCount_];
		info = values_[ctrl.first][queueCount_ - 1];
		info.updated = false;
	}

	/* Update with new controls. */
	const ControlIdMap &idmap = device_->controls().idmap();
	for (const auto &control : controls) {
		const auto &it = idmap.find(control.first);
		if (it == idmap.end()) {
			LOG(DelayedControls, Warning)
				<< "Unknown control " << control.first;
			return false;
		}

		const ControlId *id = it->second;

		if (delays_.find(id) == delays_.end())
			return false;

		Info &info = values_[id][queueCount_];

		info = control.second;
		info.updated = true;

		LOG(DelayedControls, Debug)
			<< "Queuing " << id->name()
			<< " to " << info.toString()
			<< " at index " << queueCount_;
	}

	queueCount_++;

	return true;
}

/**
 * \brief Read back controls in effect at a sequence number
 * \param[in] sequence The sequence number to get controls for
 *
 * Read back what controls where in effect at a specific sequence number. The
 * history is a ring buffer of 16 entries where new and old values coexist. It's
 * the callers responsibility to not read too old sequence numbers that have been
 * pushed out of the history.
 *
 * Historic values are evicted by pushing new values onto the queue using
 * push(). The max history from the current sequence number that yields valid
 * values are thus 16 minus number of controls pushed.
 *
 * \return The controls at \a sequence number
 */
ControlList DelayedControls::get(uint32_t sequence)
{
	uint32_t adjustedSeq = sequence - firstSequence_ + 1;
	unsigned int index = std::max<int>(0, adjustedSeq - maxDelay_);

	ControlList out(device_->controls());
	for (const auto &ctrl : values_) {
		const ControlId *id = ctrl.first;
		const Info &info = ctrl.second[index];

		out.set(id->id(), info);

		LOG(DelayedControls, Debug)
			<< "Reading " << id->name()
			<< " to " << info.toString()
			<< " at index " << index;
	}

	return out;
}

/**
 * \brief Inform DelayedControls of the start of a new frame
 * \param[in] sequence Sequence number of the frame that started
 *
 * Inform the state machine that a new frame has started and of its sequence
 * number. Any user of these helpers is responsible to inform the helper about
 * the start of any frame. This can be connected with ease to the start of a
 * exposure (SOE) V4L2 event.
 */
void DelayedControls::applyControls(uint32_t sequence)
{
	LOG(DelayedControls, Debug) << "frame " << sequence << " started";

	if (!running_) {
		firstSequence_ = sequence;
		running_ = true;
	}

	/*
	 * Create control list peeking ahead in the value queue to ensure
	 * values are set in time to satisfy the sensor delay.
	 */
	ControlList out(device_->controls());
	for (const auto &ctrl : values_) {
		const ControlId *id = ctrl.first;
		unsigned int delayDiff = maxDelay_ - delays_[id];
		unsigned int index = std::max<int>(0, writeCount_ - delayDiff);
		const Info &info = ctrl.second[index];

		if (info.updated) {
			out.set(id->id(), info);
			LOG(DelayedControls, Debug)
				<< "Setting " << id->name()
				<< " to " << info.toString()
				<< " at index " << index;
		}
	}

	writeCount_++;

	while (writeCount_ >= queueCount_) {
		LOG(DelayedControls, Debug)
			<< "Queue is empty, auto queue no-op.";
		push({});
	}

	device_->setControls(&out);
}

} /* namespace libcamera */
