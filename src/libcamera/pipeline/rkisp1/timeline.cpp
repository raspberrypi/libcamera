/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timeline.cpp - Timeline for per-frame control
 */

#include "timeline.h"

#include "libcamera/internal/log.h"

/**
 * \file timeline.h
 * \brief Timeline for per-frame control
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Timeline)

/**
 * \class FrameAction
 * \brief Action that can be schedule on a Timeline
 *
 * A frame action is an event schedule to be executed on a Timeline. A frame
 * action has two primal attributes a frame number and a type.
 *
 * The frame number describes the frame to which the action is associated. The
 * type is a numerical ID which identifies the action within the pipeline and
 * IPA protocol.
 */

/**
 * \class Timeline
 * \brief Executor of FrameAction
 *
 * The timeline has three primary functions:
 *
 * 1. Keep track of the Start of Exposure (SOE) for every frame processed by
 *    the hardware. Using this information it shall keep an up-to-date estimate
 *    of the frame interval (time between two consecutive SOE events).
 *
 *    The estimated frame interval together with recorded SOE events are the
 *    foundation for how the timeline schedule FrameAction at specific points
 *    in time.
 *    \todo Improve the frame interval estimation algorithm.
 *
 * 2. Keep track of current delays for different types of actions. The delays
 *    for different actions might differ during a capture session. Exposure time
 *    effects the over all FPS and different ISP parameters might impacts its
 *    processing time.
 *
 *    The action type delays shall be updated by the IPA in conjunction with
 *    how it changes the capture parameters.
 *
 * 3. Schedule actions on the timeline. This is the process of taking a
 *    FrameAction which contains an abstract description of what frame and
 *    what type of action it contains and turning that into an time point
 *    and make sure the action is executed at that time.
 */

Timeline::Timeline()
	: frameInterval_(0)
{
	timer_.timeout.connect(this, &Timeline::timeout);
}

/**
 * \brief Reset and stop the timeline
 *
 * The timeline needs to be reset when the timeline should no longer execute
 * actions. A timeline should be reset between two capture sessions to prevent
 * the old capture session to effect the second one.
 */
void Timeline::reset()
{
	timer_.stop();

	actions_.clear();
	history_.clear();
}

/**
 * \brief Schedule an action on the timeline
 * \param[in] action FrameAction to schedule
 *
 * The act of scheduling an action to the timeline is the process of taking
 * the properties of the action (type, frame and time offsets) and translating
 * that to a time point using the current values for the action type timings
 * value recorded in the timeline. If an action is scheduled too late, execute
 * it immediately.
 */
void Timeline::scheduleAction(std::unique_ptr<FrameAction> action)
{
	unsigned int lastFrame;
	utils::time_point lastTime;

	if (history_.empty()) {
		lastFrame = 0;
		lastTime = std::chrono::steady_clock::now();
	} else {
		lastFrame = history_.back().first;
		lastTime = history_.back().second;
	}

	/*
	 * Calculate when the action shall be schedule by first finding out how
	 * many frames in the future the action acts on and then add the actions
	 * frame offset. After the spatial frame offset is found out translate
	 * that to a time point by using the last estimated start of exposure
	 * (SOE) as the fixed offset. Lastly add the action time offset to the
	 * time point.
	 */
	int frame = action->frame() - lastFrame + frameOffset(action->type());
	utils::time_point deadline = lastTime + frame * frameInterval_
		+ timeOffset(action->type());

	utils::time_point now = std::chrono::steady_clock::now();
	if (deadline < now) {
		LOG(Timeline, Warning)
			<< "Action scheduled too late "
			<< utils::time_point_to_string(deadline)
			<< ", run now " << utils::time_point_to_string(now);
		action->run();
	} else {
		actions_.emplace(deadline, std::move(action));
		updateDeadline();
	}
}

void Timeline::notifyStartOfExposure(unsigned int frame, utils::time_point time)
{
	history_.push_back(std::make_pair(frame, time));

	if (history_.size() <= HISTORY_DEPTH / 2)
		return;

	while (history_.size() > HISTORY_DEPTH)
		history_.pop_front();

	/* Update esitmated time between two start of exposures. */
	utils::duration sumExposures(0);
	unsigned int numExposures = 0;

	utils::time_point lastTime;
	for (auto it = history_.begin(); it != history_.end(); it++) {
		if (it != history_.begin()) {
			sumExposures += it->second - lastTime;
			numExposures++;
		}

		lastTime = it->second;
	}

	frameInterval_ = sumExposures;
	if (numExposures)
		frameInterval_ /= numExposures;
}

int Timeline::frameOffset(unsigned int type) const
{
	const auto it = delays_.find(type);
	if (it == delays_.end()) {
		LOG(Timeline, Error)
			<< "No frame offset set for action type " << type;
		return 0;
	}

	return it->second.first;
}

utils::duration Timeline::timeOffset(unsigned int type) const
{
	const auto it = delays_.find(type);
	if (it == delays_.end()) {
		LOG(Timeline, Error)
			<< "No time offset set for action type " << type;
		return utils::duration::zero();
	}

	return it->second.second;
}

void Timeline::setRawDelay(unsigned int type, int frame, utils::duration time)
{
	delays_[type] = std::make_pair(frame, time);
}

void Timeline::updateDeadline()
{
	if (actions_.empty())
		return;

	const utils::time_point &deadline = actions_.begin()->first;

	if (timer_.isRunning() && deadline >= timer_.deadline())
		return;

	if (deadline <= std::chrono::steady_clock::now()) {
		timeout(&timer_);
		return;
	}

	timer_.start(deadline);
}

void Timeline::timeout(Timer *timer)
{
	utils::time_point now = std::chrono::steady_clock::now();

	for (auto it = actions_.begin(); it != actions_.end();) {
		const utils::time_point &sched = it->first;

		if (sched > now)
			break;

		FrameAction *action = it->second.get();

		action->run();

		it = actions_.erase(it);
	}

	updateDeadline();
}

} /* namespace libcamera */
