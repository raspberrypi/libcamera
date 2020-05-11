/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timeline.h - Timeline for per-frame controls
 */
#ifndef __LIBCAMERA_TIMELINE_H__
#define __LIBCAMERA_TIMELINE_H__

#include <list>
#include <map>

#include <libcamera/timer.h>

#include "libcamera/internal/utils.h"

namespace libcamera {

class FrameAction
{
public:
	FrameAction(unsigned int frame, unsigned int type)
		: frame_(frame), type_(type) {}

	virtual ~FrameAction() {}

	unsigned int frame() const { return frame_; }
	unsigned int type() const { return type_; }

	virtual void run() = 0;

private:
	unsigned int frame_;
	unsigned int type_;
};

class Timeline
{
public:
	Timeline();
	virtual ~Timeline() {}

	virtual void reset();
	virtual void scheduleAction(std::unique_ptr<FrameAction> action);
	virtual void notifyStartOfExposure(unsigned int frame, utils::time_point time);

	utils::duration frameInterval() const { return frameInterval_; }

protected:
	int frameOffset(unsigned int type) const;
	utils::duration timeOffset(unsigned int type) const;

	void setRawDelay(unsigned int type, int frame, utils::duration time);

	std::map<unsigned int, std::pair<int, utils::duration>> delays_;

private:
	static constexpr unsigned int HISTORY_DEPTH = 10;

	void timeout(Timer *timer);
	void updateDeadline();

	std::list<std::pair<unsigned int, utils::time_point>> history_;
	std::multimap<utils::time_point, std::unique_ptr<FrameAction>> actions_;
	utils::duration frameInterval_;

	Timer timer_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_TIMELINE_H__ */
