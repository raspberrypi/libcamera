/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * V4L2 Request
 */

#pragma once

#include <string>

#include <linux/videodev2.h>

#include <libcamera/base/event_notifier.h>
#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

class V4L2Request : protected Loggable
{
public:
	bool isValid() const { return fd_.isValid(); }
	int fd() const { return fd_.get(); }

	int reinit();
	int queue();

	V4L2Request(UniqueFD &&fd);

	Signal<V4L2Request *> requestDone;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(V4L2Request)

	void requestReady();
	std::string logPrefix() const override;

	UniqueFD fd_;
	EventNotifier fdNotifier_;
};

} /* namespace libcamera */
