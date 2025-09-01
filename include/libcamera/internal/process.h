/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Process object
 */

#pragma once

#include <string>

#include <libcamera/base/class.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/span.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

class EventNotifier;

class Process final
{
public:
	enum ExitStatus {
		NotExited,
		NormalExit,
		SignalExit,
	};

	Process();
	~Process();

	int start(const std::string &path,
		  Span<const std::string> args = {},
		  Span<const int> fds = {});

	ExitStatus exitStatus() const { return exitStatus_; }
	int exitCode() const { return exitCode_; }

	void kill();

	Signal<enum ExitStatus, int> finished;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(Process)

	void onPidfdNotify();

	pid_t pid_;
	enum ExitStatus exitStatus_;
	int exitCode_;

	UniqueFD pidfd_;
	std::unique_ptr<EventNotifier> pidfdNotify_;
};

} /* namespace libcamera */
