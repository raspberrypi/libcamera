/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * process.h - Process object
 */
#ifndef __LIBCAMERA_INTERNAL_PROCESS_H__
#define __LIBCAMERA_INTERNAL_PROCESS_H__

#include <string>
#include <vector>

#include <libcamera/event_notifier.h>

namespace libcamera {

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
		  const std::vector<std::string> &args = std::vector<std::string>(),
		  const std::vector<int> &fds = std::vector<int>());

	ExitStatus exitStatus() const { return exitStatus_; }
	int exitCode() const { return exitCode_; }

	void kill();

	Signal<Process *, enum ExitStatus, int> finished;

private:
	void closeAllFdsExcept(const std::vector<int> &fds);
	int isolate();
	void died(int wstatus);

	pid_t pid_;
	bool running_;
	enum ExitStatus exitStatus_;
	int exitCode_;

	friend class ProcessManager;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_PROCESS_H__ */
