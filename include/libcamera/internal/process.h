/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * process.h - Process object
 */
#ifndef __LIBCAMERA_INTERNAL_PROCESS_H__
#define __LIBCAMERA_INTERNAL_PROCESS_H__

#include <signal.h>
#include <string>
#include <vector>

#include <libcamera/signal.h>

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

class ProcessManager
{
public:
	ProcessManager();
	~ProcessManager();

	void registerProcess(Process *proc);

	static ProcessManager *instance();

	int writePipe() const;

	const struct sigaction &oldsa() const;

private:
	static ProcessManager *self_;

	void sighandler(EventNotifier *notifier);

	std::list<Process *> processes_;

	struct sigaction oldsa_;
	EventNotifier *sigEvent_;
	int pipe_[2];
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_PROCESS_H__ */
