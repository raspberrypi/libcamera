/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * process.h - Process object
 */

#pragma once

#include <signal.h>
#include <string>
#include <vector>

#include <libcamera/base/signal.h>
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
		  const std::vector<std::string> &args = std::vector<std::string>(),
		  const std::vector<int> &fds = std::vector<int>());

	ExitStatus exitStatus() const { return exitStatus_; }
	int exitCode() const { return exitCode_; }

	void kill();

	Signal<enum ExitStatus, int> finished;

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

	void sighandler();

	std::list<Process *> processes_;

	struct sigaction oldsa_;

	EventNotifier *sigEvent_;
	UniqueFD pipe_[2];
};

} /* namespace libcamera */
