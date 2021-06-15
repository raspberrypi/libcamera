/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * unixsocket_ipc.cpp - Unix socket IPC test
 */

#include <algorithm>
#include <fcntl.h>
#include <iostream>
#include <limits.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/event_dispatcher.h"
#include "libcamera/internal/ipa_data_serializer.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/process.h"
#include "libcamera/internal/thread.h"
#include "libcamera/internal/timer.h"

#include "test.h"


using namespace std;
using namespace libcamera;

enum {
	CmdExit = 0,
	CmdGetSync = 1,
	CmdSetAsync = 2,
};

const int32_t kInitialValue = 1337;
const int32_t kChangedValue = 9001;

class UnixSocketTestIPCSlave
{
public:
	UnixSocketTestIPCSlave()
		: value_(kInitialValue), exitCode_(EXIT_FAILURE), exit_(false)
	{
		dispatcher_ = Thread::current()->eventDispatcher();
		ipc_.readyRead.connect(this, &UnixSocketTestIPCSlave::readyRead);
	}

	int run(int fd)
	{
		if (ipc_.bind(fd)) {
			cerr << "Failed to connect to IPC channel" << endl;
			return EXIT_FAILURE;
		}

		while (!exit_)
			dispatcher_->processEvents();

		ipc_.close();

		return exitCode_;
	}

private:
	void readyRead(IPCUnixSocket *ipc)
	{
		IPCUnixSocket::Payload message;
		int ret;

		ret = ipc->receive(&message);
		if (ret) {
			cerr << "Receive message failed: " << ret << endl;
			return;
		}

		IPCMessage ipcMessage(message);
		uint32_t cmd = ipcMessage.header().cmd;

		switch (cmd) {
		case CmdExit: {
			exit_ = true;
			break;
		}

		case CmdGetSync: {
			IPCMessage::Header header = { cmd, ipcMessage.header().cookie };
			IPCMessage response(header);

			vector<uint8_t> buf;
			tie(buf, ignore) = IPADataSerializer<int32_t>::serialize(value_);
			response.data().insert(response.data().end(), buf.begin(), buf.end());

			ret = ipc_.send(response.payload());
			if (ret < 0) {
				cerr << "Reply failed" << endl;
				stop(ret);
			}
			break;
		}

		case CmdSetAsync: {
			value_ = IPADataSerializer<int32_t>::deserialize(ipcMessage.data());
			break;
		}
		}
	}

	void stop(int code)
	{
		exitCode_ = code;
		exit_ = true;
	}

	int32_t value_;

	IPCUnixSocket ipc_;
	EventDispatcher *dispatcher_;
	int exitCode_;
	bool exit_;
};

class UnixSocketTestIPC : public Test
{
protected:
	int init()
	{
		return 0;
	}

	int setValue(int32_t val)
	{
		IPCMessage msg(CmdSetAsync);
		tie(msg.data(), ignore) = IPADataSerializer<int32_t>::serialize(val);

		int ret = ipc_->sendAsync(msg);
		if (ret < 0) {
			cerr << "Failed to call set value" << endl;
			return ret;
		}

		return 0;
	}

	int getValue()
	{
		IPCMessage msg(CmdGetSync);
		IPCMessage buf;

		int ret = ipc_->sendSync(msg, &buf);
		if (ret < 0) {
			cerr << "Failed to call get value" << endl;
			return ret;
		}

		return IPADataSerializer<int32_t>::deserialize(buf.data());
	}

	int exit()
	{
		IPCMessage msg(CmdExit);

		int ret = ipc_->sendAsync(msg);
		if (ret < 0) {
			cerr << "Failed to call exit" << endl;
			return ret;
		}

		return 0;
	}

	int run()
	{
		ipc_ = std::make_unique<IPCPipeUnixSocket>("", "/proc/self/exe");
		if (!ipc_->isConnected()) {
			cerr << "Failed to create IPCPipe" << endl;
			return TestFail;
		}

		int ret = getValue();
		if (ret != kInitialValue) {
			cerr << "Wrong initial value, expected "
			     << kInitialValue << ", got " << ret << endl;
			return TestFail;
		}

		ret = setValue(kChangedValue);
		if (ret < 0) {
			cerr << "Failed to set value: " << strerror(-ret) << endl;
			return TestFail;
		}

		ret = getValue();
		if (ret != kChangedValue) {
			cerr << "Wrong set value, expected " << kChangedValue
			     << ", got " << ret << endl;
			return TestFail;
		}

		ret = exit();
		if (ret < 0) {
			cerr << "Failed to exit: " << strerror(-ret) << endl;
			return TestFail;
		}

		return TestPass;
	}

private:
	ProcessManager processManager_;

	unique_ptr<IPCPipeUnixSocket> ipc_;
};

/*
 * Can't use TEST_REGISTER() as single binary needs to act as both client and
 * server
 */
int main(int argc, char **argv)
{
	/* IPCPipeUnixSocket passes IPA module path in argv[1] */
	if (argc == 3) {
		int ipcfd = std::stoi(argv[2]);
		UnixSocketTestIPCSlave slave;
		return slave.run(ipcfd);
	}

	return UnixSocketTestIPC().execute();
}
