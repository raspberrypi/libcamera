/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm IPC module using unix socket
 */

#pragma once

#include <map>
#include <memory>
#include <stdint.h>

#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_unixsocket.h"

namespace libcamera {

class Process;

class IPCPipeUnixSocket : public IPCPipe
{
public:
	IPCPipeUnixSocket(const char *ipaModulePath, const char *ipaProxyWorkerPath);
	~IPCPipeUnixSocket();

	int sendSync(const IPCMessage &in,
		     IPCMessage *out = nullptr) override;

	int sendAsync(const IPCMessage &data) override;

private:
	struct CallData {
		IPCUnixSocket::Payload *response;
		bool done;
	};

	void readyRead();
	int call(const IPCUnixSocket::Payload &message,
		 IPCUnixSocket::Payload *response, uint32_t seq);

	std::unique_ptr<Process> proc_;
	std::unique_ptr<IPCUnixSocket> socket_;
	std::map<uint32_t, CallData> callData_;
};

} /* namespace libcamera */
