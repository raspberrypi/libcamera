/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipc_pipe.h - Image Processing Algorithm IPC module for IPA proxies
 */
#ifndef __LIBCAMERA_INTERNAL_IPA_IPC_H__
#define __LIBCAMERA_INTERNAL_IPA_IPC_H__

#include <vector>

#include "libcamera/internal/ipc_unixsocket.h"

#include <libcamera/signal.h>

namespace libcamera {

class IPCMessage
{
public:
	struct Header {
		uint32_t cmd;
		uint32_t cookie;
	};

	IPCMessage();
	IPCMessage(uint32_t cmd);
	IPCMessage(const Header &header);
	IPCMessage(const IPCUnixSocket::Payload &payload);

	IPCUnixSocket::Payload payload() const;

	Header &header() { return header_; }
	std::vector<uint8_t> &data() { return data_; }
	std::vector<int32_t> &fds() { return fds_; }

	const Header &header() const { return header_; }
	const std::vector<uint8_t> &data() const { return data_; }
	const std::vector<int32_t> &fds() const { return fds_; }

private:
	Header header_;

	std::vector<uint8_t> data_;
	std::vector<int32_t> fds_;
};

class IPCPipe
{
public:
	IPCPipe();
	virtual ~IPCPipe();

	bool isConnected() const { return connected_; }

	virtual int sendSync(const IPCMessage &in,
			     IPCMessage *out) = 0;

	virtual int sendAsync(const IPCMessage &data) = 0;

	Signal<const IPCMessage &> recv;

protected:
	bool connected_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_IPA_IPC_H__ */
