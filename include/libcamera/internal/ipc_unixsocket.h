/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * IPC mechanism based on Unix sockets
 */

#pragma once

#include <stdint.h>
#include <sys/types.h>
#include <vector>

#include <libcamera/base/signal.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

class EventNotifier;

class IPCUnixSocket
{
public:
	struct Payload {
		std::vector<uint8_t> data;
		std::vector<int32_t> fds;
	};

	IPCUnixSocket();
	~IPCUnixSocket();

	UniqueFD create();
	int bind(UniqueFD fd);
	void close();
	bool isBound() const;

	int send(const Payload &payload);
	int receive(Payload *payload);

	Signal<> readyRead;

private:
	struct Header {
		uint32_t data;
		uint8_t fds;
	};

	int sendData(const void *buffer, size_t length, const int32_t *fds, unsigned int num);
	int recvData(void *buffer, size_t length, int32_t *fds, unsigned int num);

	void dataNotifier();

	UniqueFD fd_;
	bool headerReceived_;
	struct Header header_;
	EventNotifier *notifier_;
};

} /* namespace libcamera */
