/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipc_unixsocket.h - IPC mechanism based on Unix sockets
 */

#ifndef __LIBCAMERA_INTERNAL_IPC_UNIXSOCKET_H__
#define __LIBCAMERA_INTERNAL_IPC_UNIXSOCKET_H__

#include <stdint.h>
#include <sys/types.h>
#include <vector>

#include <libcamera/event_notifier.h>

namespace libcamera {

class IPCUnixSocket
{
public:
	struct Payload {
		std::vector<uint8_t> data;
		std::vector<int32_t> fds;
	};

	IPCUnixSocket();
	~IPCUnixSocket();

	int create();
	int bind(int fd);
	void close();
	bool isBound() const;

	int send(const Payload &payload);
	int receive(Payload *payload);

	Signal<IPCUnixSocket *> readyRead;

private:
	struct Header {
		uint32_t data;
		uint8_t fds;
	};

	int sendData(const void *buffer, size_t length, const int32_t *fds, unsigned int num);
	int recvData(void *buffer, size_t length, int32_t *fds, unsigned int num);

	void dataNotifier(EventNotifier *notifier);

	int fd_;
	bool headerReceived_;
	struct Header header_;
	EventNotifier *notifier_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_IPC_UNIXSOCKET_H__ */
