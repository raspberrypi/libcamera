/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipc_unixsocket.cpp - IPC mechanism based on Unix sockets
 */

#include "libcamera/internal/ipc_unixsocket.h"

#include <array>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <libcamera/base/event_notifier.h>
#include <libcamera/base/log.h>

/**
 * \file ipc_unixsocket.h
 * \brief IPC mechanism based on Unix sockets
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(IPCUnixSocket)

/**
 * \struct IPCUnixSocket::Payload
 * \brief Container for an IPC payload
 *
 * Holds an array of bytes and an array of file descriptors that can be
 * transported across a IPC boundary.
 */

/**
 * \var IPCUnixSocket::Payload::data
 * \brief Array of bytes to cross IPC boundary
 */

/**
 * \var IPCUnixSocket::Payload::fds
 * \brief Array of file descriptors to cross IPC boundary
 */

/**
 * \class IPCUnixSocket
 * \brief IPC mechanism based on Unix sockets
 *
 * The Unix socket IPC allows bidirectional communication between two processes
 * through unnamed Unix sockets. It implements datagram-based communication,
 * transporting entire payloads with guaranteed ordering.
 *
 * The IPC design is asynchronous, a message is queued to a receiver which gets
 * notified that a message is ready to be consumed by the \ref readyRead
 * signal. The sender of the message gets no notification when a message is
 * delivered nor processed. If such interactions are needed a protocol specific
 * to the users use-case should be implemented on top of the IPC objects.
 *
 * Establishment of an IPC channel is asymmetrical. The side that initiates
 * communication first instantiates a local side socket and creates the channel
 * with create(). The function returns a file descriptor for the remote side of
 * the channel, which is passed to the remote process through an out-of-band
 * communication method. The remote side then instantiates a socket, and binds
 * it to the other side by passing the file descriptor to bind(). At that point
 * the channel is operation and communication is bidirectional and symmmetrical.
 *
 * \context This class is \threadbound.
 */

IPCUnixSocket::IPCUnixSocket()
	: headerReceived_(false), notifier_(nullptr)
{
}

IPCUnixSocket::~IPCUnixSocket()
{
	close();
}

/**
 * \brief Create an new IPC channel
 *
 * This function creates a new IPC channel. The socket instance is bound to the
 * local side of the channel, and the function returns a file descriptor bound
 * to the remote side. The caller is responsible for passing the file descriptor
 * to the remote process, where it can be used with IPCUnixSocket::bind() to
 * bind the remote side socket.
 *
 * \return A file descriptor. It is valid on success or invalid otherwise.
 */
UniqueFD IPCUnixSocket::create()
{
	int sockets[2];
	int ret;

	ret = socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0, sockets);
	if (ret) {
		ret = -errno;
		LOG(IPCUnixSocket, Error)
			<< "Failed to create socket pair: " << strerror(-ret);
		return {};
	}

	std::array<UniqueFD, 2> socketFds{
		UniqueFD(sockets[0]),
		UniqueFD(sockets[1]),
	};

	if (bind(std::move(socketFds[0])) < 0)
		return {};

	return std::move(socketFds[1]);
}

/**
 * \brief Bind to an existing IPC channel
 * \param[in] fd File descriptor
 *
 * This function binds the socket instance to an existing IPC channel identified
 * by the file descriptor \a fd. The file descriptor is obtained from the
 * IPCUnixSocket::create() function.
 *
 * \return 0 on success or a negative error code otherwise
 */
int IPCUnixSocket::bind(UniqueFD fd)
{
	if (isBound())
		return -EINVAL;

	fd_ = std::move(fd);
	notifier_ = new EventNotifier(fd_.get(), EventNotifier::Read);
	notifier_->activated.connect(this, &IPCUnixSocket::dataNotifier);

	return 0;
}

/**
 * \brief Close the IPC channel
 *
 * No communication is possible after close() has been called.
 */
void IPCUnixSocket::close()
{
	if (!isBound())
		return;

	delete notifier_;
	notifier_ = nullptr;

	fd_.reset();
	headerReceived_ = false;
}

/**
 * \brief Check if the IPC channel is bound
 * \return True if the IPC channel is bound, false otherwise
 */
bool IPCUnixSocket::isBound() const
{
	return fd_.isValid();
}

/**
 * \brief Send a message payload
 * \param[in] payload Message payload to send
 *
 * This function queues the message payload for transmission to the other end of
 * the IPC channel. It returns immediately, before the message is delivered to
 * the remote side.
 *
 * \return 0 on success or a negative error code otherwise
 */
int IPCUnixSocket::send(const Payload &payload)
{
	int ret;

	if (!isBound())
		return -ENOTCONN;

	Header hdr = {};
	hdr.data = payload.data.size();
	hdr.fds = payload.fds.size();

	if (!hdr.data && !hdr.fds)
		return -EINVAL;

	ret = ::send(fd_.get(), &hdr, sizeof(hdr), 0);
	if (ret < 0) {
		ret = -errno;
		LOG(IPCUnixSocket, Error)
			<< "Failed to send: " << strerror(-ret);
		return ret;
	}

	return sendData(payload.data.data(), hdr.data, payload.fds.data(), hdr.fds);
}

/**
 * \brief Receive a message payload
 * \param[out] payload Payload where to write the received message
 *
 * This function receives the message payload from the IPC channel and writes it
 * to the \a payload. If no message payload is available, it returns
 * immediately with -EAGAIN. The \ref readyRead signal shall be used to receive
 * notification of message availability.
 *
 * \todo Add state machine to make sure we don't block forever and that
 * a header is always followed by a payload.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EAGAIN No message payload is available
 * \retval -ENOTCONN The socket is not connected (neither create() nor bind()
 * has been called)
 */
int IPCUnixSocket::receive(Payload *payload)
{
	if (!isBound())
		return -ENOTCONN;

	if (!headerReceived_)
		return -EAGAIN;

	payload->data.resize(header_.data);
	payload->fds.resize(header_.fds);

	int ret = recvData(payload->data.data(), header_.data,
			   payload->fds.data(), header_.fds);
	if (ret < 0)
		return ret;

	headerReceived_ = false;
	notifier_->setEnabled(true);

	return 0;
}

/**
 * \var IPCUnixSocket::readyRead
 * \brief A Signal emitted when a message is ready to be read
 */

int IPCUnixSocket::sendData(const void *buffer, size_t length,
			    const int32_t *fds, unsigned int num)
{
	struct iovec iov[1];
	iov[0].iov_base = const_cast<void *>(buffer);
	iov[0].iov_len = length;

	char buf[CMSG_SPACE(num * sizeof(uint32_t))];
	memset(buf, 0, sizeof(buf));

	struct cmsghdr *cmsg = (struct cmsghdr *)buf;
	cmsg->cmsg_len = CMSG_LEN(num * sizeof(uint32_t));
	cmsg->cmsg_level = SOL_SOCKET;
	cmsg->cmsg_type = SCM_RIGHTS;

	struct msghdr msg;
	msg.msg_name = nullptr;
	msg.msg_namelen = 0;
	msg.msg_iov = iov;
	msg.msg_iovlen = 1;
	msg.msg_control = cmsg;
	msg.msg_controllen = cmsg->cmsg_len;
	msg.msg_flags = 0;
	if (fds)
		memcpy(CMSG_DATA(cmsg), fds, num * sizeof(uint32_t));

	if (sendmsg(fd_.get(), &msg, 0) < 0) {
		int ret = -errno;
		LOG(IPCUnixSocket, Error)
			<< "Failed to sendmsg: " << strerror(-ret);
		return ret;
	}

	return 0;
}

int IPCUnixSocket::recvData(void *buffer, size_t length,
			    int32_t *fds, unsigned int num)
{
	struct iovec iov[1];
	iov[0].iov_base = buffer;
	iov[0].iov_len = length;

	char buf[CMSG_SPACE(num * sizeof(uint32_t))];
	memset(buf, 0, sizeof(buf));

	struct cmsghdr *cmsg = (struct cmsghdr *)buf;
	cmsg->cmsg_len = CMSG_LEN(num * sizeof(uint32_t));
	cmsg->cmsg_level = SOL_SOCKET;
	cmsg->cmsg_type = SCM_RIGHTS;

	struct msghdr msg;
	msg.msg_name = nullptr;
	msg.msg_namelen = 0;
	msg.msg_iov = iov;
	msg.msg_iovlen = 1;
	msg.msg_control = cmsg;
	msg.msg_controllen = cmsg->cmsg_len;
	msg.msg_flags = 0;

	if (recvmsg(fd_.get(), &msg, 0) < 0) {
		int ret = -errno;
		if (ret != -EAGAIN)
			LOG(IPCUnixSocket, Error)
				<< "Failed to recvmsg: " << strerror(-ret);
		return ret;
	}

	if (fds)
		memcpy(fds, CMSG_DATA(cmsg), num * sizeof(uint32_t));

	return 0;
}

void IPCUnixSocket::dataNotifier()
{
	int ret;

	if (!headerReceived_) {
		/* Receive the header. */
		ret = ::recv(fd_.get(), &header_, sizeof(header_), 0);
		if (ret < 0) {
			ret = -errno;
			LOG(IPCUnixSocket, Error)
				<< "Failed to receive header: " << strerror(-ret);
			return;
		}

		headerReceived_ = true;
	}

	/*
	 * If the payload has arrived, disable the notifier and emit the
	 * readyRead signal. The notifier will be reenabled by the receive()
	 * function.
	 */
	struct pollfd fds = { fd_.get(), POLLIN, 0 };
	ret = poll(&fds, 1, 0);
	if (ret < 0)
		return;

	if (!(fds.revents & POLLIN))
		return;

	notifier_->setEnabled(false);
	readyRead.emit();
}

} /* namespace libcamera */
