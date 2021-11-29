/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipc_pipe.cpp - Image Processing Algorithm IPC module for IPA proxies
 */

#include "libcamera/internal/ipc_pipe.h"

#include <libcamera/base/log.h>

/**
 * \file ipc_pipe.h
 * \brief IPC mechanism for IPA isolation
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(IPCPipe)

/**
 * \struct IPCMessage::Header
 * \brief Container for an IPCMessage header
 *
 * Holds a cmd code for the IPC message, and a cookie.
 */

/**
 * \var IPCMessage::Header::cmd
 * \brief Type of IPCMessage
 *
 * Typically used to carry a command code for an RPC.
 */

/**
 * \var IPCMessage::Header::cookie
 * \brief Cookie to identify the message and a corresponding reply.
 *
 * Populated and used by IPCPipe implementations for matching calls with
 * replies.
 */

/**
 * \class IPCMessage
 * \brief IPC message to be passed through IPC message pipe
 */

/**
 * \brief Construct an empty IPCMessage instance
 */
IPCMessage::IPCMessage()
	: header_(Header{ 0, 0 })
{
}

/**
 * \brief Construct an IPCMessage instance with a given command code
 * \param[in] cmd The command code
 */
IPCMessage::IPCMessage(uint32_t cmd)
	: header_(Header{ cmd, 0 })
{
}

/**
 * \brief Construct an IPCMessage instance with a given header
 * \param[in] header The header that the constructed IPCMessage will contain
 */
IPCMessage::IPCMessage(const Header &header)
	: header_(header)
{
}

/**
 * \brief Construct an IPCMessage instance from an IPC payload
 * \param[in] payload The IPCUnixSocket payload to construct from
 *
 * This essentially converts an IPCUnixSocket payload into an IPCMessage.
 * The header is extracted from the payload into the IPCMessage's header field.
 *
 * If the IPCUnixSocket payload had any valid file descriptors, then they will
 * all be invalidated.
 */
IPCMessage::IPCMessage(IPCUnixSocket::Payload &payload)
{
	memcpy(&header_, payload.data.data(), sizeof(header_));
	data_ = std::vector<uint8_t>(payload.data.begin() + sizeof(header_),
				     payload.data.end());
	for (int32_t &fd : payload.fds)
		fds_.push_back(SharedFD(std::move(fd)));
}

/**
 * \brief Create an IPCUnixSocket payload from the IPCMessage
 *
 * This essentially converts the IPCMessage into an IPCUnixSocket payload.
 *
 * \todo Resolve the layering violation (add other converters later?)
 */
IPCUnixSocket::Payload IPCMessage::payload() const
{
	IPCUnixSocket::Payload payload;

	payload.data.resize(sizeof(Header) + data_.size());
	payload.fds.reserve(fds_.size());

	memcpy(payload.data.data(), &header_, sizeof(Header));

	if (data_.size() > 0) {
		/* \todo Make this work without copy */
		memcpy(payload.data.data() + sizeof(Header),
		       data_.data(), data_.size());
	}

	for (const SharedFD &fd : fds_)
		payload.fds.push_back(fd.get());

	return payload;
}

/**
 * \fn IPCMessage::header()
 * \brief Returns a reference to the header
 */

/**
 * \fn IPCMessage::data()
 * \brief Returns a reference to the byte vector containing data
 */

/**
 * \fn IPCMessage::fds()
 * \brief Returns a reference to the vector containing file descriptors
 */

/**
 * \fn IPCMessage::header() const
 * \brief Returns a const reference to the header
 */

/**
 * \fn IPCMessage::data() const
 * \brief Returns a const reference to the byte vector containing data
 */

/**
 * \fn IPCMessage::fds() const
 * \brief Returns a const reference to the vector containing file descriptors
 */

/**
 * \class IPCPipe
 * \brief IPC message pipe for IPA isolation
 *
 * Virtual class to model an IPC message pipe for use by IPA proxies for IPA
 * isolation. sendSync() and sendAsync() must be implemented, and the recvMessage
 * signal must be emitted whenever new data is available.
 */

/**
 * \brief Construct an IPCPipe instance
 */
IPCPipe::IPCPipe()
	: connected_(false)
{
}

IPCPipe::~IPCPipe()
{
}

/**
 * \fn IPCPipe::isConnected()
 * \brief Check if the IPCPipe instance is connected
 *
 * An IPCPipe instance is connected if IPC is successfully set up.
 *
 * \return True if the IPCPipe is connected, false otherwise
 */

/**
 * \fn IPCPipe::sendSync()
 * \brief Send a message over IPC synchronously
 * \param[in] in Data to send
 * \param[in] out IPCMessage instance in which to receive data, if applicable
 *
 * This function will not return until a response is received. The event loop
 * will still continue to execute, however.
 *
 * \return Zero on success, negative error code otherwise
 *
 * \todo Determine if the event loop should limit the types of messages it
 * processes, to avoid reintrancy in the caller, and carefully document what
 * the caller needs to implement to make this safe.
 */

/**
 * \fn IPCPipe::sendAsync()
 * \brief Send a message over IPC asynchronously
 * \param[in] data Data to send
 *
 * This function will return immediately after sending the message.
 *
 * \return Zero on success, negative error code otherwise
 */

/**
 * \var IPCPipe::recv
 * \brief Signal to be emitted when a message is received over IPC
 *
 * When a message is received over IPC, this signal shall be emitted. Users must
 * connect to this to receive messages.
 */

/**
 * \var IPCPipe::connected_
 * \brief Flag to indicate if the IPCPipe instance is connected
 *
 * An IPCPipe instance is connected if IPC is successfully set up.
 *
 * This flag can be read via IPCPipe::isConnected().
 *
 * Implementations of the IPCPipe class should set this flag upon successful
 * connection.
 */

} /* namespace libcamera */
