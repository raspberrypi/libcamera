/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * message.cpp - Message queue support
 */

#include "message.h"

#include "log.h"

/**
 * \file message.h
 * \brief Message queue support
 *
 * The messaging API enables inter-thread communication through message
 * posting. Messages can be sent from any thread to any recipient deriving from
 * the Object class.
 *
 * To post a message, the sender allocates it dynamically as instance of a class
 * derived from Message. It then posts the message to an Object recipient
 * through Object::postMessage(). Message ownership is passed to the object,
 * thus the message shall not store any temporary data.
 *
 * The message is delivered in the context of the object's thread, through the
 * Object::message() virtual method. After delivery the message is
 * automatically deleted.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Message)

/**
 * \class Message
 * \brief A message that can be posted to a Thread
 */

/**
 * \enum Message::Type
 * \brief The message type
 * \var Message::None
 * \brief Invalid message type
 */

/**
 * \brief Construct a message object of type \a type
 * \param[in] type The message type
 */
Message::Message(Message::Type type)
	: type_(type)
{
}

Message::~Message()
{
}

/**
 * \fn Message::type()
 * \brief Retrieve the message type
 * \return The message type
 */

/**
 * \fn Message::receiver()
 * \brief Retrieve the message receiver
 * \return The message receiver
 */

}; /* namespace libcamera */
