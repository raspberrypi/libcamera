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

std::atomic_uint Message::nextUserType_{ Message::UserMessage };

/**
 * \class Message
 * \brief A message that can be posted to a Thread
 */

/**
 * \enum Message::Type
 * \brief The message type
 * \var Message::None
 * \brief Invalid message type
 * \var Message::SignalMessage
 * \brief Asynchronous signal delivery across threads
 * \var Message::UserMessage
 * \brief First value available for user-defined messages
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

/**
 * \brief Reserve and register a custom user-defined message type
 *
 * Custom message types use values starting at Message::UserMessage. Assigning
 * custom types manually may lead to accidental duplicated types. To avoid this
 * problem, this method reserves and returns the next available user-defined
 * message type.
 *
 * The recommended way to use this method is to subclass Message and provide a
 * static accessor for the custom message type.
 *
 * \code{.cpp}
 * class MyCustomMessage : public Message
 * {
 * public:
 *	MyCustomMessage() : Message(type()) { }
 *
 *	static Message::Type type()
 *	{
 *		static MessageType type = registerMessageType();
 *		return type;
 *	}
 * };
 * \endcode
 *
 * \return A new unique message type
 */
Message::Type Message::registerMessageType()
{
	return static_cast<Message::Type>(nextUserType_++);
}

/**
 * \class SignalMessage
 * \brief A message carrying a Signal across threads
 */

/**
 * \fn SignalMessage::SignalMessage()
 * \brief Construct a SignalMessage
 * \param[in] slot The slot that the signal targets
 * \param[in] pack The signal arguments
 */

/**
 * \var SignalMessage::slot_
 * \brief The slot that the signal targets
 */

/**
 * \var SignalMessage::pack_
 * \brief The signal arguments
 */

}; /* namespace libcamera */
