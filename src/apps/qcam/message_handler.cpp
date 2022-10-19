/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * message_handler.cpp - qcam - Log message handling
 */

#include "message_handler.h"

QtMessageHandler MessageHandler::handler_ = nullptr;
bool MessageHandler::verbose_ = false;

MessageHandler::MessageHandler(bool verbose)
{
	verbose_ = verbose;
	handler_ = qInstallMessageHandler(&MessageHandler::handleMessage);
}

void MessageHandler::handleMessage(QtMsgType type,
				   const QMessageLogContext &context,
				   const QString &msg)
{
	if (type == QtDebugMsg && !verbose_)
		return;

	handler_(type, context, msg);
}
