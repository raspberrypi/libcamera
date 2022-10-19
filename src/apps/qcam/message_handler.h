/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * message_handler.cpp - qcam - Log message handling
 */

#pragma once

#include <QtGlobal>

class MessageHandler
{
public:
	MessageHandler(bool verbose);

private:
	static void handleMessage(QtMsgType type,
				  const QMessageLogContext &context,
				  const QString &msg);

	static QtMessageHandler handler_;
	static bool verbose_;
};
