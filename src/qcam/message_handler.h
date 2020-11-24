/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * message_handler.cpp - qcam - Log message handling
 */
#ifndef __QCAM_MESSAGE_HANDLER_H__
#define __QCAM_MESSAGE_HANDLER_H__

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

#endif /* __QCAM_MESSAGE_HANDLER_H__ */
