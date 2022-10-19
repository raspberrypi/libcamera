/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * frame_sink.cpp - Base Frame Sink Class
 */

#include "frame_sink.h"

/**
 * \class FrameSink
 * \brief Abstract class to model a consumer of frames
 *
 * The FrameSink class models the consumer that processes frames after a request
 * completes. It receives requests through processRequest(), and processes them
 * synchronously or asynchronously. This allows frame sinks to hold onto frames
 * for an extended period of time, for instance to display them until a new
 * frame arrives.
 *
 * A frame sink processes whole requests, and is solely responsible for deciding
 * how to handle different frame buffers in case multiple streams are captured.
 */

FrameSink::~FrameSink()
{
}

int FrameSink::configure([[maybe_unused]] const libcamera::CameraConfiguration &config)
{
	return 0;
}

void FrameSink::mapBuffer([[maybe_unused]] libcamera::FrameBuffer *buffer)
{
}

int FrameSink::start()
{
	return 0;
}

int FrameSink::stop()
{
	return 0;
}

/**
 * \fn FrameSink::processRequest()
 * \param[in] request The request
 *
 * This function is called to instruct the sink to process a request. The sink
 * may process the request synchronously or queue it for asynchronous
 * processing.
 *
 * When the request is processed synchronously, this function shall return true.
 * The \a request shall not be accessed by the FrameSink after the function
 * returns.
 *
 * When the request is processed asynchronously, the FrameSink temporarily takes
 * ownership of the \a request. The function shall return false, and the
 * FrameSink shall emit the requestProcessed signal when the request processing
 * completes. If the stop() function is called before the request processing
 * completes, it shall release the request synchronously.
 *
 * \return True if the request has been processed synchronously, false if
 * processing has been queued
 */
