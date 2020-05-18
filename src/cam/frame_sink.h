/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * frame_sink.h - Base Frame Sink Class
 */
#ifndef __CAM_FRAME_SINK_H__
#define __CAM_FRAME_SINK_H__

#include <libcamera/base/signal.h>

namespace libcamera {
class CameraConfiguration;
class FrameBuffer;
class Request;
} /* namespace libcamera */

class FrameSink
{
public:
	virtual ~FrameSink();

	virtual int configure(const libcamera::CameraConfiguration &config);

	virtual void mapBuffer(libcamera::FrameBuffer *buffer);

	virtual int start();
	virtual int stop();

	virtual bool processRequest(libcamera::Request *request) = 0;
	libcamera::Signal<libcamera::Request *> requestProcessed;
};

#endif /* __CAM_FRAME_SINK_H__ */
