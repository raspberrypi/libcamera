/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Vedant Paranjape
 *
 * gstreamer_test.cpp - GStreamer test base class
 */

#ifndef __LIBCAMERA_GSTREAMER_TEST_H__
#define __LIBCAMERA_GSTREAMER_TEST_H__

#include <iostream>
#include <unistd.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/source_paths.h"

#include <gst/gst.h>

class GstreamerTest
{
public:
	GstreamerTest();
	virtual ~GstreamerTest();

protected:
	virtual int createPipeline();
	int startPipeline();
	int processEvent();
	void printError(GstMessage *msg);

	GstElement *pipeline_;
	GstElement *libcameraSrc_;
	int status_;
};

#endif /* __LIBCAMERA_GSTREAMER_TEST_H__ */
