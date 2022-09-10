/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Vedant Paranjape
 *
 * gstreamer_test.cpp - GStreamer test base class
 */

#pragma once

#include <iostream>
#include <unistd.h>

#include <gst/gst.h>

class GstreamerTest
{
public:
	GstreamerTest(unsigned int numStreams = 1);
	virtual ~GstreamerTest();

protected:
	virtual int createPipeline();
	int startPipeline();
	int processEvent();
	void printError(GstMessage *msg);

	std::string cameraName_;
	GstElement *pipeline_;
	GstElement *libcameraSrc_;
	int status_;

private:
	bool checkMinCameraStreamsAndSetCameraName(unsigned int numStreams);
};
