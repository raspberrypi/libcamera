/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Vedant Paranjape
 *
 * gstreamer_single_stream_test.cpp - GStreamer single stream capture test
 */

#include <iostream>
#include <unistd.h>

#include <gst/gst.h>

#include "gstreamer_test.h"
#include "test.h"

using namespace std;

class GstreamerSingleStreamTest : public GstreamerTest, public Test
{
public:
	GstreamerSingleStreamTest()
		: GstreamerTest()
	{
	}

protected:
	int init() override
	{
		if (status_ != TestPass)
			return status_;

		const gchar *streamDescription = "videoconvert ! fakesink";
		g_autoptr(GError) error0 = NULL;
		stream0_ = gst_parse_bin_from_description_full(streamDescription, TRUE,
						NULL,
						GST_PARSE_FLAG_FATAL_ERRORS,
						&error0);

		if (!stream0_) {
			g_printerr("Bin could not be created (%s)\n", error0->message);
			return TestFail;
		}
		g_object_ref_sink(stream0_);

		if (createPipeline() != TestPass)
			return TestFail;

		return TestPass;
	}

	int run() override
	{
		/* Build the pipeline */
		gst_bin_add_many(GST_BIN(pipeline_), libcameraSrc_, stream0_, NULL);
		if (gst_element_link(libcameraSrc_, stream0_) != TRUE) {
			g_printerr("Elements could not be linked.\n");
			return TestFail;
		}

		if (startPipeline() != TestPass)
			return TestFail;

		if (processEvent() != TestPass)
			return TestFail;

		return TestPass;
	}

	void cleanup() override
	{
		g_clear_object(&stream0_);
	}

private:
	GstElement *stream0_;
};

TEST_REGISTER(GstreamerSingleStreamTest)
