/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Vedant Paranjape
 *
 * gstreamer_single_stream_test.cpp - GStreamer single stream capture test
 */

#include <iostream>
#include <unistd.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/source_paths.h"

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

		g_autoptr(GstElement) convert0 = gst_element_factory_make("videoconvert", "convert0");
		g_autoptr(GstElement) sink0 = gst_element_factory_make("fakesink", "sink0");
		g_object_ref_sink(convert0);
		g_object_ref_sink(sink0);

		if (!convert0 || !sink0) {
			g_printerr("Not all elements could be created. %p.%p\n",
				   convert0, sink0);

			return TestFail;
		}

		convert0_ = reinterpret_cast<GstElement *>(g_steal_pointer(&convert0));
		sink0_ = reinterpret_cast<GstElement *>(g_steal_pointer(&sink0));

		if (createPipeline() != TestPass)
			return TestFail;

		return TestPass;
	}

	int run() override
	{
		/* Build the pipeline */
		gst_bin_add_many(GST_BIN(pipeline_), libcameraSrc_, convert0_, sink0_, NULL);
		if (gst_element_link_many(libcameraSrc_, convert0_, sink0_, NULL) != TRUE) {
			g_printerr("Elements could not be linked.\n");
			return TestFail;
		}

		if (startPipeline() != TestPass)
			return TestFail;

		if (processEvent() != TestPass)
			return TestFail;

		return TestPass;
	}

private:
	GstElement *convert0_;
	GstElement *sink0_;
};

TEST_REGISTER(GstreamerSingleStreamTest)
