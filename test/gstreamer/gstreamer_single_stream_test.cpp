/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Vedant Paranjape
 *
 * GStreamer single stream capture test
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

		fakesink_ = gst_element_factory_make("fakesink", nullptr);
		if (!fakesink_) {
			g_printerr("Your installation is missing 'fakesink'\n");
			return TestFail;
		}
		g_object_ref_sink(fakesink_);

		return createPipeline();
	}

	int run() override
	{
		/* Build the pipeline */
		gst_bin_add_many(GST_BIN(pipeline_), libcameraSrc_, fakesink_, nullptr);
		if (!gst_element_link(libcameraSrc_, fakesink_)) {
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
		g_clear_object(&fakesink_);
	}

private:
	GstElement *fakesink_;
};

TEST_REGISTER(GstreamerSingleStreamTest)
