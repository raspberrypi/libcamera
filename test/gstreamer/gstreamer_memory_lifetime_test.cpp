/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Nicolas Dufresne
 *
 * gstreamer_memory_lifetime_test.cpp - GStreamer memory lifetime test
 */

#include <iostream>
#include <unistd.h>

#include <gst/app/app.h>
#include <gst/gst.h>

#include "gstreamer_test.h"
#include "test.h"

using namespace std;

class GstreamerMemoryLifetimeTest : public GstreamerTest, public Test
{
public:
	GstreamerMemoryLifetimeTest()
		: GstreamerTest()
	{
	}

protected:
	int init() override
	{
		if (status_ != TestPass)
			return status_;

		appsink_ = gst_element_factory_make("appsink", nullptr);
		if (!appsink_) {
			g_printerr("Your installation is missing 'appsink'\n");
			return TestFail;
		}
		g_object_ref_sink(appsink_);

		return createPipeline();
	}

	int run() override
	{
		/* Build the pipeline */
		gst_bin_add_many(GST_BIN(pipeline_), libcameraSrc_, appsink_, nullptr);
		if (gst_element_link(libcameraSrc_, appsink_) != TRUE) {
			g_printerr("Elements could not be linked.\n");
			return TestFail;
		}

		if (startPipeline() != TestPass)
			return TestFail;

		sample_ = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), GST_SECOND * 5);
		if (!sample_) {
			/* Failed to obtain a sample. Abort the test */
			gst_element_set_state(pipeline_, GST_STATE_NULL);
			return TestFail;
		}

		/*
		 * Keep the sample referenced and set the pipeline state to
		 * NULL. This causes the libcamerasrc element to synchronously
		 * release resources it holds. The sample will be released
		 * later in cleanup().
		 *
		 * The test case verifies that libcamerasrc keeps alive long
		 * enough all the resources that are needed until memory
		 * allocated for frames gets freed. We return TestPass at this
		 * stage, and any use-after-free will be caught by the test
		 * crashing in cleanup().
		 */
		gst_element_set_state(pipeline_, GST_STATE_NULL);

		return TestPass;
	}

	void cleanup() override
	{
		g_clear_pointer(&sample_, gst_sample_unref);
		g_clear_object(&appsink_);
	}

private:
	GstElement *appsink_;
	GstSample *sample_;
};

TEST_REGISTER(GstreamerMemoryLifetimeTest)
