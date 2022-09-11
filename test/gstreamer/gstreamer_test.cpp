/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Vedant Paranjape
 *
 * libcamera Gstreamer element API tests
 */

#include <libcamera/libcamera.h>

#include <libcamera/base/utils.h>

#include "gstreamer_test.h"

#include "test.h"

using namespace std;

extern "C" {
const char *__asan_default_options()
{
	/*
	 * Disable leak detection due to a known global variable initialization
	 * leak in glib's g_quark_init(). This should ideally be handled by
	 * using a suppression file instead of disabling leak detection.
	 */
	return "detect_leaks=false";
}
}

GstreamerTest::GstreamerTest(unsigned int numStreams)
	: pipeline_(nullptr), libcameraSrc_(nullptr)
{
	/*
	* GStreamer by default spawns a process to run the
	* gst-plugin-scanner helper. If libcamera is compiled with ASan
	* enabled, and as GStreamer is most likely not, this causes the
	* ASan link order check to fail when gst-plugin-scanner
	* dlopen()s the plugin as many libraries will have already been
	* loaded by then. Fix this issue by disabling spawning of a
	* child helper process when scanning the build directory for
	* plugins.
	*/
	gst_registry_fork_set_enabled(false);

	/* Initialize GStreamer */
	g_autoptr(GError) errInit = NULL;
	if (!gst_init_check(nullptr, nullptr, &errInit)) {
		g_printerr("Could not initialize GStreamer: %s\n",
			   errInit ? errInit->message : "unknown error");

		status_ = TestFail;
		return;
	}

	/*
	* Remove the system libcamera plugin, if any, and add the
	* plugin from the build directory.
	*/
	GstRegistry *registry = gst_registry_get();
	g_autoptr(GstPlugin) plugin = gst_registry_lookup(registry, "libgstlibcamera.so");
	if (plugin)
		gst_registry_remove_plugin(registry, plugin);

	std::string path = libcamera::utils::libcameraBuildPath() + "src/gstreamer";
	if (!gst_registry_scan_path(registry, path.c_str())) {
		g_printerr("Failed to add plugin to registry\n");

		status_ = TestFail;
		return;
	}

	/*
	 * Atleast one camera should be available with numStreams streams,
	 * otherwise skip the test entirely.
	 */
	if (!checkMinCameraStreamsAndSetCameraName(numStreams)) {
		status_ = TestSkip;
		return;
	}

	status_ = TestPass;
}

bool GstreamerTest::checkMinCameraStreamsAndSetCameraName(unsigned int numStreams)
{
	libcamera::CameraManager cm;
	bool cameraFound = false;

	cm.start();

	for (auto &camera : cm.cameras()) {
		if (camera->streams().size() < numStreams)
			continue;

		cameraFound = true;
		cameraName_ = camera->id();
		break;
	}

	cm.stop();

	return cameraFound;
}

GstreamerTest::~GstreamerTest()
{
	g_clear_object(&pipeline_);
	g_clear_object(&libcameraSrc_);

	gst_deinit();
}

int GstreamerTest::createPipeline()
{
	libcameraSrc_ = gst_element_factory_make("libcamerasrc", "libcamera");
	pipeline_ = gst_pipeline_new("test-pipeline");

	if (!libcameraSrc_ || !pipeline_) {
		g_printerr("Unable to create pipeline %p.%p\n",
			   libcameraSrc_, pipeline_);

		return TestFail;
	}

	g_object_set(libcameraSrc_, "camera-name", cameraName_.c_str(), NULL);
	g_object_ref_sink(libcameraSrc_);

	return TestPass;
}

int GstreamerTest::startPipeline()
{
	GstStateChangeReturn ret;

	/* Start playing */
	ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
	if (ret == GST_STATE_CHANGE_FAILURE) {
		g_printerr("Unable to set the pipeline to the playing state.\n");
		return TestFail;
	}

	return TestPass;
}

int GstreamerTest::processEvent()
{
	/* Wait until error or EOS or timeout after 2 seconds */
	constexpr GstMessageType msgType =
		static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
	constexpr GstClockTime timeout = 2 * GST_SECOND;

	g_autoptr(GstBus) bus = gst_element_get_bus(pipeline_);
	g_autoptr(GstMessage) msg = gst_bus_timed_pop_filtered(bus, timeout, msgType);

	gst_element_set_state(pipeline_, GST_STATE_NULL);

	/* Parse error message */
	if (msg == NULL)
		return TestPass;

	switch (GST_MESSAGE_TYPE(msg)) {
	case GST_MESSAGE_ERROR:
		printError(msg);
		break;
	case GST_MESSAGE_EOS:
		g_print("End-Of-Stream reached.\n");
		break;
	default:
		g_printerr("Unexpected message received.\n");
		break;
	}

	return TestFail;
}

void GstreamerTest::printError(GstMessage *msg)
{
	g_autoptr(GError) err = NULL;
	g_autofree gchar *debug_info = NULL;

	gst_message_parse_error(msg, &err, &debug_info);
	g_printerr("Error received from element %s: %s\n",
		   GST_OBJECT_NAME(msg->src), err->message);
	g_printerr("Debugging information: %s\n",
		   debug_info ? debug_info : "none");
}
