/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2023, Umang Jain <umang.jain@ideasonboard.com>
 *
 * GStreamer single stream capture test
 */

#include <vector>

#include <libcamera/libcamera.h>
#include <gst/gst.h>

#include "gstreamer_test.h"
#include "test.h"

using namespace std;

class GstreamerDeviceProviderTest : public GstreamerTest, public Test
{
public:
	GstreamerDeviceProviderTest()
		: GstreamerTest()
	{
	}

protected:
	int init() override
	{
		if (status_ != TestPass)
			return status_;

		return TestPass;
	}

	int run() override
	{
		g_autoptr(GstDeviceProvider) provider = NULL;
		GList *devices, *l;
		std::vector<std::string> cameraNames;
		std::unique_ptr<libcamera::CameraManager> cm;

		cm = std::make_unique<libcamera::CameraManager>();
		cm->start();
		for (auto &camera : cm->cameras())
			cameraNames.push_back(camera->id());
		cm->stop();
		cm.reset();

		provider = gst_device_provider_factory_get_by_name("libcameraprovider");
		devices = gst_device_provider_get_devices(provider);

		for (l = devices; l != NULL; l = g_list_next(l)) {
			GstDevice *device = GST_DEVICE(l->data);
			g_autofree gchar *gst_name;

			g_autoptr(GstElement) element = gst_device_create_element(device, NULL);
			g_object_get(element, "camera-name", &gst_name, NULL);

			if (std::find(cameraNames.begin(), cameraNames.end(), gst_name) ==
			    cameraNames.end())
				return TestFail;
		}

		g_list_free_full(devices, (GDestroyNotify)gst_object_unref);

		return TestPass;
	}
};

TEST_REGISTER(GstreamerDeviceProviderTest)
