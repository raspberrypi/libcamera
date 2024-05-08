/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer plugin
 */

#include "gstlibcameraprovider.h"
#include "gstlibcamerasrc.h"

static gboolean
plugin_init(GstPlugin *plugin)
{
	if (!gst_element_register(plugin, "libcamerasrc", GST_RANK_PRIMARY,
				  GST_TYPE_LIBCAMERA_SRC) ||
	    !gst_device_provider_register(plugin, "libcameraprovider",
					  GST_RANK_PRIMARY,
					  GST_TYPE_LIBCAMERA_PROVIDER))
		return FALSE;

	return TRUE;
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR,
		  libcamera, "libcamera capture plugin",
		  plugin_init, VERSION, "LGPL", PACKAGE, "https://libcamera.org")
