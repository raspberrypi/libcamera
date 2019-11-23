/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamera.c - GStreamer plugin
 */

#include "gstlibcamerasrc.h"

static gboolean
plugin_init(GstPlugin *plugin)
{
	return gst_element_register(plugin, "libcamerasrc", GST_RANK_PRIMARY,
				    GST_TYPE_LIBCAMERA_SRC);
	return TRUE;
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR,
		  libcamera, "libcamera capture plugin",
		  plugin_init, VERSION, "LGPL", PACKAGE, "https://libcamera.org");
