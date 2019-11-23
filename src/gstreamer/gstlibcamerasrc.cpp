/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamerasrc.cpp - GStreamer Capture Element
 */

#include "gstlibcamerasrc.h"

struct _GstLibcameraSrc {
	GstElement parent;
};

G_DEFINE_TYPE(GstLibcameraSrc, gst_libcamera_src, GST_TYPE_ELEMENT);

static void
gst_libcamera_src_init(GstLibcameraSrc *self)
{
}

static void
gst_libcamera_src_class_init(GstLibcameraSrcClass *klass)
{
	GstElementClass *element_class = (GstElementClass *)klass;

	gst_element_class_set_metadata(element_class,
				       "libcamera Source", "Source/Video",
				       "Linux Camera source using libcamera",
				       "Nicolas Dufresne <nicolas.dufresne@collabora.com");
}
