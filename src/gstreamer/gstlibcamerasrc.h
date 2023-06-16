/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamerasrc.h - GStreamer Capture Element
 */

#pragma once

#include <libcamera/control_ids.h>

#include <gst/gst.h>

G_BEGIN_DECLS

#define GST_TYPE_LIBCAMERA_SRC gst_libcamera_src_get_type()
G_DECLARE_FINAL_TYPE(GstLibcameraSrc, gst_libcamera_src,
		     GST_LIBCAMERA, SRC, GstElement)

G_END_DECLS

inline GType
gst_libcamera_auto_focus_get_type()
{
	static GType type = 0;
	static const GEnumValue values[] = {
		{
			static_cast<gint>(libcamera::controls::AfModeManual),
			"AfModeManual",
			"manual-focus",
		},
		{
			static_cast<gint>(libcamera::controls::AfModeAuto),
			"AfModeAuto",
			"automatic-auto-focus",
		},
		{
			static_cast<gint>(libcamera::controls::AfModeContinuous),
			"AfModeContinuous",
			"continuous-auto-focus",
		},
		{ 0, NULL, NULL }
	};

	if (!type)
		type = g_enum_register_static("GstLibcameraAutoFocus", values);

	return type;
}
