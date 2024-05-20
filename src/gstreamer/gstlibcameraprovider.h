/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer Device Provider
 */

#pragma once

#include <gst/gst.h>

G_BEGIN_DECLS

#define GST_TYPE_LIBCAMERA_PROVIDER gst_libcamera_provider_get_type()
G_DECLARE_FINAL_TYPE(GstLibcameraProvider, gst_libcamera_provider,
		     GST_LIBCAMERA, PROVIDER, GstDeviceProvider)

G_END_DECLS
