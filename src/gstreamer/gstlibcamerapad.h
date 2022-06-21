/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamerapad.h - GStreamer Capture Element
 */

#pragma once

#include "gstlibcamerapool.h"

#include <gst/gst.h>

#include <libcamera/stream.h>

#define GST_TYPE_LIBCAMERA_PAD gst_libcamera_pad_get_type()
G_DECLARE_FINAL_TYPE(GstLibcameraPad, gst_libcamera_pad, GST_LIBCAMERA, PAD, GstPad)

libcamera::StreamRole gst_libcamera_pad_get_role(GstPad *pad);

GstLibcameraPool *gst_libcamera_pad_get_pool(GstPad *pad);

void gst_libcamera_pad_set_pool(GstPad *pad, GstLibcameraPool *pool);

libcamera::Stream *gst_libcamera_pad_get_stream(GstPad *pad);

void gst_libcamera_pad_set_latency(GstPad *pad, GstClockTime latency);
