/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamerapool.h - GStreamer Buffer Pool
 *
 * This is a partial implementation of GstBufferPool intended for internal use
 * only. This pool cannot be configured or activated.
 */

#pragma once

#include "gstlibcameraallocator.h"

#include <gst/gst.h>

#include <libcamera/stream.h>

#define GST_TYPE_LIBCAMERA_POOL gst_libcamera_pool_get_type()
G_DECLARE_FINAL_TYPE(GstLibcameraPool, gst_libcamera_pool, GST_LIBCAMERA, POOL, GstBufferPool)

GstLibcameraPool *gst_libcamera_pool_new(GstLibcameraAllocator *allocator,
					 libcamera::Stream *stream);

libcamera::Stream *gst_libcamera_pool_get_stream(GstLibcameraPool *self);

libcamera::FrameBuffer *gst_libcamera_buffer_get_frame_buffer(GstBuffer *buffer);
