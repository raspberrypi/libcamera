/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamera-utils.h - GStreamer libcamera Utility Functions
 */

#ifndef __GST_LIBCAMERA_UTILS_H__
#define __GST_LIBCAMERA_UTILS_H__

#include <gst/gst.h>
#include <gst/video/video.h>

#include <libcamera/stream.h>

GstCaps *gst_libcamera_stream_formats_to_caps(const libcamera::StreamFormats &formats);

#endif /* __GST_LIBCAMERA_UTILS_H__ */
