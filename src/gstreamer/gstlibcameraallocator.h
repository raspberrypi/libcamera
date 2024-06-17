/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer Custom Allocator
 */

#pragma once

#include <gst/gst.h>
#include <gst/allocators/allocators.h>

#include <libcamera/camera.h>
#include <libcamera/stream.h>

#define GST_TYPE_LIBCAMERA_ALLOCATOR gst_libcamera_allocator_get_type()
G_DECLARE_FINAL_TYPE(GstLibcameraAllocator, gst_libcamera_allocator,
		     GST_LIBCAMERA, ALLOCATOR, GstDmaBufAllocator)

GstLibcameraAllocator *gst_libcamera_allocator_new(std::shared_ptr<libcamera::Camera> camera,
						   libcamera::CameraConfiguration *config_);

bool gst_libcamera_allocator_prepare_buffer(GstLibcameraAllocator *self,
					    libcamera::Stream *stream,
					    GstBuffer *buffer);

gsize gst_libcamera_allocator_get_pool_size(GstLibcameraAllocator *allocator,
					    libcamera::Stream *stream);

libcamera::FrameBuffer *gst_libcamera_memory_get_frame_buffer(GstMemory *mem);
