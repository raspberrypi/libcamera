/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamera-utils.c - GStreamer libcamera Utility Function
 */

#include "gstlibcamera-utils.h"

#include <libcamera/formats.h>

using namespace libcamera;

static struct {
	GstVideoFormat gst_format;
	PixelFormat format;
} format_map[] = {
	/* Compressed */
	{ GST_VIDEO_FORMAT_ENCODED, formats::MJPEG },

	/* RGB16 */
	{ GST_VIDEO_FORMAT_RGB16, formats::RGB565 },

	/* RGB24 */
	{ GST_VIDEO_FORMAT_RGB, formats::BGR888 },
	{ GST_VIDEO_FORMAT_BGR, formats::RGB888 },

	/* RGB32 */
	{ GST_VIDEO_FORMAT_BGRx, formats::XRGB8888 },
	{ GST_VIDEO_FORMAT_RGBx, formats::XBGR8888 },
	{ GST_VIDEO_FORMAT_xBGR, formats::RGBX8888 },
	{ GST_VIDEO_FORMAT_xRGB, formats::BGRX8888 },
	{ GST_VIDEO_FORMAT_BGRA, formats::ARGB8888 },
	{ GST_VIDEO_FORMAT_RGBA, formats::ABGR8888 },
	{ GST_VIDEO_FORMAT_ABGR, formats::RGBA8888 },
	{ GST_VIDEO_FORMAT_ARGB, formats::BGRA8888 },

	/* YUV Semiplanar */
	{ GST_VIDEO_FORMAT_NV12, formats::NV12 },
	{ GST_VIDEO_FORMAT_NV21, formats::NV21 },
	{ GST_VIDEO_FORMAT_NV16, formats::NV16 },
	{ GST_VIDEO_FORMAT_NV61, formats::NV61 },
	{ GST_VIDEO_FORMAT_NV24, formats::NV24 },

	/* YUV Planar */
	{ GST_VIDEO_FORMAT_I420, formats::YUV420 },
	{ GST_VIDEO_FORMAT_YV12, formats::YVU420 },
	{ GST_VIDEO_FORMAT_Y42B, formats::YUV422 },

	/* YUV Packed */
	{ GST_VIDEO_FORMAT_UYVY, formats::UYVY },
	{ GST_VIDEO_FORMAT_VYUY, formats::VYUY },
	{ GST_VIDEO_FORMAT_YUY2, formats::YUYV },
	{ GST_VIDEO_FORMAT_YVYU, formats::YVYU },

	/* \todo NV42 is used in libcamera but is not mapped in GStreamer yet. */
};

static GstVideoFormat
pixel_format_to_gst_format(const PixelFormat &format)
{
	for (const auto &item : format_map) {
		if (item.format == format)
			return item.gst_format;
	}
	return GST_VIDEO_FORMAT_UNKNOWN;
}

static PixelFormat
gst_format_to_pixel_format(GstVideoFormat gst_format)
{
	if (gst_format == GST_VIDEO_FORMAT_ENCODED)
		return PixelFormat{};

	for (const auto &item : format_map)
		if (item.gst_format == gst_format)
			return item.format;
	return PixelFormat{};
}

static GstStructure *
bare_structure_from_format(const PixelFormat &format)
{
	GstVideoFormat gst_format = pixel_format_to_gst_format(format);

	if (gst_format == GST_VIDEO_FORMAT_UNKNOWN)
		return nullptr;

	if (gst_format != GST_VIDEO_FORMAT_ENCODED)
		return gst_structure_new("video/x-raw", "format", G_TYPE_STRING,
					 gst_video_format_to_string(gst_format), nullptr);

	switch (format) {
	case formats::MJPEG:
		return gst_structure_new_empty("image/jpeg");
	default:
		return nullptr;
	}
}

GstCaps *
gst_libcamera_stream_formats_to_caps(const StreamFormats &formats)
{
	GstCaps *caps = gst_caps_new_empty();

	for (PixelFormat pixelformat : formats.pixelformats()) {
		g_autoptr(GstStructure) bare_s = bare_structure_from_format(pixelformat);

		if (!bare_s) {
			GST_WARNING("Unsupported DRM format %" GST_FOURCC_FORMAT,
				    GST_FOURCC_ARGS(pixelformat));
			continue;
		}

		for (const Size &size : formats.sizes(pixelformat)) {
			GstStructure *s = gst_structure_copy(bare_s);
			gst_structure_set(s,
					  "width", G_TYPE_INT, size.width,
					  "height", G_TYPE_INT, size.height,
					  nullptr);
			gst_caps_append_structure(caps, s);
		}

		const SizeRange &range = formats.range(pixelformat);
		if (range.hStep && range.vStep) {
			GstStructure *s = gst_structure_copy(bare_s);
			GValue val = G_VALUE_INIT;

			g_value_init(&val, GST_TYPE_INT_RANGE);
			gst_value_set_int_range_step(&val, range.min.width, range.max.width, range.hStep);
			gst_structure_set_value(s, "width", &val);
			gst_value_set_int_range_step(&val, range.min.height, range.max.height, range.vStep);
			gst_structure_set_value(s, "height", &val);
			g_value_unset(&val);

			gst_caps_append_structure(caps, s);
		}
	}

	return caps;
}

GstCaps *
gst_libcamera_stream_configuration_to_caps(const StreamConfiguration &stream_cfg)
{
	GstCaps *caps = gst_caps_new_empty();
	GstStructure *s = bare_structure_from_format(stream_cfg.pixelFormat);

	gst_structure_set(s,
			  "width", G_TYPE_INT, stream_cfg.size.width,
			  "height", G_TYPE_INT, stream_cfg.size.height,
			  nullptr);
	gst_caps_append_structure(caps, s);

	return caps;
}

void
gst_libcamera_configure_stream_from_caps(StreamConfiguration &stream_cfg,
					 GstCaps *caps)
{
	GstVideoFormat gst_format = pixel_format_to_gst_format(stream_cfg.pixelFormat);
	guint i;
	gint best_fixed = -1, best_in_range = -1;
	GstStructure *s;

	/*
	 * These are delta weight computed from:
	 *   ABS(width - stream_cfg.size.width) * ABS(height - stream_cfg.size.height)
	 */
	guint best_fixed_delta = G_MAXUINT;
	guint best_in_range_delta = G_MAXUINT;

	/* First fixate the caps using default configuration value. */
	g_assert(gst_caps_is_writable(caps));

	/* Lookup the structure for a close match to the stream_cfg.size */
	for (i = 0; i < gst_caps_get_size(caps); i++) {
		s = gst_caps_get_structure(caps, i);
		gint width, height;
		guint delta;

		if (gst_structure_has_field_typed(s, "width", G_TYPE_INT) &&
		    gst_structure_has_field_typed(s, "height", G_TYPE_INT)) {
			gst_structure_get_int(s, "width", &width);
			gst_structure_get_int(s, "height", &height);

			delta = ABS(width - (gint)stream_cfg.size.width) * ABS(height - (gint)stream_cfg.size.height);

			if (delta < best_fixed_delta) {
				best_fixed_delta = delta;
				best_fixed = i;
			}
		} else {
			gst_structure_fixate_field_nearest_int(s, "width", stream_cfg.size.width);
			gst_structure_fixate_field_nearest_int(s, "height", stream_cfg.size.height);
			gst_structure_get_int(s, "width", &width);
			gst_structure_get_int(s, "height", &height);

			delta = ABS(width - (gint)stream_cfg.size.width) * ABS(height - (gint)stream_cfg.size.height);

			if (delta < best_in_range_delta) {
				best_in_range_delta = delta;
				best_in_range = i;
			}
		}
	}

	/* Prefer reliable fixed value over ranges */
	if (best_fixed >= 0)
		s = gst_caps_get_structure(caps, best_fixed);
	else
		s = gst_caps_get_structure(caps, best_in_range);

	if (gst_structure_has_name(s, "video/x-raw")) {
		const gchar *format = gst_video_format_to_string(gst_format);
		gst_structure_fixate_field_string(s, "format", format);
	}

	/* Then configure the stream with the result. */
	if (gst_structure_has_name(s, "video/x-raw")) {
		const gchar *format = gst_structure_get_string(s, "format");
		gst_format = gst_video_format_from_string(format);
		stream_cfg.pixelFormat = gst_format_to_pixel_format(gst_format);
	} else if (gst_structure_has_name(s, "image/jpeg")) {
		stream_cfg.pixelFormat = formats::MJPEG;
	} else {
		g_critical("Unsupported media type: %s", gst_structure_get_name(s));
	}

	gint width, height;
	gst_structure_get_int(s, "width", &width);
	gst_structure_get_int(s, "height", &height);
	stream_cfg.size.width = width;
	stream_cfg.size.height = height;
}

#if !GST_CHECK_VERSION(1, 17, 1)
gboolean
gst_task_resume(GstTask *task)
{
	/* We only want to resume the task if it's paused. */
	GLibLocker lock(GST_OBJECT(task));
	if (GST_TASK_STATE(task) != GST_TASK_PAUSED)
		return FALSE;

	GST_TASK_STATE(task) = GST_TASK_STARTED;
	GST_TASK_SIGNAL(task);
	return TRUE;
}
#endif

G_LOCK_DEFINE_STATIC(cm_singleton_lock);
static std::weak_ptr<CameraManager> cm_singleton_ptr;

std::shared_ptr<CameraManager>
gst_libcamera_get_camera_manager(int &ret)
{
	std::shared_ptr<CameraManager> cm;

	G_LOCK(cm_singleton_lock);

	cm = cm_singleton_ptr.lock();
	if (!cm) {
		cm = std::make_shared<CameraManager>();
		cm_singleton_ptr = cm;
		ret = cm->start();
	} else {
		ret = 0;
	}

	G_UNLOCK(cm_singleton_lock);

	return cm;
}
