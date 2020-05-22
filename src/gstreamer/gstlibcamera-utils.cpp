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
	{ GST_VIDEO_FORMAT_ENCODED, formats::MJPEG },
	{ GST_VIDEO_FORMAT_RGB, formats::BGR888 },
	{ GST_VIDEO_FORMAT_BGR, formats::RGB888 },
	{ GST_VIDEO_FORMAT_ARGB, formats::BGRA8888 },
	{ GST_VIDEO_FORMAT_NV12, formats::NV12 },
	{ GST_VIDEO_FORMAT_NV21, formats::NV21 },
	{ GST_VIDEO_FORMAT_NV16, formats::NV16 },
	{ GST_VIDEO_FORMAT_NV61, formats::NV61 },
	{ GST_VIDEO_FORMAT_NV24, formats::NV24 },
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

	/* First fixate the caps using default configuration value. */
	g_assert(gst_caps_is_writable(caps));
	caps = gst_caps_truncate(caps);
	GstStructure *s = gst_caps_get_structure(caps, 0);

	gst_structure_fixate_field_nearest_int(s, "width", stream_cfg.size.width);
	gst_structure_fixate_field_nearest_int(s, "height", stream_cfg.size.height);

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

void
gst_libcamera_resume_task(GstTask *task)
{
	/* We only want to resume the task if it's paused. */
	GLibLocker lock(GST_OBJECT(task));
	if (GST_TASK_STATE(task) == GST_TASK_PAUSED) {
		GST_TASK_STATE(task) = GST_TASK_STARTED;
		GST_TASK_SIGNAL(task);
	}
}
