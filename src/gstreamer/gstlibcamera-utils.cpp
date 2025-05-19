/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer libcamera Utility Function
 */

#include "gstlibcamera-utils.h"

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

using namespace libcamera;

static struct {
	GstVideoFormat gst_format;
	PixelFormat format;
} format_map[] = {
	/* Compressed */
	{ GST_VIDEO_FORMAT_ENCODED, formats::MJPEG },

	/* Bayer formats, gstreamer only supports 8-bit */
	{ GST_VIDEO_FORMAT_ENCODED, formats::SBGGR8 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGBRG8 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGRBG8 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SRGGB8 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SBGGR10 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGBRG10 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGRBG10 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SRGGB10 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SBGGR12 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGBRG12 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGRBG12 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SRGGB12 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SBGGR14 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGBRG14 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGRBG14 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SRGGB14 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SBGGR16 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGBRG16 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SGRBG16 },
	{ GST_VIDEO_FORMAT_ENCODED, formats::SRGGB16 },

	/* Monochrome */
	{ GST_VIDEO_FORMAT_GRAY8, formats::R8 },
	{ GST_VIDEO_FORMAT_GRAY16_LE, formats::R16 },

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
	{ GST_VIDEO_FORMAT_Y444, formats::YUV444 },

	/* YUV Packed */
	{ GST_VIDEO_FORMAT_UYVY, formats::UYVY },
	{ GST_VIDEO_FORMAT_VYUY, formats::VYUY },
	{ GST_VIDEO_FORMAT_YUY2, formats::YUYV },
	{ GST_VIDEO_FORMAT_YVYU, formats::YVYU },

	/* \todo NV42 is used in libcamera but is not mapped in GStreamer yet. */
};

static GstVideoColorimetry
colorimetry_from_colorspace(const ColorSpace &colorSpace, GstVideoTransferFunction transfer)
{
	GstVideoColorimetry colorimetry;

	switch (colorSpace.primaries) {
	case ColorSpace::Primaries::Raw:
		colorimetry.primaries = GST_VIDEO_COLOR_PRIMARIES_UNKNOWN;
		break;
	case ColorSpace::Primaries::Smpte170m:
		colorimetry.primaries = GST_VIDEO_COLOR_PRIMARIES_SMPTE170M;
		break;
	case ColorSpace::Primaries::Rec709:
		colorimetry.primaries = GST_VIDEO_COLOR_PRIMARIES_BT709;
		break;
	case ColorSpace::Primaries::Rec2020:
		colorimetry.primaries = GST_VIDEO_COLOR_PRIMARIES_BT2020;
		break;
	}

	switch (colorSpace.transferFunction) {
	case ColorSpace::TransferFunction::Linear:
		colorimetry.transfer = GST_VIDEO_TRANSFER_GAMMA10;
		break;
	case ColorSpace::TransferFunction::Srgb:
		colorimetry.transfer = GST_VIDEO_TRANSFER_SRGB;
		break;
	case ColorSpace::TransferFunction::Rec709:
		colorimetry.transfer = GST_VIDEO_TRANSFER_BT709;
		if (transfer != GST_VIDEO_TRANSFER_UNKNOWN)
			colorimetry.transfer = transfer;
		break;
	}

	switch (colorSpace.ycbcrEncoding) {
	case ColorSpace::YcbcrEncoding::None:
		colorimetry.matrix = GST_VIDEO_COLOR_MATRIX_RGB;
		break;
	case ColorSpace::YcbcrEncoding::Rec601:
		colorimetry.matrix = GST_VIDEO_COLOR_MATRIX_BT601;
		break;
	case ColorSpace::YcbcrEncoding::Rec709:
		colorimetry.matrix = GST_VIDEO_COLOR_MATRIX_BT709;
		break;
	case ColorSpace::YcbcrEncoding::Rec2020:
		colorimetry.matrix = GST_VIDEO_COLOR_MATRIX_BT2020;
		break;
	}

	switch (colorSpace.range) {
	case ColorSpace::Range::Full:
		colorimetry.range = GST_VIDEO_COLOR_RANGE_0_255;
		break;
	case ColorSpace::Range::Limited:
		colorimetry.range = GST_VIDEO_COLOR_RANGE_16_235;
		break;
	}

	return colorimetry;
}

static std::optional<ColorSpace>
colorspace_from_colorimetry(const GstVideoColorimetry &colorimetry,
			    GstVideoTransferFunction *transfer)
{
	std::optional<ColorSpace> colorspace = ColorSpace::Raw;

	switch (colorimetry.primaries) {
	case GST_VIDEO_COLOR_PRIMARIES_UNKNOWN:
		/* Unknown primaries map to raw colorspace in gstreamer */
		return ColorSpace::Raw;
	case GST_VIDEO_COLOR_PRIMARIES_SMPTE170M:
		colorspace->primaries = ColorSpace::Primaries::Smpte170m;
		break;
	case GST_VIDEO_COLOR_PRIMARIES_BT709:
		colorspace->primaries = ColorSpace::Primaries::Rec709;
		break;
	case GST_VIDEO_COLOR_PRIMARIES_BT2020:
		colorspace->primaries = ColorSpace::Primaries::Rec2020;
		break;
	default:
		GST_WARNING("Colorimetry primaries %d not mapped in gstlibcamera",
			    colorimetry.primaries);
		return std::nullopt;
	}

	switch (colorimetry.transfer) {
	/* Transfer function mappings inspired from v4l2src plugin */
	case GST_VIDEO_TRANSFER_GAMMA18:
	case GST_VIDEO_TRANSFER_GAMMA20:
	case GST_VIDEO_TRANSFER_GAMMA22:
	case GST_VIDEO_TRANSFER_GAMMA28:
		GST_WARNING("GAMMA 18, 20, 22, 28 transfer functions not supported");
		[[fallthrough]];
	case GST_VIDEO_TRANSFER_GAMMA10:
		colorspace->transferFunction = ColorSpace::TransferFunction::Linear;
		break;
	case GST_VIDEO_TRANSFER_SRGB:
		colorspace->transferFunction = ColorSpace::TransferFunction::Srgb;
		break;
#if GST_CHECK_VERSION(1, 18, 0)
	case GST_VIDEO_TRANSFER_BT601:
	case GST_VIDEO_TRANSFER_BT2020_10:
#endif
	case GST_VIDEO_TRANSFER_BT2020_12:
	case GST_VIDEO_TRANSFER_BT709:
		colorspace->transferFunction = ColorSpace::TransferFunction::Rec709;
		*transfer = colorimetry.transfer;
		break;
	default:
		GST_WARNING("Colorimetry transfer function %d not mapped in gstlibcamera",
			    colorimetry.transfer);
		return std::nullopt;
	}

	switch (colorimetry.matrix) {
	case GST_VIDEO_COLOR_MATRIX_RGB:
		colorspace->ycbcrEncoding = ColorSpace::YcbcrEncoding::None;
		break;
	/* FCC is about the same as BT601 with less digit */
	case GST_VIDEO_COLOR_MATRIX_FCC:
	case GST_VIDEO_COLOR_MATRIX_BT601:
		colorspace->ycbcrEncoding = ColorSpace::YcbcrEncoding::Rec601;
		break;
	case GST_VIDEO_COLOR_MATRIX_BT709:
		colorspace->ycbcrEncoding = ColorSpace::YcbcrEncoding::Rec709;
		break;
	case GST_VIDEO_COLOR_MATRIX_BT2020:
		colorspace->ycbcrEncoding = ColorSpace::YcbcrEncoding::Rec2020;
		break;
	default:
		GST_WARNING("Colorimetry matrix %d not mapped in gstlibcamera",
			    colorimetry.matrix);
		return std::nullopt;
	}

	switch (colorimetry.range) {
	case GST_VIDEO_COLOR_RANGE_0_255:
		colorspace->range = ColorSpace::Range::Full;
		break;
	case GST_VIDEO_COLOR_RANGE_16_235:
		colorspace->range = ColorSpace::Range::Limited;
		break;
	default:
		GST_WARNING("Colorimetry range %d not mapped in gstlibcamera",
			    colorimetry.range);
		return std::nullopt;
	}

	return colorspace;
}

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

static const struct {
	PixelFormat format;
	const gchar *name;
} bayer_map[]{
	{ formats::SBGGR8, "bggr" },
	{ formats::SGBRG8, "gbrg" },
	{ formats::SGRBG8, "grbg" },
	{ formats::SRGGB8, "rggb" },
	{ formats::SBGGR10, "bggr10le" },
	{ formats::SGBRG10, "gbrg10le" },
	{ formats::SGRBG10, "grbg10le" },
	{ formats::SRGGB10, "rggb10le" },
	{ formats::SBGGR12, "bggr12le" },
	{ formats::SGBRG12, "gbrg12le" },
	{ formats::SGRBG12, "grbg12le" },
	{ formats::SRGGB12, "rggb12le" },
	{ formats::SBGGR14, "bggr14le" },
	{ formats::SGBRG14, "gbrg14le" },
	{ formats::SGRBG14, "grbg14le" },
	{ formats::SRGGB14, "rggb14le" },
	{ formats::SBGGR16, "bggr16le" },
	{ formats::SGBRG16, "gbrg16le" },
	{ formats::SGRBG16, "grbg16le" },
	{ formats::SRGGB16, "rggb16le" },
};

static const gchar *
bayer_format_to_string(PixelFormat format)
{
	for (auto &b : bayer_map) {
		if (b.format == format)
			return b.name;
	}
	return nullptr;
}

static PixelFormat
bayer_format_from_string(const gchar *name)
{
	for (auto &b : bayer_map) {
		if (strcmp(b.name, name) == 0)
			return b.format;
	}
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

	case formats::SBGGR8:
	case formats::SGBRG8:
	case formats::SGRBG8:
	case formats::SRGGB8:
		return gst_structure_new("video/x-bayer", "format", G_TYPE_STRING,
					 bayer_format_to_string(format), nullptr);

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
			if (range.min.width == range.max.width) {
				gst_structure_set(s, "width", G_TYPE_INT, range.min.width, nullptr);
			} else {
				gst_value_set_int_range_step(&val, range.min.width, range.max.width, range.hStep);
				gst_structure_set_value(s, "width", &val);
			}
			if (range.min.height == range.max.height) {
				gst_structure_set(s, "height", G_TYPE_INT, range.min.height, nullptr);
			} else {
				gst_value_set_int_range_step(&val, range.min.height, range.max.height, range.vStep);
				gst_structure_set_value(s, "height", &val);
			}
			g_value_unset(&val);

			caps = gst_caps_merge_structure(caps, s);
		}
	}

	return caps;
}

GstCaps *
gst_libcamera_stream_configuration_to_caps(const StreamConfiguration &stream_cfg,
					   GstVideoTransferFunction transfer)
{
	GstCaps *caps = gst_caps_new_empty();
	GstStructure *s = bare_structure_from_format(stream_cfg.pixelFormat);

	gst_structure_set(s,
			  "width", G_TYPE_INT, stream_cfg.size.width,
			  "height", G_TYPE_INT, stream_cfg.size.height,
			  nullptr);

	if (stream_cfg.colorSpace) {
		GstVideoColorimetry colorimetry = colorimetry_from_colorspace(stream_cfg.colorSpace.value(), transfer);
		g_autofree gchar *colorimetry_str = gst_video_colorimetry_to_string(&colorimetry);

		if (colorimetry_str)
			gst_structure_set(s, "colorimetry", G_TYPE_STRING, colorimetry_str, nullptr);
		else
			g_error("Got invalid colorimetry from ColorSpace: %s",
				ColorSpace::toString(stream_cfg.colorSpace).c_str());
	}

	gst_caps_append_structure(caps, s);

	return caps;
}

void gst_libcamera_configure_stream_from_caps(StreamConfiguration &stream_cfg,
					      GstCaps *caps, GstVideoTransferFunction *transfer)
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
	} else if (gst_structure_has_name(s, "video/x-bayer")) {
		const gchar *format = gst_structure_get_string(s, "format");
		stream_cfg.pixelFormat = bayer_format_from_string(format);
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

	/* Configure colorimetry */
	if (gst_structure_has_field(s, "colorimetry")) {
		const gchar *colorimetry_str;
		GstVideoColorimetry colorimetry;

		gst_structure_fixate_field(s, "colorimetry");
		colorimetry_str = gst_structure_get_string(s, "colorimetry");

		if (!gst_video_colorimetry_from_string(&colorimetry, colorimetry_str))
			g_critical("Invalid colorimetry %s", colorimetry_str);

		stream_cfg.colorSpace = colorspace_from_colorimetry(colorimetry, transfer);
	}
}

void gst_libcamera_get_framerate_from_caps(GstCaps *caps,
					   GstStructure *element_caps)
{
	GstStructure *s = gst_caps_get_structure(caps, 0);
	/*
	 * Default to 30 fps. If the "framerate" fraction is invalid below,
	 * libcamerasrc will set 30fps as the framerate.
	 */
	gint fps_n = 30, fps_d = 1;

	if (gst_structure_has_field_typed(s, "framerate", GST_TYPE_FRACTION)) {
		if (!gst_structure_get_fraction(s, "framerate", &fps_n, &fps_d))
			GST_WARNING("Invalid framerate in the caps");
	}

	gst_structure_set(element_caps, "framerate", GST_TYPE_FRACTION,
			  fps_n, fps_d, nullptr);
}

void gst_libcamera_clamp_and_set_frameduration(ControlList &initCtrls,
					       const ControlInfoMap &cam_ctrls,
					       GstStructure *element_caps)
{
	gint fps_caps_n, fps_caps_d;

	if (!gst_structure_has_field_typed(element_caps, "framerate", GST_TYPE_FRACTION))
		return;

	auto iterFrameDuration = cam_ctrls.find(&controls::FrameDurationLimits);
	if (iterFrameDuration == cam_ctrls.end()) {
		GST_WARNING("FrameDurationLimits not found in camera controls.");
		return;
	}

	const GValue *framerate = gst_structure_get_value(element_caps, "framerate");

	fps_caps_n = gst_value_get_fraction_numerator(framerate);
	fps_caps_d = gst_value_get_fraction_denominator(framerate);

	int64_t target_duration = (fps_caps_d * 1000000.0) / fps_caps_n;
	int64_t min_frame_duration = iterFrameDuration->second.min().get<int64_t>();
	int64_t max_frame_duration = iterFrameDuration->second.max().get<int64_t>();

	int64_t frame_duration = std::clamp(target_duration,
					    min_frame_duration,
					    max_frame_duration);

	if (frame_duration != target_duration) {
		gint framerate_clamped = 1000000 / frame_duration;

		/*
		 * Update the clamped framerate which then will be exposed in
		 * downstream caps.
		 */
		gst_structure_set(element_caps, "framerate", GST_TYPE_FRACTION,
				  framerate_clamped, 1, nullptr);
	}

	initCtrls.set(controls::FrameDurationLimits,
		      { frame_duration, frame_duration });
}

void gst_libcamera_framerate_to_caps(GstCaps *caps, const GstStructure *element_caps)
{
	const GValue *framerate = gst_structure_get_value(element_caps, "framerate");
	if (!GST_VALUE_HOLDS_FRACTION(framerate))
		return;

	GstStructure *s = gst_caps_get_structure(caps, 0);
	gint fps_caps_n, fps_caps_d;

	fps_caps_n = gst_value_get_fraction_numerator(framerate);
	fps_caps_d = gst_value_get_fraction_denominator(framerate);

	gst_structure_set(s, "framerate", GST_TYPE_FRACTION, fps_caps_n, fps_caps_d, nullptr);
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

#if !GST_CHECK_VERSION(1, 22, 0)
/*
 * Copyright (C) <1999> Erik Walthinsen <omega@cse.ogi.edu>
 * Library       <2002> Ronald Bultje <rbultje@ronald.bitfreak.net>
 * Copyright (C) <2007> David A. Schleef <ds@schleef.org>
 */
/*
 * This function has been imported directly from the gstreamer project to
 * support backwards compatibility and should be removed when the older version
 * is no longer supported.
 */
gint gst_video_format_info_extrapolate_stride(const GstVideoFormatInfo *finfo, gint plane, gint stride)
{
	gint estride;
	gint comp[GST_VIDEO_MAX_COMPONENTS];
	gint i;

	/* There is nothing to extrapolate on first plane. */
	if (plane == 0)
		return stride;

	gst_video_format_info_component(finfo, plane, comp);

	/*
	 * For now, all planar formats have a single component on first plane, but
	 * if there was a planar format with more, we'd have to make a ratio of the
	 * number of component on the first plane against the number of component on
	 * the current plane.
	 */
	estride = 0;
	for (i = 0; i < GST_VIDEO_MAX_COMPONENTS && comp[i] >= 0; i++)
		estride += GST_VIDEO_FORMAT_INFO_SCALE_WIDTH(finfo, comp[i], stride);

	return estride;
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
