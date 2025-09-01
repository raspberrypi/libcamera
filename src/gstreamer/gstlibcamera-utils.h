/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer libcamera Utility Functions
 */

#pragma once

#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/stream.h>

#include <gst/gst.h>
#include <gst/video/video.h>

GstCaps *gst_libcamera_stream_formats_to_caps(const libcamera::StreamFormats &formats);
GstCaps *gst_libcamera_stream_configuration_to_caps(const libcamera::StreamConfiguration &stream_cfg,
						    GstVideoTransferFunction transfer);
void gst_libcamera_configure_stream_from_caps(libcamera::StreamConfiguration &stream_cfg,
					      GstCaps *caps, GstVideoTransferFunction *transfer);
void gst_libcamera_get_framerate_from_caps(GstCaps *caps, GstStructure *element_caps);
void gst_libcamera_clamp_and_set_frameduration(libcamera::ControlList &controls,
					       const libcamera::ControlInfoMap &camera_controls,
					       GstStructure *element_caps);
void gst_libcamera_framerate_to_caps(GstCaps *caps, const GstStructure *element_caps);
void gst_libcamera_gvalue_set_point(GValue *value, const libcamera::Point &point);
void gst_libcamera_gvalue_set_size(GValue *value, const libcamera::Size &size);
void gst_libcamera_gvalue_set_rectangle(GValue *value, const libcamera::Rectangle &rect);
libcamera::Rectangle gst_libcamera_gvalue_get_rectangle(const GValue *value);
int gst_libcamera_set_structure_field(GstStructure *structure,
				      const libcamera::ControlId *id,
				      const libcamera::ControlValue &value);

#if !GST_CHECK_VERSION(1, 16, 0)
static inline void gst_clear_event(GstEvent **event_ptr)
{
	g_clear_pointer(event_ptr, gst_mini_object_unref);
}
#endif

#if !GST_CHECK_VERSION(1, 17, 1)
gboolean gst_task_resume(GstTask *task);
#endif

#if !GST_CHECK_VERSION(1, 22, 0)
gint gst_video_format_info_extrapolate_stride(const GstVideoFormatInfo *finfo, gint plane, gint stride);
#endif

std::shared_ptr<libcamera::CameraManager> gst_libcamera_get_camera_manager(int &ret);

/**
 * \class GLibLocker
 * \brief A simple scoped mutex locker for GMutex
 */
class GLibLocker
{
public:
	GLibLocker(GMutex *mutex)
		: mutex_(mutex)
	{
		g_mutex_lock(mutex_);
	}

	GLibLocker(GstObject *object)
		: mutex_(GST_OBJECT_GET_LOCK(object))
	{
		g_mutex_lock(mutex_);
	}

	~GLibLocker()
	{
		g_mutex_unlock(mutex_);
	}

private:
	GMutex *mutex_;
};

/**
 * \class GLibRecLocker
 * \brief A simple scoped mutex locker for GRecMutex
 */
class GLibRecLocker
{
public:
	GLibRecLocker(GRecMutex *mutex)
		: mutex_(mutex)
	{
		g_rec_mutex_lock(mutex_);
	}

	~GLibRecLocker()
	{
		g_rec_mutex_unlock(mutex_);
	}

private:
	GRecMutex *mutex_;
};
