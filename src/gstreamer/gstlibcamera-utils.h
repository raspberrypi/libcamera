/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamera-utils.h - GStreamer libcamera Utility Functions
 */

#pragma once

#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/stream.h>

#include <gst/gst.h>
#include <gst/video/video.h>

GstCaps *gst_libcamera_stream_formats_to_caps(const libcamera::StreamFormats &formats);
GstCaps *gst_libcamera_stream_configuration_to_caps(const libcamera::StreamConfiguration &stream_cfg);
void gst_libcamera_configure_stream_from_caps(libcamera::StreamConfiguration &stream_cfg,
					      GstCaps *caps);
void gst_libcamera_get_framerate_from_caps(GstCaps *caps, GstStructure *element_caps);
void gst_libcamera_clamp_and_set_frameduration(libcamera::ControlList &controls,
					       const libcamera::ControlInfoMap &camera_controls,
					       GstStructure *element_caps);
void gst_libcamera_framerate_to_caps(GstCaps *caps, const GstStructure *element_caps);

#if !GST_CHECK_VERSION(1, 17, 1)
gboolean gst_task_resume(GstTask *task);
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
