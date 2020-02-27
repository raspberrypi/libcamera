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
GstCaps *gst_libcamera_stream_configuration_to_caps(const libcamera::StreamConfiguration &stream_cfg);
void gst_libcamera_configure_stream_from_caps(libcamera::StreamConfiguration &stream_cfg,
					      GstCaps *caps);
void gst_libcamera_resume_task(GstTask *task);

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

#endif /* __GST_LIBCAMERA_UTILS_H__ */
