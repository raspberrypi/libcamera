/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer Buffer Pool
 */

#include "gstlibcamerapool.h"

#include <deque>

#include <libcamera/stream.h>

#include "gstlibcamera-utils.h"

using namespace libcamera;

enum {
	SIGNAL_BUFFER_NOTIFY,
	N_SIGNALS
};

static guint signals[N_SIGNALS];

struct _GstLibcameraPool {
	GstBufferPool parent;

	std::deque<GstBuffer *> *queue;
	GstLibcameraAllocator *allocator;
	Stream *stream;
};

G_DEFINE_TYPE(GstLibcameraPool, gst_libcamera_pool, GST_TYPE_BUFFER_POOL)

static GstBuffer *
gst_libcamera_pool_pop_buffer(GstLibcameraPool *self)
{
	GLibLocker lock(GST_OBJECT(self));
	GstBuffer *buf;

	if (self->queue->empty())
		return nullptr;

	buf = self->queue->front();
	self->queue->pop_front();

	return buf;
}

static GstFlowReturn
gst_libcamera_pool_acquire_buffer(GstBufferPool *pool, GstBuffer **buffer,
				  [[maybe_unused]] GstBufferPoolAcquireParams *params)
{
	GstLibcameraPool *self = GST_LIBCAMERA_POOL(pool);
	GstBuffer *buf = gst_libcamera_pool_pop_buffer(self);

	if (!buf)
		return GST_FLOW_ERROR;

	if (!gst_libcamera_allocator_prepare_buffer(self->allocator, self->stream, buf)) {
		GLibLocker lock(GST_OBJECT(self));
		self->queue->push_back(buf);
		return GST_FLOW_ERROR;
	}

	*buffer = buf;
	return GST_FLOW_OK;
}

static void
gst_libcamera_pool_reset_buffer(GstBufferPool *pool, GstBuffer *buffer)
{
	GstBufferPoolClass *klass = GST_BUFFER_POOL_CLASS(gst_libcamera_pool_parent_class);

	/* Clears all the memories and only pool the GstBuffer objects */
	gst_buffer_remove_all_memory(buffer);
	klass->reset_buffer(pool, buffer);
	GST_BUFFER_FLAGS(buffer) = 0;
}

static void
gst_libcamera_pool_release_buffer(GstBufferPool *pool, GstBuffer *buffer)
{
	GstLibcameraPool *self = GST_LIBCAMERA_POOL(pool);
	bool do_notify;

	{
		GLibLocker lock(GST_OBJECT(self));
		do_notify = self->queue->empty();
		self->queue->push_back(buffer);
	}

	if (do_notify)
		g_signal_emit(self, signals[SIGNAL_BUFFER_NOTIFY], 0);
}

static void
gst_libcamera_pool_init(GstLibcameraPool *self)
{
	self->queue = new std::deque<GstBuffer *>();
}

static void
gst_libcamera_pool_finalize(GObject *object)
{
	GstLibcameraPool *self = GST_LIBCAMERA_POOL(object);
	GstBuffer *buf;

	while ((buf = gst_libcamera_pool_pop_buffer(self)))
		gst_buffer_unref(buf);

	delete self->queue;
	g_object_unref(self->allocator);

	G_OBJECT_CLASS(gst_libcamera_pool_parent_class)->finalize(object);
}

static void
gst_libcamera_pool_class_init(GstLibcameraPoolClass *klass)
{
	auto *object_class = G_OBJECT_CLASS(klass);
	auto *pool_class = GST_BUFFER_POOL_CLASS(klass);

	object_class->finalize = gst_libcamera_pool_finalize;
	pool_class->start = nullptr;
	pool_class->acquire_buffer = gst_libcamera_pool_acquire_buffer;
	pool_class->reset_buffer = gst_libcamera_pool_reset_buffer;
	pool_class->release_buffer = gst_libcamera_pool_release_buffer;

	signals[SIGNAL_BUFFER_NOTIFY] = g_signal_new("buffer-notify",
						     G_OBJECT_CLASS_TYPE(klass), G_SIGNAL_RUN_LAST,
						     0, nullptr, nullptr, nullptr,
						     G_TYPE_NONE, 0);
}

static void
gst_libcamera_buffer_add_video_meta(GstBuffer *buffer, GstVideoInfo *info)
{
	GstVideoMeta *vmeta;
	vmeta = gst_buffer_add_video_meta_full(buffer, GST_VIDEO_FRAME_FLAG_NONE,
					       GST_VIDEO_INFO_FORMAT(info), GST_VIDEO_INFO_WIDTH(info),
					       GST_VIDEO_INFO_HEIGHT(info), GST_VIDEO_INFO_N_PLANES(info),
					       info->offset, info->stride);
	GST_META_FLAGS(vmeta) = (GstMetaFlags)(GST_META_FLAGS(vmeta) | GST_META_FLAG_POOLED);
}

GstLibcameraPool *
gst_libcamera_pool_new(GstLibcameraAllocator *allocator, Stream *stream,
		       GstVideoInfo *info)
{
	auto *pool = GST_LIBCAMERA_POOL(g_object_new(GST_TYPE_LIBCAMERA_POOL, nullptr));

	pool->allocator = GST_LIBCAMERA_ALLOCATOR(g_object_ref(allocator));
	pool->stream = stream;

	gsize pool_size = gst_libcamera_allocator_get_pool_size(allocator, stream);
	for (gsize i = 0; i < pool_size; i++) {
		GstBuffer *buffer = gst_buffer_new();
		gst_libcamera_buffer_add_video_meta(buffer, info);
		pool->queue->push_back(buffer);
	}

	return pool;
}

Stream *
gst_libcamera_pool_get_stream(GstLibcameraPool *self)
{
	return self->stream;
}

FrameBuffer *
gst_libcamera_buffer_get_frame_buffer(GstBuffer *buffer)
{
	GstMemory *mem = gst_buffer_peek_memory(buffer, 0);
	return gst_libcamera_memory_get_frame_buffer(mem);
}
