/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcameraallocator.cpp - GStreamer Custom Allocator
 */

#include "gstlibcameraallocator.h"

#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>

#include "gstlibcamera-utils.h"

using namespace libcamera;

static gboolean gst_libcamera_allocator_release(GstMiniObject *mini_object);

/**
 * \struct FrameWrap
 * \brief An internal wrapper to track the relation between FrameBuffer and
 * GstMemory(s)
 *
 * This wrapper maintains a count of the outstanding GstMemory (there may be
 * multiple GstMemory per FrameBuffer), and give back the FrameBuffer to the
 * allocator pool when all memory objects have returned.
 */

struct FrameWrap {
	FrameWrap(GstAllocator *allocator, FrameBuffer *buffer,
		  gpointer stream);
	~FrameWrap();

	void acquirePlane() { ++outstandingPlanes_; }
	bool releasePlane() { return --outstandingPlanes_ == 0; }

	static GQuark getQuark();

	gpointer stream_;
	FrameBuffer *buffer_;
	std::vector<GstMemory *> planes_;
	gint outstandingPlanes_;
};

FrameWrap::FrameWrap(GstAllocator *allocator, FrameBuffer *buffer,
		     gpointer stream)

	: stream_(stream),
	  buffer_(buffer),
	  outstandingPlanes_(0)
{
	for (const FrameBuffer::Plane &plane : buffer->planes()) {
		GstMemory *mem = gst_fd_allocator_alloc(allocator, plane.fd.get(),
							plane.offset + plane.length,
							GST_FD_MEMORY_FLAG_DONT_CLOSE);
		gst_memory_resize(mem, plane.offset, plane.length);
		gst_mini_object_set_qdata(GST_MINI_OBJECT(mem), getQuark(), this, nullptr);
		GST_MINI_OBJECT(mem)->dispose = gst_libcamera_allocator_release;
		g_object_unref(mem->allocator);
		planes_.push_back(mem);
	}
}

FrameWrap::~FrameWrap()
{
	for (GstMemory *mem : planes_) {
		GST_MINI_OBJECT(mem)->dispose = nullptr;
		g_object_ref(mem->allocator);
		gst_memory_unref(mem);
	}
}

GQuark FrameWrap::getQuark()
{
	static gsize frame_quark = 0;

	if (g_once_init_enter(&frame_quark)) {
		GQuark quark = g_quark_from_string("GstLibcameraFrameWrap");
		g_once_init_leave(&frame_quark, quark);
	}

	return frame_quark;
}

/**
 * \struct _GstLibcameraAllocator
 * \brief A pooling GstDmaBufAllocator for libcamera
 *
 * This is a pooling GstDmaBufAllocator implementation. This implementation override
 * the dispose function of memory object in order to keep them alive when they
 * are disposed by downstream elements.
 */
struct _GstLibcameraAllocator {
	GstDmaBufAllocator parent;
	FrameBufferAllocator *fb_allocator;
	/*
	 * A hash table using Stream pointer as key and returning a GQueue of
	 * FrameWrap.
	 */
	GHashTable *pools;
};

G_DEFINE_TYPE(GstLibcameraAllocator, gst_libcamera_allocator,
	      GST_TYPE_DMABUF_ALLOCATOR)

static gboolean
gst_libcamera_allocator_release(GstMiniObject *mini_object)
{
	GstMemory *mem = GST_MEMORY_CAST(mini_object);
	GstLibcameraAllocator *self = GST_LIBCAMERA_ALLOCATOR(mem->allocator);

	{
		GLibLocker lock(GST_OBJECT(self));
		auto *frame = reinterpret_cast<FrameWrap *>(gst_mini_object_get_qdata(mini_object, FrameWrap::getQuark()));

		gst_memory_ref(mem);

		if (frame->releasePlane()) {
			auto *pool = reinterpret_cast<GQueue *>(g_hash_table_lookup(self->pools, frame->stream_));
			g_return_val_if_fail(pool, TRUE);
			g_queue_push_tail(pool, frame);
		}
	}

	/* Keep last in case we are holding on the last allocator ref. */
	g_object_unref(mem->allocator);

	/* Return FALSE so that our mini object isn't freed. */
	return FALSE;
}

static void
gst_libcamera_allocator_free_pool(gpointer data)
{
	GQueue *queue = reinterpret_cast<GQueue *>(data);
	FrameWrap *frame;

	while ((frame = reinterpret_cast<FrameWrap *>(g_queue_pop_head(queue)))) {
		g_warn_if_fail(frame->outstandingPlanes_ == 0);
		delete frame;
	}

	g_queue_free(queue);
}

static void
gst_libcamera_allocator_init(GstLibcameraAllocator *self)
{
	self->pools = g_hash_table_new_full(nullptr, nullptr, nullptr,
					    gst_libcamera_allocator_free_pool);
	GST_OBJECT_FLAG_SET(self, GST_ALLOCATOR_FLAG_CUSTOM_ALLOC);
}

static void
gst_libcamera_allocator_dispose(GObject *object)
{
	GstLibcameraAllocator *self = GST_LIBCAMERA_ALLOCATOR(object);

	if (self->pools) {
		g_hash_table_unref(self->pools);
		self->pools = nullptr;
	}

	G_OBJECT_CLASS(gst_libcamera_allocator_parent_class)->dispose(object);
}

static void
gst_libcamera_allocator_finalize(GObject *object)
{
	GstLibcameraAllocator *self = GST_LIBCAMERA_ALLOCATOR(object);

	delete self->fb_allocator;

	G_OBJECT_CLASS(gst_libcamera_allocator_parent_class)->finalize(object);
}

static void
gst_libcamera_allocator_class_init(GstLibcameraAllocatorClass *klass)
{
	auto *allocator_class = GST_ALLOCATOR_CLASS(klass);
	auto *object_class = G_OBJECT_CLASS(klass);

	object_class->dispose = gst_libcamera_allocator_dispose;
	object_class->finalize = gst_libcamera_allocator_finalize;
	allocator_class->alloc = nullptr;
}

GstLibcameraAllocator *
gst_libcamera_allocator_new(std::shared_ptr<Camera> camera,
			    CameraConfiguration *config_)
{
	auto *self = GST_LIBCAMERA_ALLOCATOR(g_object_new(GST_TYPE_LIBCAMERA_ALLOCATOR,
							  nullptr));

	self->fb_allocator = new FrameBufferAllocator(camera);
	for (StreamConfiguration &streamCfg : *config_) {
		Stream *stream = streamCfg.stream();
		gint ret;

		ret = self->fb_allocator->allocate(stream);
		if (ret == 0)
			return nullptr;

		GQueue *pool = g_queue_new();
		for (const std::unique_ptr<FrameBuffer> &buffer :
		     self->fb_allocator->buffers(stream)) {
			auto *fb = new FrameWrap(GST_ALLOCATOR(self),
						 buffer.get(), stream);
			g_queue_push_tail(pool, fb);
		}

		g_hash_table_insert(self->pools, stream, pool);
	}

	return self;
}

bool
gst_libcamera_allocator_prepare_buffer(GstLibcameraAllocator *self,
				       Stream *stream, GstBuffer *buffer)
{
	GLibLocker lock(GST_OBJECT(self));

	auto *pool = reinterpret_cast<GQueue *>(g_hash_table_lookup(self->pools, stream));
	g_return_val_if_fail(pool, false);

	auto *frame = reinterpret_cast<FrameWrap *>(g_queue_pop_head(pool));
	if (!frame)
		return false;

	for (GstMemory *mem : frame->planes_) {
		frame->acquirePlane();
		gst_buffer_append_memory(buffer, mem);
		g_object_ref(mem->allocator);
	}

	return true;
}

gsize
gst_libcamera_allocator_get_pool_size(GstLibcameraAllocator *self,
				      Stream *stream)
{
	GLibLocker lock(GST_OBJECT(self));

	auto *pool = reinterpret_cast<GQueue *>(g_hash_table_lookup(self->pools, stream));
	g_return_val_if_fail(pool, false);

	return pool->length;
}

FrameBuffer *
gst_libcamera_memory_get_frame_buffer(GstMemory *mem)
{
	auto *frame = reinterpret_cast<FrameWrap *>(gst_mini_object_get_qdata(GST_MINI_OBJECT_CAST(mem),
									      FrameWrap::getQuark()));
	return frame->buffer_;
}
