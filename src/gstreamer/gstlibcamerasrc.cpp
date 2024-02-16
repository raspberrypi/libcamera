/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamerasrc.cpp - GStreamer Capture Element
 */

/**
 * \todo The following is a list of items that needs implementation in the GStreamer plugin
 *  - Implement GstElement::send_event
 *    + Allowing application to use FLUSH/FLUSH_STOP
 *    + Prevent the main thread from accessing streaming thread
 *  - Implement GstElement::request-new-pad (multi stream)
 *    + Evaluate if a single streaming thread is fine
 *  - Add application driven request (snapshot)
 *  - Add framerate control
 *  - Add buffer importation support
 *
 *  Requires new libcamera API:
 *  - Add framerate negotiation support
 *  - Add colorimetry support
 *  - Add timestamp support
 *  - Use unique names to select the camera devices
 *  - Add GstVideoMeta support (strides and offsets)
 */

#include "gstlibcamerasrc.h"

#include <atomic>
#include <queue>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>

#include <gst/base/base.h>

#include "gstlibcameraallocator.h"
#include "gstlibcamerapad.h"
#include "gstlibcamerapool.h"
#include "gstlibcamera-utils.h"

using namespace libcamera;

GST_DEBUG_CATEGORY_STATIC(source_debug);
#define GST_CAT_DEFAULT source_debug

struct RequestWrap {
	RequestWrap(std::unique_ptr<Request> request);
	~RequestWrap();

	void attachBuffer(Stream *stream, GstBuffer *buffer);
	GstBuffer *detachBuffer(Stream *stream);

	std::unique_ptr<Request> request_;
	std::map<Stream *, GstBuffer *> buffers_;

	GstClockTime latency_;
	GstClockTime pts_;
};

RequestWrap::RequestWrap(std::unique_ptr<Request> request)
	: request_(std::move(request)), latency_(0), pts_(GST_CLOCK_TIME_NONE)
{
}

RequestWrap::~RequestWrap()
{
	for (std::pair<Stream *const, GstBuffer *> &item : buffers_) {
		if (item.second)
			gst_buffer_unref(item.second);
	}
}

void RequestWrap::attachBuffer(Stream *stream, GstBuffer *buffer)
{
	FrameBuffer *fb = gst_libcamera_buffer_get_frame_buffer(buffer);

	request_->addBuffer(stream, fb);

	auto item = buffers_.find(stream);
	if (item != buffers_.end()) {
		gst_buffer_unref(item->second);
		item->second = buffer;
	} else {
		buffers_[stream] = buffer;
	}
}

GstBuffer *RequestWrap::detachBuffer(Stream *stream)
{
	GstBuffer *buffer = nullptr;

	auto item = buffers_.find(stream);
	if (item != buffers_.end()) {
		buffer = item->second;
		item->second = nullptr;
	}

	return buffer;
}

/* Used for C++ object with destructors. */
struct GstLibcameraSrcState {
	GstLibcameraSrc *src_;

	std::shared_ptr<CameraManager> cm_;
	std::shared_ptr<Camera> cam_;
	std::unique_ptr<CameraConfiguration> config_;

	std::vector<GstPad *> srcpads_; /* Protected by stream_lock */

	/*
	 * Contention on this lock_ must be minimized, as it has to be taken in
	 * the realtime-sensitive requestCompleted() handler to protect
	 * queuedRequests_ and completedRequests_.
	 *
	 * stream_lock must be taken before lock_ in contexts where both locks
	 * need to be taken. In particular, this means that the lock_ must not
	 * be held while calling into other graph elements (e.g. when calling
	 * gst_pad_query()).
	 */
	GMutex lock_;
	std::queue<std::unique_ptr<RequestWrap>> queuedRequests_;
	std::queue<std::unique_ptr<RequestWrap>> completedRequests_;

	ControlList initControls_;
	guint group_id_;

	int queueRequest();
	void requestCompleted(Request *request);
	int processRequest();
	void clearRequests();
};

struct _GstLibcameraSrc {
	GstElement parent;

	GRecMutex stream_lock;
	GstTask *task;

	gchar *camera_name;
	controls::AfModeEnum auto_focus_mode = controls::AfModeManual;

	std::atomic<GstEvent *> pending_eos;

	GstLibcameraSrcState *state;
	GstLibcameraAllocator *allocator;
	GstFlowCombiner *flow_combiner;
};

enum {
	PROP_0,
	PROP_CAMERA_NAME,
	PROP_AUTO_FOCUS_MODE,
};

G_DEFINE_TYPE_WITH_CODE(GstLibcameraSrc, gst_libcamera_src, GST_TYPE_ELEMENT,
			GST_DEBUG_CATEGORY_INIT(source_debug, "libcamerasrc", 0,
						"libcamera Source"))

#define TEMPLATE_CAPS GST_STATIC_CAPS("video/x-raw; image/jpeg; video/x-bayer")

/* For the simple case, we have a src pad that is always present. */
GstStaticPadTemplate src_template = {
	"src", GST_PAD_SRC, GST_PAD_ALWAYS, TEMPLATE_CAPS
};

/* More pads can be requested in state < PAUSED */
GstStaticPadTemplate request_src_template = {
	"src_%u", GST_PAD_SRC, GST_PAD_REQUEST, TEMPLATE_CAPS
};

/* Must be called with stream_lock held. */
int GstLibcameraSrcState::queueRequest()
{
	std::unique_ptr<Request> request = cam_->createRequest();
	if (!request)
		return -ENOMEM;

	std::unique_ptr<RequestWrap> wrap =
		std::make_unique<RequestWrap>(std::move(request));

	for (GstPad *srcpad : srcpads_) {
		Stream *stream = gst_libcamera_pad_get_stream(srcpad);
		GstLibcameraPool *pool = gst_libcamera_pad_get_pool(srcpad);
		GstBuffer *buffer;
		GstFlowReturn ret;

		ret = gst_buffer_pool_acquire_buffer(GST_BUFFER_POOL(pool),
						     &buffer, nullptr);
		if (ret != GST_FLOW_OK) {
			/*
			 * RequestWrap has ownership of the request, and we
			 * won't be queueing this one due to lack of buffers.
			 */
			return -ENOBUFS;
		}

		wrap->attachBuffer(stream, buffer);
	}

	GST_TRACE_OBJECT(src_, "Requesting buffers");
	cam_->queueRequest(wrap->request_.get());

	{
		GLibLocker locker(&lock_);
		queuedRequests_.push(std::move(wrap));
	}

	/* The RequestWrap will be deleted in the completion handler. */
	return 0;
}

void
GstLibcameraSrcState::requestCompleted(Request *request)
{
	GST_DEBUG_OBJECT(src_, "buffers are ready");

	std::unique_ptr<RequestWrap> wrap;

	{
		GLibLocker locker(&lock_);
		wrap = std::move(queuedRequests_.front());
		queuedRequests_.pop();
	}

	g_return_if_fail(wrap->request_.get() == request);

	if ((request->status() == Request::RequestCancelled)) {
		GST_DEBUG_OBJECT(src_, "Request was cancelled");
		return;
	}

	if (GST_ELEMENT_CLOCK(src_)) {
		int64_t timestamp = request->metadata().get(controls::SensorTimestamp).value_or(0);

		GstClockTime gst_base_time = GST_ELEMENT(src_)->base_time;
		GstClockTime gst_now = gst_clock_get_time(GST_ELEMENT_CLOCK(src_));
		/* \todo Need to expose which reference clock the timestamp relates to. */
		GstClockTime sys_now = g_get_monotonic_time() * 1000;

		/* Deduced from: sys_now - sys_base_time == gst_now - gst_base_time */
		GstClockTime sys_base_time = sys_now - (gst_now - gst_base_time);
		wrap->pts_ = timestamp - sys_base_time;
		wrap->latency_ = sys_now - timestamp;
	}

	{
		GLibLocker locker(&lock_);
		completedRequests_.push(std::move(wrap));
	}

	gst_task_resume(src_->task);
}

/* Must be called with stream_lock held. */
int GstLibcameraSrcState::processRequest()
{
	std::unique_ptr<RequestWrap> wrap;
	int err = 0;

	{
		GLibLocker locker(&lock_);

		if (!completedRequests_.empty()) {
			wrap = std::move(completedRequests_.front());
			completedRequests_.pop();
		}

		if (completedRequests_.empty())
			err = -ENOBUFS;
	}

	if (!wrap)
		return -ENOBUFS;

	GstFlowReturn ret = GST_FLOW_OK;
	gst_flow_combiner_reset(src_->flow_combiner);

	for (GstPad *srcpad : srcpads_) {
		Stream *stream = gst_libcamera_pad_get_stream(srcpad);
		GstBuffer *buffer = wrap->detachBuffer(stream);

		FrameBuffer *fb = gst_libcamera_buffer_get_frame_buffer(buffer);

		if (GST_CLOCK_TIME_IS_VALID(wrap->pts_)) {
			GST_BUFFER_PTS(buffer) = wrap->pts_;
			gst_libcamera_pad_set_latency(srcpad, wrap->latency_);
		} else {
			GST_BUFFER_PTS(buffer) = 0;
		}

		GST_BUFFER_OFFSET(buffer) = fb->metadata().sequence;
		GST_BUFFER_OFFSET_END(buffer) = fb->metadata().sequence;

		ret = gst_pad_push(srcpad, buffer);
		ret = gst_flow_combiner_update_pad_flow(src_->flow_combiner,
							srcpad, ret);
	}

	switch (ret) {
	case GST_FLOW_OK:
		break;

	case GST_FLOW_NOT_NEGOTIATED: {
		bool reconfigure = false;
		for (GstPad *srcpad : srcpads_) {
			if (gst_pad_needs_reconfigure(srcpad)) {
				reconfigure = true;
				break;
			}
		}

		/* If no pads need a reconfiguration something went wrong. */
		if (!reconfigure)
			err = -EPIPE;

		break;
	}

	case GST_FLOW_EOS: {
		g_autoptr(GstEvent) eos = gst_event_new_eos();
		guint32 seqnum = gst_util_seqnum_next();
		gst_event_set_seqnum(eos, seqnum);
		for (GstPad *srcpad : srcpads_)
			gst_pad_push_event(srcpad, gst_event_ref(eos));

		err = -EPIPE;
		break;
	}

	case GST_FLOW_FLUSHING:
		err = -EPIPE;
		break;

	default:
		GST_ELEMENT_FLOW_ERROR(src_, ret);

		err = -EPIPE;
		break;
	}

	return err;
}

void GstLibcameraSrcState::clearRequests()
{
	GLibLocker locker(&lock_);
	completedRequests_ = {};
}

static bool
gst_libcamera_src_open(GstLibcameraSrc *self)
{
	std::shared_ptr<CameraManager> cm;
	std::shared_ptr<Camera> cam;
	gint ret;

	GST_DEBUG_OBJECT(self, "Opening camera device ...");

	cm = gst_libcamera_get_camera_manager(ret);
	if (ret) {
		GST_ELEMENT_ERROR(self, LIBRARY, INIT,
				  ("Failed listing cameras."),
				  ("libcamera::CameraMananger::start() failed: %s", g_strerror(-ret)));
		return false;
	}

	g_autofree gchar *camera_name = nullptr;
	{
		GLibLocker lock(GST_OBJECT(self));
		if (self->camera_name)
			camera_name = g_strdup(self->camera_name);
	}

	if (camera_name) {
		cam = cm->get(self->camera_name);
		if (!cam) {
			GST_ELEMENT_ERROR(self, RESOURCE, NOT_FOUND,
					  ("Could not find a camera named '%s'.", self->camera_name),
					  ("libcamera::CameraMananger::get() returned nullptr"));
			return false;
		}
	} else {
		if (cm->cameras().empty()) {
			GST_ELEMENT_ERROR(self, RESOURCE, NOT_FOUND,
					  ("Could not find any supported camera on this system."),
					  ("libcamera::CameraMananger::cameras() is empty"));
			return false;
		}
		cam = cm->cameras()[0];
	}

	GST_INFO_OBJECT(self, "Using camera '%s'", cam->id().c_str());

	ret = cam->acquire();
	if (ret) {
		GST_ELEMENT_ERROR(self, RESOURCE, BUSY,
				  ("Camera '%s' is already in use.", cam->id().c_str()),
				  ("libcamera::Camera::acquire() failed: %s", g_strerror(ret)));
		return false;
	}

	cam->requestCompleted.connect(self->state, &GstLibcameraSrcState::requestCompleted);

	/* No need to lock here, we didn't start our threads yet. */
	self->state->cm_ = cm;
	self->state->cam_ = cam;

	return true;
}

/* Must be called with stream_lock held. */
static bool
gst_libcamera_src_negotiate(GstLibcameraSrc *self)
{
	GstLibcameraSrcState *state = self->state;

	g_autoptr(GstStructure) element_caps = gst_structure_new_empty("caps");

	for (gsize i = 0; i < state->srcpads_.size(); i++) {
		GstPad *srcpad = state->srcpads_[i];
		StreamConfiguration &stream_cfg = state->config_->at(i);

		/* Retrieve the supported caps. */
		g_autoptr(GstCaps) filter = gst_libcamera_stream_formats_to_caps(stream_cfg.formats());
		g_autoptr(GstCaps) caps = gst_pad_peer_query_caps(srcpad, filter);
		if (gst_caps_is_empty(caps))
			return false;

		/* Fixate caps and configure the stream. */
		caps = gst_caps_make_writable(caps);
		gst_libcamera_configure_stream_from_caps(stream_cfg, caps);
		gst_libcamera_get_framerate_from_caps(caps, element_caps);
	}

	/* Validate the configuration. */
	if (state->config_->validate() == CameraConfiguration::Invalid)
		return false;

	int ret = state->cam_->configure(state->config_.get());
	if (ret) {
		GST_ELEMENT_ERROR(self, RESOURCE, SETTINGS,
				  ("Failed to configure camera: %s", g_strerror(-ret)),
				  ("Camera::configure() failed with error code %i", ret));
		return false;
	}

	/* Check frame duration bounds within controls::FrameDurationLimits */
	gst_libcamera_clamp_and_set_frameduration(state->initControls_,
						  state->cam_->controls(), element_caps);

	/*
	 * Regardless if it has been modified, create clean caps and push the
	 * caps event. Downstream will decide if the caps are acceptable.
	 */
	for (gsize i = 0; i < state->srcpads_.size(); i++) {
		GstPad *srcpad = state->srcpads_[i];
		const StreamConfiguration &stream_cfg = state->config_->at(i);

		g_autoptr(GstCaps) caps = gst_libcamera_stream_configuration_to_caps(stream_cfg);
		gst_libcamera_framerate_to_caps(caps, element_caps);

		if (!gst_pad_push_event(srcpad, gst_event_new_caps(caps)))
			return false;
	}

	if (self->allocator)
		g_clear_object(&self->allocator);

	self->allocator = gst_libcamera_allocator_new(state->cam_, state->config_.get());
	if (!self->allocator) {
		GST_ELEMENT_ERROR(self, RESOURCE, NO_SPACE_LEFT,
				  ("Failed to allocate memory"),
				  ("gst_libcamera_allocator_new() failed."));
		return false;
	}

	for (gsize i = 0; i < state->srcpads_.size(); i++) {
		GstPad *srcpad = state->srcpads_[i];
		const StreamConfiguration &stream_cfg = state->config_->at(i);

		GstLibcameraPool *pool = gst_libcamera_pool_new(self->allocator,
								stream_cfg.stream());
		g_signal_connect_swapped(pool, "buffer-notify",
					 G_CALLBACK(gst_task_resume), self->task);

		gst_libcamera_pad_set_pool(srcpad, pool);

		/* Clear all reconfigure flags. */
		gst_pad_check_reconfigure(srcpad);
	}

	return true;
}

static void
gst_libcamera_src_task_run(gpointer user_data)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(user_data);
	GstLibcameraSrcState *state = self->state;

	/*
	 * Start by pausing the task. The task may also get resumed by the
	 * buffer-notify signal when new buffers are queued back to the pool,
	 * or by the request completion handler when a new request has
	 * completed.  Both will resume the task after adding the buffers or
	 * request to their respective lists, which are checked below to decide
	 * if the task needs to be resumed for another iteration. This is thus
	 * guaranteed to be race-free, the lock taken by gst_task_pause() and
	 * gst_task_resume() serves as a memory barrier.
	 */
	gst_task_pause(self->task);

	bool doResume = false;

	g_autoptr(GstEvent) event = self->pending_eos.exchange(nullptr);
	if (event) {
		for (GstPad *srcpad : state->srcpads_)
			gst_pad_push_event(srcpad, gst_event_ref(event));

		return;
	}

	/* Check if a srcpad requested a renegotiation. */
	bool reconfigure = false;
	for (GstPad *srcpad : state->srcpads_) {
		if (gst_pad_check_reconfigure(srcpad)) {
			/* Check if the caps even need changing. */
			g_autoptr(GstCaps) caps = gst_pad_get_current_caps(srcpad);
			if (!gst_pad_peer_query_accept_caps(srcpad, caps)) {
				reconfigure = true;
				break;
			}
		}
	}

	if (reconfigure) {
		state->cam_->stop();
		state->clearRequests();

		if (!gst_libcamera_src_negotiate(self)) {
			GST_ELEMENT_FLOW_ERROR(self, GST_FLOW_NOT_NEGOTIATED);
			gst_task_stop(self->task);
		}

		state->cam_->start(&state->initControls_);
	}

	/*
	 * Create and queue one request. If no buffers are available the
	 * function returns -ENOBUFS, which we ignore here as that's not a
	 * fatal error.
	 */
	int ret = state->queueRequest();
	switch (ret) {
	case 0:
		/*
		 * The request was successfully queued, there may be enough
		 * buffers to create a new one. Don't pause the task to give it
		 * another try.
		 */
		doResume = true;
		break;

	case -ENOMEM:
		GST_ELEMENT_ERROR(self, RESOURCE, NO_SPACE_LEFT,
				  ("Failed to allocate request for camera '%s'.",
				   state->cam_->id().c_str()),
				  ("libcamera::Camera::createRequest() failed"));
		gst_task_stop(self->task);
		return;

	case -ENOBUFS:
	default:
		break;
	}

	/*
	 * Process one completed request, if available, and record if further
	 * requests are ready for processing.
	 */
	ret = state->processRequest();
	switch (ret) {
	case 0:
		/* Another completed request is available, resume the task. */
		doResume = true;
		break;

	case -EPIPE:
		gst_task_stop(self->task);
		return;

	case -ENOBUFS:
	default:
		break;
	}

	/* Resume the task for another iteration if needed. */
	if (doResume)
		gst_task_resume(self->task);
}

static void
gst_libcamera_src_task_enter(GstTask *task, [[maybe_unused]] GThread *thread,
			     gpointer user_data)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(user_data);
	GLibRecLocker lock(&self->stream_lock);
	GstLibcameraSrcState *state = self->state;
	gint ret;

	GST_DEBUG_OBJECT(self, "Streaming thread has started");

	gint stream_id_num = 0;
	std::vector<StreamRole> roles;
	for (GstPad *srcpad : state->srcpads_) {
		/* Create stream-id and push stream-start. */
		g_autofree gchar *stream_id_intermediate = g_strdup_printf("%i%i", state->group_id_, stream_id_num++);
		g_autofree gchar *stream_id = gst_pad_create_stream_id(srcpad, GST_ELEMENT(self), stream_id_intermediate);
		GstEvent *event = gst_event_new_stream_start(stream_id);
		gst_event_set_group_id(event, state->group_id_);
		gst_pad_push_event(srcpad, event);

		/* Collect the streams roles for the next iteration. */
		roles.push_back(gst_libcamera_pad_get_role(srcpad));
	}

	/* Generate the stream configurations, there should be one per pad. */
	state->config_ = state->cam_->generateConfiguration(roles);
	if (state->config_ == nullptr) {
		GST_ELEMENT_ERROR(self, RESOURCE, SETTINGS,
				  ("Failed to generate camera configuration from roles"),
				  ("Camera::generateConfiguration() returned nullptr"));
		gst_task_stop(task);
		return;
	}
	g_assert(state->config_->size() == state->srcpads_.size());

	if (!gst_libcamera_src_negotiate(self)) {
		state->initControls_.clear();
		GST_ELEMENT_FLOW_ERROR(self, GST_FLOW_NOT_NEGOTIATED);
		gst_task_stop(task);
		return;
	}

	self->flow_combiner = gst_flow_combiner_new();
	for (GstPad *srcpad : state->srcpads_) {
		gst_flow_combiner_add_pad(self->flow_combiner, srcpad);

		/* Send an open segment event with time format. */
		GstSegment segment;
		gst_segment_init(&segment, GST_FORMAT_TIME);
		gst_pad_push_event(srcpad, gst_event_new_segment(&segment));
	}

	if (self->auto_focus_mode != controls::AfModeManual) {
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::AfMode) != infoMap.end()) {
			state->initControls_.set(controls::AfMode, self->auto_focus_mode);
		} else {
			GST_ELEMENT_ERROR(self, RESOURCE, SETTINGS,
					  ("Failed to enable auto focus"),
					  ("AfMode not supported by this camera, "
					   "please retry with 'auto-focus-mode=AfModeManual'"));
		}
	}

	ret = state->cam_->start(&state->initControls_);
	if (ret) {
		GST_ELEMENT_ERROR(self, RESOURCE, SETTINGS,
				  ("Failed to start the camera: %s", g_strerror(-ret)),
				  ("Camera.start() failed with error code %i", ret));
		gst_task_stop(task);
		return;
	}
}

static void
gst_libcamera_src_task_leave([[maybe_unused]] GstTask *task,
			     [[maybe_unused]] GThread *thread,
			     gpointer user_data)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(user_data);
	GstLibcameraSrcState *state = self->state;

	GST_DEBUG_OBJECT(self, "Streaming thread is about to stop");

	state->cam_->stop();
	state->clearRequests();

	{
		GLibRecLocker locker(&self->stream_lock);
		for (GstPad *srcpad : state->srcpads_)
			gst_libcamera_pad_set_pool(srcpad, nullptr);
	}

	g_clear_object(&self->allocator);
	g_clear_pointer(&self->flow_combiner,
			(GDestroyNotify)gst_flow_combiner_free);
}

static void
gst_libcamera_src_close(GstLibcameraSrc *self)
{
	GstLibcameraSrcState *state = self->state;
	gint ret;

	GST_DEBUG_OBJECT(self, "Releasing resources");

	state->config_.reset();

	ret = state->cam_->release();
	if (ret) {
		GST_ELEMENT_WARNING(self, RESOURCE, BUSY,
				    ("Camera '%s' is still in use.", state->cam_->id().c_str()),
				    ("libcamera::Camera.release() failed: %s", g_strerror(-ret)));
	}

	state->cam_.reset();
	state->cm_.reset();
}

static void
gst_libcamera_src_set_property(GObject *object, guint prop_id,
			       const GValue *value, GParamSpec *pspec)
{
	GLibLocker lock(GST_OBJECT(object));
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(object);

	switch (prop_id) {
	case PROP_CAMERA_NAME:
		g_free(self->camera_name);
		self->camera_name = g_value_dup_string(value);
		break;
	case PROP_AUTO_FOCUS_MODE:
		self->auto_focus_mode = static_cast<controls::AfModeEnum>(g_value_get_enum(value));
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		break;
	}
}

static void
gst_libcamera_src_get_property(GObject *object, guint prop_id, GValue *value,
			       GParamSpec *pspec)
{
	GLibLocker lock(GST_OBJECT(object));
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(object);

	switch (prop_id) {
	case PROP_CAMERA_NAME:
		g_value_set_string(value, self->camera_name);
		break;
	case PROP_AUTO_FOCUS_MODE:
		g_value_set_enum(value, static_cast<gint>(self->auto_focus_mode));
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		break;
	}
}

static GstStateChangeReturn
gst_libcamera_src_change_state(GstElement *element, GstStateChange transition)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(element);
	GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
	GstElementClass *klass = GST_ELEMENT_CLASS(gst_libcamera_src_parent_class);

	ret = klass->change_state(element, transition);
	if (ret == GST_STATE_CHANGE_FAILURE)
		return ret;

	switch (transition) {
	case GST_STATE_CHANGE_NULL_TO_READY:
		if (!gst_libcamera_src_open(self))
			return GST_STATE_CHANGE_FAILURE;
		break;
	case GST_STATE_CHANGE_READY_TO_PAUSED:
		/* This needs to be called after pads activation.*/
		self->state->group_id_ = gst_util_group_id_next();
		if (!gst_task_pause(self->task))
			return GST_STATE_CHANGE_FAILURE;
		ret = GST_STATE_CHANGE_NO_PREROLL;
		break;
	case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
		gst_task_start(self->task);
		break;
	case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
		ret = GST_STATE_CHANGE_NO_PREROLL;
		break;
	case GST_STATE_CHANGE_PAUSED_TO_READY:
		/*
		 * \todo this might require some thread unblocking in the future
		 * if the streaming thread starts doing any kind of blocking
		 * operations. If this was the case, we would need to do so
		 * before pad deactivation, so before chaining to the parent
		 * change_state function.
		 */
		gst_task_join(self->task);
		break;
	case GST_STATE_CHANGE_READY_TO_NULL:
		gst_libcamera_src_close(self);
		break;
	default:
		break;
	}

	return ret;
}

static gboolean
gst_libcamera_src_send_event(GstElement *element, GstEvent *event)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(element);
	gboolean ret = FALSE;

	switch (GST_EVENT_TYPE(event)) {
	case GST_EVENT_EOS: {
		GstEvent *oldEvent = self->pending_eos.exchange(event);
		gst_clear_event(&oldEvent);
		ret = TRUE;
		break;
	}
	default:
		gst_event_unref(event);
		break;
	}

	return ret;
}

static void
gst_libcamera_src_finalize(GObject *object)
{
	GObjectClass *klass = G_OBJECT_CLASS(gst_libcamera_src_parent_class);
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(object);

	g_rec_mutex_clear(&self->stream_lock);
	g_clear_object(&self->task);
	g_mutex_clear(&self->state->lock_);
	g_free(self->camera_name);
	delete self->state;

	return klass->finalize(object);
}

static void
gst_libcamera_src_init(GstLibcameraSrc *self)
{
	GstLibcameraSrcState *state = new GstLibcameraSrcState();
	GstPadTemplate *templ = gst_element_get_pad_template(GST_ELEMENT(self), "src");

	g_rec_mutex_init(&self->stream_lock);
	self->task = gst_task_new(gst_libcamera_src_task_run, self, nullptr);
	gst_task_set_enter_callback(self->task, gst_libcamera_src_task_enter, self, nullptr);
	gst_task_set_leave_callback(self->task, gst_libcamera_src_task_leave, self, nullptr);
	gst_task_set_lock(self->task, &self->stream_lock);

	g_mutex_init(&state->lock_);

	state->srcpads_.push_back(gst_pad_new_from_template(templ, "src"));
	gst_element_add_pad(GST_ELEMENT(self), state->srcpads_.back());

	GST_OBJECT_FLAG_SET(self, GST_ELEMENT_FLAG_SOURCE);

	/* C-style friend. */
	state->src_ = self;
	self->state = state;
}

static GstPad *
gst_libcamera_src_request_new_pad(GstElement *element, GstPadTemplate *templ,
				  const gchar *name, [[maybe_unused]] const GstCaps *caps)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(element);
	g_autoptr(GstPad) pad = NULL;

	GST_DEBUG_OBJECT(self, "new request pad created");

	pad = gst_pad_new_from_template(templ, name);
	g_object_ref_sink(pad);

	if (gst_element_add_pad(element, pad)) {
		GLibRecLocker lock(&self->stream_lock);
		self->state->srcpads_.push_back(reinterpret_cast<GstPad *>(g_object_ref(pad)));
	} else {
		GST_ELEMENT_ERROR(element, STREAM, FAILED,
				  ("Internal data stream error."),
				  ("Could not add pad to element"));
		return NULL;
	}

	return reinterpret_cast<GstPad *>(g_steal_pointer(&pad));
}

static void
gst_libcamera_src_release_pad(GstElement *element, GstPad *pad)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(element);

	GST_DEBUG_OBJECT(self, "Pad %" GST_PTR_FORMAT " being released", pad);

	{
		GLibRecLocker lock(&self->stream_lock);
		std::vector<GstPad *> &pads = self->state->srcpads_;
		auto begin_iterator = pads.begin();
		auto end_iterator = pads.end();
		auto pad_iterator = std::find(begin_iterator, end_iterator, pad);

		if (pad_iterator != end_iterator) {
			g_object_unref(*pad_iterator);
			pads.erase(pad_iterator);
		}
	}
	gst_element_remove_pad(element, pad);
}

static void
gst_libcamera_src_class_init(GstLibcameraSrcClass *klass)
{
	GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
	GObjectClass *object_class = G_OBJECT_CLASS(klass);

	object_class->set_property = gst_libcamera_src_set_property;
	object_class->get_property = gst_libcamera_src_get_property;
	object_class->finalize = gst_libcamera_src_finalize;

	element_class->request_new_pad = gst_libcamera_src_request_new_pad;
	element_class->release_pad = gst_libcamera_src_release_pad;
	element_class->change_state = gst_libcamera_src_change_state;
	element_class->send_event = gst_libcamera_src_send_event;

	gst_element_class_set_metadata(element_class,
				       "libcamera Source", "Source/Video",
				       "Linux Camera source using libcamera",
				       "Nicolas Dufresne <nicolas.dufresne@collabora.com");
	gst_element_class_add_static_pad_template_with_gtype(element_class,
							     &src_template,
							     GST_TYPE_LIBCAMERA_PAD);
	gst_element_class_add_static_pad_template_with_gtype(element_class,
							     &request_src_template,
							     GST_TYPE_LIBCAMERA_PAD);

	GParamSpec *spec = g_param_spec_string("camera-name", "Camera Name",
					       "Select by name which camera to use.", nullptr,
					       (GParamFlags)(GST_PARAM_MUTABLE_READY
							     | G_PARAM_CONSTRUCT
							     | G_PARAM_READWRITE
							     | G_PARAM_STATIC_STRINGS));
	g_object_class_install_property(object_class, PROP_CAMERA_NAME, spec);

	spec = g_param_spec_enum("auto-focus-mode",
				 "Set auto-focus mode",
				 "Available options: AfModeManual, "
				 "AfModeAuto or AfModeContinuous.",
				 gst_libcamera_auto_focus_get_type(),
				 static_cast<gint>(controls::AfModeManual),
				 G_PARAM_WRITABLE);
	g_object_class_install_property(object_class, PROP_AUTO_FOCUS_MODE, spec);
}
