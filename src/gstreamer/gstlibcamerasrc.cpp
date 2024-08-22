/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer Capture Element
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
#include <vector>

#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <gst/base/base.h>
#include <queue>
#define GST_USE_UNSTABLE_API
#include <gst/interfaces/photography.h>

#include "gstlibcamera-utils.h"
#include "gstlibcameraallocator.h"
#include "gstlibcamerapad.h"
#include "gstlibcamerapool.h"

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
	std::shared_ptr<CameraConfiguration> config_;

	std::shared_ptr<std::vector<GstLibcameraSrcSensorModes>> sensor_modes_;
	std::optional<libcamera::SensorConfiguration> override_sensor_configuration_;

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

	/* Controls state */
	controls::AfModeEnum auto_focus_mode = controls::AfModeManual;

	controls::AeExposureModeEnum exposure_mode = controls::ExposureNormal;
	int32_t exposure_time;
	float exposure_value;
	controls::AwbModeEnum awb_mode = controls::AwbAuto;
	float analog_gain;
	float saturation = 1.0;

	/* GstPhotography interface implementation */
	GstPhotographyCaps capabilities;
	ControlList pendingControls_;
};

struct _GstLibcameraSrc {
	GstElement parent;

	GRecMutex stream_lock;
	GstTask *task;

	gchar *camera_name;

	std::atomic<GstEvent *> pending_eos;

	GstLibcameraSrcState *state;
	GstLibcameraAllocator *allocator;
	GstFlowCombiner *flow_combiner;
};

enum {
	PROP_0,
	PROP_CAMERA_NAME,
	PROP_AUTO_FOCUS_MODE,
	PROP_WB_MODE,
	PROP_COLOUR_TONE,
	PROP_SCENE_MODE,
	PROP_FLASH_MODE,
	PROP_NOISE_REDUCTION,
	PROP_CAPABILITIES,
	PROP_EV_COMP,
	PROP_ISO_SPEED,
	PROP_APERTURE,
	PROP_EXPOSURE_MODE,
	PROP_IMAGE_CAPTURE_SUPPORTED_CAPS,
	PROP_IMAGE_PREVIEW_SUPPORTED_CAPS,
	PROP_FLICKER_MODE,
	PROP_FOCUS_MODE,
	PROP_ZOOM,
	PROP_SMOOTH_ZOOM,
	PROP_WHITE_POINT,
	PROP_MIN_EXPOSURE_TIME,
	PROP_MAX_EXPOSURE_TIME,
	PROP_LENS_FOCUS,
	PROP_EXPOSURE_TIME,
	PROP_COLOR_TEMPERATURE,
	PROP_ANALOG_GAIN,
	PROP_OVERRIDE_SENSOR_MODE,
	PROP_SATURATION,
};

static void gst_libcamera_src_child_proxy_init(gpointer g_iface,
					       gpointer iface_data);
static const struct
{
	const GstPhotographyCaps pcap;
	const ControlId *id;
} controls_map[] = {
	{ GST_PHOTOGRAPHY_CAPS_FOCUS, &controls::AfMode },
	{ GST_PHOTOGRAPHY_CAPS_EXPOSURE, &controls::ExposureTime },
	{ GST_PHOTOGRAPHY_CAPS_EXPOSURE, &controls::ExposureValue },
	{ GST_PHOTOGRAPHY_CAPS_EXPOSURE, &controls::AnalogueGain },
	{ GST_PHOTOGRAPHY_CAPS_SATURATION, &controls::Saturation },
	{ GST_PHOTOGRAPHY_CAPS_WB_MODE, &controls::AwbEnable },
};

static void gst_libcamera_src_photography_init(gpointer g_iface);

G_DEFINE_TYPE_WITH_CODE(GstLibcameraSrc, gst_libcamera_src, GST_TYPE_ELEMENT,
			G_IMPLEMENT_INTERFACE(GST_TYPE_CHILD_PROXY,
					      gst_libcamera_src_child_proxy_init)
			GST_DEBUG_CATEGORY_INIT(source_debug, "libcamerasrc", 0,
						"libcamera Source");
			G_IMPLEMENT_INTERFACE(GST_TYPE_PHOTOGRAPHY,
					      gst_libcamera_src_photography_init))
#define TEMPLATE_CAPS GST_STATIC_CAPS("video/x-raw; image/jpeg; video/x-bayer")
/* For the simple case, we have a src pad that is always present. */
GstStaticPadTemplate src_template = {
	"src",
	GST_PAD_SRC,
	GST_PAD_ALWAYS,
	TEMPLATE_CAPS
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

	{
		GLibLocker locker(&lock_);
		if (pendingControls_.size()) {
			GST_LOG_OBJECT(src_, "Setting %" G_GSIZE_FORMAT " pending controls", pendingControls_.size());
			wrap->request_->controls() = pendingControls_;
			pendingControls_.clear();
		}
	}

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

void GstLibcameraSrcState::requestCompleted(Request *request)
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

// We're going to make a list of all the available sensor modes.
std::vector<GstLibcameraSrcSensorModes> gst_libcamera_src_enumerate_sensor_modes(GstLibcameraSrc *self)
{
	GST_DEBUG_OBJECT(self, "Enumerating sensor modes");

	std::vector<GstLibcameraSrcSensorModes> modes;
	std::unique_ptr<CameraConfiguration> config = self->state->cam_->generateConfiguration({ libcamera::StreamRole::Raw });

	if (config == nullptr) {
		GST_ERROR_OBJECT(self, "Failed to generate config for camera, could not enumerate stream modes");
		return modes;
	}
	libcamera::StreamConfiguration &stream = config->at(0);

	const libcamera::StreamFormats &formats = stream.formats();

	for (const auto &pix : formats.pixelformats()) {
		for (const auto &size : formats.sizes(pix)) {
			GST_DEBUG_OBJECT(self, "Inspecting sensor mode %dx%d@%s",
					 size.width, size.height, pix.toString().c_str());

			config->at(0).size = size;
			config->at(0).pixelFormat = pix;
			// if (is_pixel_format_mjpeg(pix)) {
			// 	GST_WARNING_OBJECT(self, "Skipping MJPEG sensor mode %dx%d@%s",
			// 			   size.width, size.height, pix.toString().c_str());
			// 	continue;
			// }
			config->validate();

			if (config->validate() == CameraConfiguration::Invalid) {
				GST_WARNING_OBJECT(self, "Skipping unsupported sensor mode %dx%d@%s",
						   size.width, size.height, pix.toString().c_str());
				continue;
			}

			GST_DEBUG_OBJECT(self, "Configuring camera with a validated config to get FrameDurationLimits");
			self->state->cam_->configure(config.get());

			auto fd_ctrl = self->state->cam_->controls().find(&controls::FrameDurationLimits);
			gdouble min_framerate = 0.0;
			gdouble max_framerate = 0.0;
			// check if the optional FrameDurationLimits control is available
			if (fd_ctrl != self->state->cam_->controls().end()) {
				min_framerate = gst_util_guint64_to_gdouble(1.0e6) /
						gst_util_guint64_to_gdouble(fd_ctrl->second.max().get<int64_t>());
				max_framerate = gst_util_guint64_to_gdouble(1.0e6) /
						gst_util_guint64_to_gdouble(fd_ctrl->second.min().get<int64_t>());
			} else {
				GST_DEBUG_OBJECT(self, "mode has no FrameDurationLimits,leaving min and max at 0.0");
			}

			// Storing our sensorMode in a struct to be added to the list of available sensor modes.
			GstLibcameraSrcSensorModes current_mode;
			current_mode.size = size;
			current_mode.pixel_format = pix;
			current_mode.bit_depth = pixel_format_to_depth(pix);
			current_mode.min_framerate = min_framerate;
			current_mode.max_framerate = max_framerate;
			modes.push_back(current_mode);

			GST_DEBUG_OBJECT(self, "Found Sensor mode of max_width: %d, max_height: %d, format: %s, min_framerate: %.1f, max_framerate: %.1f",
					 current_mode.size.height,
					 current_mode.size.width,
					 current_mode.pixel_format.toString().c_str(),
					 current_mode.min_framerate,
					 current_mode.max_framerate);

			GST_DEBUG_OBJECT(self, "Select this mode using the 'mmal -mode style' string: 'libcamera override-sensormode=%d:%d:%d ",
					 current_mode.size.width,
					 current_mode.size.height,
					 current_mode.bit_depth);
		}
	}

	return modes;
}

/**
 * @brief
 *   Configure a sensor mode based on aspect ratio, size and framerate.
 *   Similar scoring mechanism that the libcamera pipeline handler does internally, but considers the framerate as well.
 * @param element_caps A GstStructure that contains the framerate of the last pad considered in the loop above. (fixme)
 * @param sensor_modes A vector of GstLibcameraSrcSensorModes, which contains the available sensor modes including their size, pixelformat and framerate.
 * @param cam_cfg A CameraConfiguration, which contains a list of StreamConfigurations. We assume these have been configured based on caps using `gst_libcamera_configure_stream_from_caps`
 */
void gst_libcamera_find_best_format_with_framerate(GstLibcameraSrc *self, std::shared_ptr<std::vector<GstLibcameraSrcSensorModes>> available_sensor_modes,
						   GstStructure *element_caps,
						   std::shared_ptr<libcamera::CameraConfiguration> cam_cfg_from_pads)
{
	// Scoring mechanism for 2D values.
	auto scoreFormat = [](double desired, double actual) -> double {
		double score = desired - actual;
		// Smaller desired dimensions are preferred.
		if (score < 0.0)
			score = (-score) / 8;
		// Penalise non-exact matches.
		if (actual != desired)
			score *= 2;

		return score;
	};

	// sort the streamconfigurations inside cam_cfg_from_pads by their size.
	std::sort(cam_cfg_from_pads->begin(), cam_cfg_from_pads->end(),
		  [](auto &l, auto &r) { return l.size > r.size; });

	// print all streamconfigurations inside cam_cfg_from_pads
	for (auto &stream_cfg : *cam_cfg_from_pads) {
		GST_INFO_OBJECT(self, "StreamConfigurations in camera config: size: %dx%d, pixelformat: %s. Will only consider the first/largest one.", stream_cfg.size.width, stream_cfg.size.height, stream_cfg.pixelFormat.toString().c_str());
	}

	// We only consider the largest video size going forward.
	auto requested_stream_cfg = cam_cfg_from_pads->at(0);

	// Determine the aspect ratio of the requested caps.
	//
	double req_aspect_ratio = static_cast<double>(requested_stream_cfg.size.width) / requested_stream_cfg.size.height;

	// Determine the framerate of the requested caps.
	// todo: handle framerate for every streamconfiguration. Right now the loop creating the streamconfigurations will always override the framerate in the element_caps

	// The sensor configuration that is happen in the pipeline works fine as long as we don't care about framerate.
	// Therefore, if no framerate is specified in the requested caps, we don't touch the StreamConfiguration.
	gint req_fps_n = 30, req_fps_d = 1;
	gdouble req_fps = 30.0;
	if (gst_structure_has_field_typed(element_caps, "framerate", GST_TYPE_FRACTION)) {
		if (!gst_structure_get_fraction(element_caps, "framerate", &req_fps_n, &req_fps_d)) {
			GST_WARNING_OBJECT(self, "Invalid framerate in the requested caps, not touching StreamConfiguration");
			return;
		} else {
			gst_util_fraction_to_double(req_fps_n, req_fps_d, &req_fps);
		}
	} else {
		GST_WARNING_OBJECT(self, "No framerate in the requested caps, not touching StreamConfiguration");
		return;
	}

	// Determine the bit depth of the requested stream.
	unsigned int req_bit_depth = pixel_format_to_depth(requested_stream_cfg.pixelFormat);

	constexpr float penalty_AR = 1500.0;
	constexpr float penalty_BD = 500.0;
	constexpr float penalty_FPS = 2000.0;

	std::optional<GstLibcameraSrcSensorModes> best_mode;
	double best_score = std::numeric_limits<double>::max();
	double score = std::numeric_limits<double>::max();

	// Iterate over the sensor_modes
	for (auto &sensor_mode : *available_sensor_modes) {
		// Compute a penalty based on the difference in width and height between the requested one and the one supported by the sensor mode.

		score = scoreFormat(requested_stream_cfg.size.width, sensor_mode.size.width);
		GST_DEBUG_OBJECT(self, "Sensor mode has width of %d, requested width is %d. Score is %.1f", sensor_mode.size.width, requested_stream_cfg.size.width, score);
		score = scoreFormat(requested_stream_cfg.size.height, sensor_mode.size.height);
		GST_DEBUG_OBJECT(self, "Sensor mode has height of %d, requested height is %d. Score is now %.1f", sensor_mode.size.height, requested_stream_cfg.size.height, score);

		// Compute a penalty based on the difference in aspect ration between the requested one and the one supported by the sensor mode.
		double sm_aspect_ratio = static_cast<double>(sensor_mode.size.width) / sensor_mode.size.height;
		score += penalty_AR * scoreFormat(req_aspect_ratio, sm_aspect_ratio);
		GST_DEBUG_OBJECT(self, "Sensor mode has aspect ratio of %.1f, requested aspect ratio is %.1f. Score is now %.1f", sm_aspect_ratio, req_aspect_ratio, score);

		// Compute a penalty based on the framerate difference between the requested one and the one supported by the sensor mode.
		score += penalty_FPS * std::abs(req_fps - std::min(sensor_mode.max_framerate, req_fps));
		GST_DEBUG_OBJECT(self, "Sensor mode has max framerate of %.1f, requested framerate is %.1f. Score is now %.1f", sensor_mode.max_framerate, req_fps, score);

		// Compute a penalty based on the bit depth difference between the requested one and the one supported by the sensor mode.
		// If no bit depth is requested, we don't apply a penalty.
		if (req_bit_depth != 0) {
			score += penalty_BD * abs((int)(req_bit_depth - sensor_mode.bit_depth));
		}
		GST_DEBUG_OBJECT(self, "Sensor mode has bit_depth of %d (%s), requested bit_depth is %d (%s). Score is now %.1f",
				 sensor_mode.bit_depth,
				 sensor_mode.pixel_format.toString().c_str(),
				 req_bit_depth,
				 requested_stream_cfg.pixelFormat.toString().c_str(),
				 score);
		GST_DEBUG_OBJECT(self, "Scoring of this sensor mode has concluded. Score is now %.1f", score);

		// Smaller score is better.
		if (score <= best_score) {
			best_score = score;
			best_mode = GstLibcameraSrcSensorModes{
				.size = sensor_mode.size,
				.pixel_format = sensor_mode.pixel_format,
				.bit_depth = sensor_mode.bit_depth,
				.min_framerate = sensor_mode.min_framerate,
				.max_framerate = sensor_mode.max_framerate
			};

			GST_DEBUG_OBJECT(self, "New best mode found: width: %d, height: %d, bit_depth: %d, pixel_format: %s. Score: %.1f",
					 sensor_mode.size.width,
					 sensor_mode.size.height,
					 sensor_mode.bit_depth,
					 sensor_mode.pixel_format.toString().c_str(),
					 score);
		}
	}

	// Apply the best mode found.
	if (best_mode) {
		GST_INFO_OBJECT(self, "Applying best mode: width: %d, height: %d, bit_depth: %d, pixel_format: %s.",
				best_mode->size.width,
				best_mode->size.height,
				best_mode->bit_depth,
				best_mode->pixel_format.toString().c_str());
		libcamera::SensorConfiguration sensorconfig = libcamera::SensorConfiguration();
		sensorconfig.outputSize = best_mode->size;
		sensorconfig.bitDepth = best_mode->bit_depth;
		cam_cfg_from_pads->sensorConfig = sensorconfig;
	}
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
		cam = cm->get(camera_name);
		if (!cam) {
			GST_ELEMENT_ERROR(self, RESOURCE, NOT_FOUND,
					  ("Could not find a camera named '%s'.", camera_name),
					  ("libcamera::CameraMananger::get() returned nullptr"));
			return false;
		}
	} else {
		auto cameras = cm->cameras();
		if (cameras.empty()) {
			GST_ELEMENT_ERROR(self, RESOURCE, NOT_FOUND,
					  ("Could not find any supported camera on this system."),
					  ("libcamera::CameraMananger::cameras() is empty"));
			return false;
		}
		cam = cameras[0];
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

	/* Lock before exposing the camera into the state */
	GLibLocker locker(&self->state->lock_);

	self->state->cm_ = cm;
	self->state->cam_ = cam;

	self->state->sensor_modes_ = std::make_shared<std::vector<GstLibcameraSrcSensorModes>>(
		gst_libcamera_src_enumerate_sensor_modes(self));
	/* Update GstPhotography caps */
	{
		int pcaps = GST_PHOTOGRAPHY_CAPS_NONE;
		const ControlInfoMap &infoMap = cam->controls();

		for (gsize i = 0; i < G_N_ELEMENTS(controls_map); i++) {
			if (infoMap.find(controls_map[i].id) != infoMap.end()) {
				pcaps = pcaps | controls_map[i].pcap;
			}
		}
		self->state->capabilities = (GstPhotographyCaps)pcaps;
	}

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
		// fixme: later caps from pads will override the framerate from earlier caps
		gst_libcamera_get_framerate_from_caps(caps, element_caps);
	}

	// Override the sensorconfiguration
	if (state->override_sensor_configuration_) {
		GST_DEBUG_OBJECT(self, "Overriding sensor mode/configuration based on set property");
		state->config_->sensorConfig = state->override_sensor_configuration_;
	} else
	//todo: consider turning this into an else if, handling the case of the override_sensormode property string to be set to framerate
	{
		// Libcamera selects the actual sensor mode based on the stream size (width, height) and the bitdepth. It ignores the framerate.
		// We run a modificated version of the sensor mode selection which does consider framerate.
		// We then set the optional sensorConfig in the CameraConfiguration based on the selected sensor mode
		gst_libcamera_find_best_format_with_framerate(self, state->sensor_modes_, element_caps, state->config_);
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
		// fixme: This will set the same framerate for all streams, based on the last pad considered in the loop above.
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

	/* Generate the default stream configurations, there should be one per pad. */
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

	if (state->pendingControls_.size()) {
		/* Merge any pending controls into the initial control set */
		state->initControls_.merge(state->pendingControls_);
	}

	if (state->auto_focus_mode != controls::AfModeManual) {
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::AfMode) != infoMap.end()) {
			state->initControls_.set(controls::AfMode, state->auto_focus_mode);
		} else {
			GST_ELEMENT_ERROR(self, RESOURCE, SETTINGS,
					  ("Failed to enable auto focus"),
					  ("AfMode not supported by this camera, "
					   "please retry with 'auto-focus-mode=AfModeManual'"));
		}
	}

	GST_LOG_OBJECT(state->src_,
		       "Setting %" G_GSIZE_FORMAT " initial controls", state->initControls_.size());
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
		self->state->auto_focus_mode = static_cast<controls::AfModeEnum>(g_value_get_enum(value));
		break;
	case PROP_CAPABILITIES:
		break;
	case PROP_WB_MODE:
		gst_photography_set_white_balance_mode(GST_PHOTOGRAPHY(self),
						       (GstPhotographyWhiteBalanceMode)g_value_get_enum(value));
		break;
	case PROP_EXPOSURE_MODE: {
		GstPhotographyInterface *iface = GST_PHOTOGRAPHY_GET_INTERFACE(self);
		if (iface->set_exposure_mode != NULL) {
			iface->set_exposure_mode(GST_PHOTOGRAPHY(self),
						 (GstPhotographyExposureMode)g_value_get_enum(value));
		}
		break;
	}
	case PROP_EXPOSURE_TIME:
		gst_photography_set_exposure(GST_PHOTOGRAPHY(self),
					     g_value_get_uint(value));
		break;
	case PROP_EV_COMP:
		gst_photography_set_ev_compensation(GST_PHOTOGRAPHY(self),
						    g_value_get_float(value));
		break;
	case PROP_ANALOG_GAIN: {
		GstPhotographyInterface *iface = GST_PHOTOGRAPHY_GET_INTERFACE(self);
		if (iface->set_analog_gain != NULL) {
			iface->set_analog_gain(GST_PHOTOGRAPHY(self),
					       g_value_get_float(value));
		}
		break;
	}
	case PROP_COLOUR_TONE:
	case PROP_SCENE_MODE:
	case PROP_FLASH_MODE:
	case PROP_NOISE_REDUCTION:
	case PROP_ISO_SPEED:
	case PROP_APERTURE:
	case PROP_IMAGE_CAPTURE_SUPPORTED_CAPS:
	case PROP_IMAGE_PREVIEW_SUPPORTED_CAPS:
	case PROP_FLICKER_MODE:
	case PROP_FOCUS_MODE:
	case PROP_ZOOM:
	case PROP_SMOOTH_ZOOM:
	case PROP_WHITE_POINT:
	case PROP_MIN_EXPOSURE_TIME:
	case PROP_MAX_EXPOSURE_TIME:
	case PROP_LENS_FOCUS:
	case PROP_COLOR_TEMPERATURE:
	case PROP_OVERRIDE_SENSOR_MODE: {
		const gchar *prop_string = g_value_get_string(value);
		if (prop_string) {
			auto prop_array = g_strsplit(prop_string, ":", 4);

			if (g_strv_length(prop_array) >= 3) {
				guint width, height, bit_depth;

				width = static_cast<guint>(g_ascii_strtoull(prop_array[0], nullptr, 10));
				height = static_cast<guint>(g_ascii_strtoull(prop_array[1], nullptr, 10));
				bit_depth = static_cast<guint>(g_ascii_strtoull(prop_array[2], nullptr, 10));

				libcamera::SensorConfiguration sensorconfig = libcamera::SensorConfiguration();
				sensorconfig.outputSize = { width, height };
				sensorconfig.bitDepth = bit_depth;
				self->state->override_sensor_configuration_ = sensorconfig;

				GST_DEBUG_OBJECT(self, "Setting %s to %d:%d:%d", pspec->name, width, height, bit_depth);
			} else {
				GST_ERROR_OBJECT(self, "Invalid format for property %s, should begin with: 'width:height:bit_depth', got %s", pspec->name, prop_string);
			}
			g_strfreev(prop_array);
		}
		break;
	}
	case PROP_SATURATION: {
		GstPhotographyInterface *iface = GST_PHOTOGRAPHY_GET_INTERFACE(self);
		if (iface->set_saturation != NULL) {
			iface->set_saturation(GST_PHOTOGRAPHY(self),
					      g_value_get_float(value));
		}
		break;
	}
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
		g_value_set_enum(value,
				 static_cast<gint>(self->state->auto_focus_mode));
		break;
	case PROP_CAPABILITIES:
		g_value_set_ulong(value,
				  (gulong)gst_photography_get_capabilities(GST_PHOTOGRAPHY(self)));
		break;
	case PROP_WB_MODE: {
		GstPhotographyWhiteBalanceMode wb;

		if (gst_photography_get_white_balance_mode(GST_PHOTOGRAPHY(self), &wb)) {
			g_value_set_enum(value, wb);
		}
		break;
	}
	case PROP_EXPOSURE_MODE: {
		GstPhotographyExposureMode exposure_mode;
		GstPhotographyInterface *iface = GST_PHOTOGRAPHY_GET_INTERFACE(self);
		if (iface->get_exposure_mode != NULL && iface->get_exposure_mode(GST_PHOTOGRAPHY(self),
										 &exposure_mode)) {
			g_value_set_enum(value, exposure_mode);
		}
		break;
	}
	case PROP_EXPOSURE_TIME: {
		guint32 exposure_time;
		if (gst_photography_get_exposure(GST_PHOTOGRAPHY(self), &exposure_time)) {
			g_value_set_uint(value, exposure_time);
		}
		break;
	}
	case PROP_EV_COMP: {
		gfloat exposure_value;
		if (gst_photography_get_ev_compensation(GST_PHOTOGRAPHY(self),
							&exposure_value)) {
			g_value_set_float(value, exposure_value);
		}
		break;
	}
	case PROP_ANALOG_GAIN: {
		gfloat gain;

		GstPhotographyInterface *iface = GST_PHOTOGRAPHY_GET_INTERFACE(self);
		if (iface->get_analog_gain != NULL && iface->get_analog_gain(GST_PHOTOGRAPHY(self), &gain)) {
			g_value_set_float(value, gain);
		}
		break;
	}
	case PROP_COLOUR_TONE:
	case PROP_SCENE_MODE:
	case PROP_FLASH_MODE:
	case PROP_NOISE_REDUCTION:
	case PROP_ISO_SPEED:
	case PROP_APERTURE:
	case PROP_IMAGE_CAPTURE_SUPPORTED_CAPS:
	case PROP_IMAGE_PREVIEW_SUPPORTED_CAPS:
	case PROP_FLICKER_MODE:
	case PROP_FOCUS_MODE:
	case PROP_ZOOM:
	case PROP_SMOOTH_ZOOM:
	case PROP_WHITE_POINT:
	case PROP_MIN_EXPOSURE_TIME:
	case PROP_MAX_EXPOSURE_TIME:
	case PROP_LENS_FOCUS:
	case PROP_COLOR_TEMPERATURE:
	case PROP_OVERRIDE_SENSOR_MODE: {
		if (self->state->override_sensor_configuration_) {
			// Construct the width:height:bitdepth string
			gchar *prop_string = g_strdup_printf("%d:%d:%d", self->state->override_sensor_configuration_->outputSize.width, self->state->override_sensor_configuration_->outputSize.height, self->state->override_sensor_configuration_->bitDepth);
			g_value_set_string(value, prop_string);
		} else {
			g_value_set_string(value, "false");
		}
	} break;
	case PROP_SATURATION: {
		gfloat saturation;

		GstPhotographyInterface *iface = GST_PHOTOGRAPHY_GET_INTERFACE(self);
		if (iface->get_saturation != NULL && iface->get_saturation(GST_PHOTOGRAPHY(self), &saturation)) {
			g_value_set_float(value, saturation);
		}
		break;
	}
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

	GstPad *pad = gst_pad_new_from_template(templ, "src");
	state->srcpads_.push_back(pad);
	gst_element_add_pad(GST_ELEMENT(self), pad);
	gst_child_proxy_child_added(GST_CHILD_PROXY(self), G_OBJECT(pad), GST_OBJECT_NAME(pad));

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

	gst_child_proxy_child_added(GST_CHILD_PROXY(self), G_OBJECT(pad), GST_OBJECT_NAME(pad));

	return reinterpret_cast<GstPad *>(g_steal_pointer(&pad));
}

static void
gst_libcamera_src_release_pad(GstElement *element, GstPad *pad)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(element);

	gst_child_proxy_child_removed(GST_CHILD_PROXY(self), G_OBJECT(pad), GST_OBJECT_NAME(pad));

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
				       "Nicolas Dufresne <nicolas.dufresne@collabora.com>, Raphael Dürscheid <raphael.duerscheid@aivero.com>");
	gst_element_class_add_static_pad_template_with_gtype(element_class,
							     &src_template,
							     GST_TYPE_LIBCAMERA_PAD);
	gst_element_class_add_static_pad_template_with_gtype(element_class,
							     &request_src_template,
							     GST_TYPE_LIBCAMERA_PAD);

	GParamSpec *spec = g_param_spec_string("camera-name", "Camera Name",
					       "Select by name which camera to use.", nullptr,
					       (GParamFlags)(GST_PARAM_MUTABLE_READY | G_PARAM_CONSTRUCT | G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
	g_object_class_install_property(object_class, PROP_CAMERA_NAME, spec);

	spec = g_param_spec_enum("auto-focus-mode",
				 "Set auto-focus mode",
				 "Available options: AfModeManual, "
				 "AfModeAuto or AfModeContinuous.",
				 gst_libcamera_auto_focus_get_type(),
				 static_cast<gint>(controls::AfModeManual),
				 G_PARAM_WRITABLE);
	g_object_class_install_property(object_class, PROP_AUTO_FOCUS_MODE, spec);

	/* Override GstPhotography properties */
	g_object_class_override_property(object_class, PROP_WB_MODE,
					 GST_PHOTOGRAPHY_PROP_WB_MODE);

	g_object_class_override_property(object_class, PROP_COLOUR_TONE,
					 GST_PHOTOGRAPHY_PROP_COLOR_TONE);

	g_object_class_override_property(object_class, PROP_SCENE_MODE,
					 GST_PHOTOGRAPHY_PROP_SCENE_MODE);

	g_object_class_override_property(object_class, PROP_FLASH_MODE,
					 GST_PHOTOGRAPHY_PROP_FLASH_MODE);

	g_object_class_override_property(object_class, PROP_NOISE_REDUCTION,
					 GST_PHOTOGRAPHY_PROP_NOISE_REDUCTION);

	g_object_class_override_property(object_class, PROP_CAPABILITIES,
					 GST_PHOTOGRAPHY_PROP_CAPABILITIES);

	g_object_class_override_property(object_class, PROP_EV_COMP,
					 GST_PHOTOGRAPHY_PROP_EV_COMP);

	g_object_class_override_property(object_class, PROP_ISO_SPEED,
					 GST_PHOTOGRAPHY_PROP_ISO_SPEED);

	g_object_class_override_property(object_class, PROP_APERTURE,
					 GST_PHOTOGRAPHY_PROP_APERTURE);

	g_object_class_override_property(object_class, PROP_EXPOSURE_MODE,
					 GST_PHOTOGRAPHY_PROP_EXPOSURE_MODE);

	g_object_class_override_property(object_class,
					 PROP_IMAGE_CAPTURE_SUPPORTED_CAPS,
					 GST_PHOTOGRAPHY_PROP_IMAGE_CAPTURE_SUPPORTED_CAPS);

	g_object_class_override_property(object_class,
					 PROP_IMAGE_PREVIEW_SUPPORTED_CAPS,
					 GST_PHOTOGRAPHY_PROP_IMAGE_PREVIEW_SUPPORTED_CAPS);

	g_object_class_override_property(object_class, PROP_FLICKER_MODE,
					 GST_PHOTOGRAPHY_PROP_FLICKER_MODE);

	g_object_class_override_property(object_class, PROP_FOCUS_MODE,
					 GST_PHOTOGRAPHY_PROP_FOCUS_MODE);

	g_object_class_override_property(object_class, PROP_ZOOM,
					 GST_PHOTOGRAPHY_PROP_ZOOM);

	g_object_class_override_property(object_class, PROP_WHITE_POINT,
					 GST_PHOTOGRAPHY_PROP_WHITE_POINT);

	g_object_class_override_property(object_class, PROP_MIN_EXPOSURE_TIME,
					 GST_PHOTOGRAPHY_PROP_MIN_EXPOSURE_TIME);

	g_object_class_override_property(object_class, PROP_MAX_EXPOSURE_TIME,
					 GST_PHOTOGRAPHY_PROP_MAX_EXPOSURE_TIME);

	g_object_class_override_property(object_class, PROP_LENS_FOCUS,
					 GST_PHOTOGRAPHY_PROP_LENS_FOCUS);

	g_object_class_override_property(object_class, PROP_EXPOSURE_TIME,
					 GST_PHOTOGRAPHY_PROP_EXPOSURE_TIME);

	g_object_class_override_property(object_class, PROP_COLOR_TEMPERATURE,
					 GST_PHOTOGRAPHY_PROP_COLOR_TEMPERATURE);

	g_object_class_override_property(object_class, PROP_ANALOG_GAIN,
					 GST_PHOTOGRAPHY_PROP_ANALOG_GAIN);

	g_object_class_override_property(object_class, PROP_SATURATION,
					 GST_PHOTOGRAPHY_PROP_SATURATION);

	spec = g_param_spec_string("override-sensormode", "Override sensor mode",
				   "You can override the sensor mode using the former mmal '-mode' syntax: 'width:height:bitDepth. Defaults to no override", NULL,
				   (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
	g_object_class_install_property(object_class, PROP_OVERRIDE_SENSOR_MODE, spec);
}

static GstPhotographyCaps
gst_libcamera_src_get_capabilities(GstPhotography *p)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);

	return self->state->capabilities;
}

static void
gst_libcamera_src_set_autofocus(GstPhotography *p, gboolean on)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);

	if (on)
		state->auto_focus_mode = controls::AfModeAuto;
	else
		state->auto_focus_mode = controls::AfModeManual;
}

static gboolean
gst_libcamera_src_get_focus_mode(GstPhotography *p,
				 GstPhotographyFocusMode *focus_mode)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);
	gboolean ret = FALSE;

	switch (state->auto_focus_mode) {
	case controls::AfModeAuto:
		*focus_mode = GST_PHOTOGRAPHY_FOCUS_MODE_AUTO;
		ret = TRUE;
		break;
	case controls::AfModeContinuous:
		*focus_mode = GST_PHOTOGRAPHY_FOCUS_MODE_CONTINUOUS_NORMAL;
		ret = TRUE;
		break;
	case controls::AfModeManual:
		*focus_mode = GST_PHOTOGRAPHY_FOCUS_MODE_MANUAL;
		ret = TRUE;
		break;
	}

	return ret;
}

static gboolean
gst_libcamera_src_set_focus_mode(GstPhotography *p,
				 GstPhotographyFocusMode focus_mode)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;

	controls::AfModeEnum auto_focus_mode = controls::AfModeAuto;

	switch (focus_mode) {
	case GST_PHOTOGRAPHY_FOCUS_MODE_AUTO:
		auto_focus_mode = controls::AfModeAuto;
		break;
	case GST_PHOTOGRAPHY_FOCUS_MODE_CONTINUOUS_NORMAL:
		auto_focus_mode = controls::AfModeContinuous;
		break;
	case GST_PHOTOGRAPHY_FOCUS_MODE_MANUAL:
		auto_focus_mode = controls::AfModeManual;
		break;
	default:
		break; /* Ignore unhandled modes */
	}

	GLibLocker locker(&state->lock_);
	if (state->cam_ != NULL) {
		/* If the camera is opened, check whether we can actually set the control */
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::AfMode) == infoMap.end()) {
			return FALSE;
		}
	}

	if (state->auto_focus_mode != auto_focus_mode) {
		GST_LOG_OBJECT(self, "Setting autofocus mode %u", auto_focus_mode);
		state->auto_focus_mode = auto_focus_mode;
		state->pendingControls_.set(controls::AfMode, state->auto_focus_mode);
	}

	return TRUE;
}

static gboolean
gst_libcamera_src_set_exposure(GstPhotography *p, guint32 exposure)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);

	if (state->cam_ != NULL) {
		/* If the camera is opened, check whether we can actually set the control */
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::ExposureTime) == infoMap.end())
			return FALSE;
	}

	if (state->exposure_time != (int32_t)exposure) {
		GST_LOG_OBJECT(self, "Setting exposure time %u", exposure);
		state->exposure_time = exposure;
		state->pendingControls_.set(controls::ExposureTime,
					    state->exposure_time);
	}

	return TRUE;
}

static gboolean
gst_libcamera_src_get_exposure(GstPhotography *p, guint32 *exposure)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	gboolean ret = FALSE;
	GLibLocker locker(&state->lock_);
	if (state->cam_ == NULL)
		return FALSE;

	const ControlInfoMap &infoMap = state->cam_->controls();
	if (infoMap.find(&controls::ExposureTime) != infoMap.end()) {
		*exposure = (guint32)state->exposure_time;
		ret = TRUE;
	}

	return ret;
}

static gboolean
gst_libcamera_src_set_white_balance_mode(GstPhotography *p,
					 GstPhotographyWhiteBalanceMode wb_mode)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;

	controls::AwbModeEnum awb_mode = controls::AwbAuto;
	switch (wb_mode) {
	case GST_PHOTOGRAPHY_WB_MODE_AUTO:
	case GST_PHOTOGRAPHY_WB_MODE_SUNSET:
		awb_mode = controls::AwbAuto;
		break;
	case GST_PHOTOGRAPHY_WB_MODE_DAYLIGHT:
		awb_mode = controls::AwbDaylight;
		break;
	case GST_PHOTOGRAPHY_WB_MODE_CLOUDY:
		awb_mode = controls::AwbCloudy;
		break;
	case GST_PHOTOGRAPHY_WB_MODE_TUNGSTEN:
		awb_mode = controls::AwbTungsten;
		break;
	case GST_PHOTOGRAPHY_WB_MODE_FLUORESCENT:
		awb_mode = controls::AwbFluorescent;
		break;
	case GST_PHOTOGRAPHY_WB_MODE_WARM_FLUORESCENT:
		awb_mode = controls::AwbIncandescent;
		break;
	case GST_PHOTOGRAPHY_WB_MODE_SHADE:
		awb_mode = controls::AwbIndoor;
		break;
	default:
		break; /* Ignore unhandled modes */
	}

	GLibLocker locker(&state->lock_);
	if (state->cam_ != NULL) {
		/* If the camera is opened, check whether we can actually set the control */
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::AwbMode) == infoMap.end())
			return FALSE;
	}

	if (state->awb_mode != awb_mode) {
		GST_LOG_OBJECT(self, "Setting AWB mode %u", awb_mode);
		state->awb_mode = awb_mode;
		state->pendingControls_.set(controls::AwbMode, state->awb_mode);
	}

	return TRUE;
}

static gboolean
gst_libcamera_src_get_white_balance_mode(GstPhotography *p,
					 GstPhotographyWhiteBalanceMode *wb_mode)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);
	gboolean ret = FALSE;

	switch (state->awb_mode) {
	case controls::AwbAuto:
		*wb_mode = GST_PHOTOGRAPHY_WB_MODE_AUTO;
		ret = TRUE;
		break;
	case controls::AwbDaylight:
		*wb_mode = GST_PHOTOGRAPHY_WB_MODE_DAYLIGHT;
		ret = TRUE;
		break;
	case controls::AwbCloudy:
		*wb_mode = GST_PHOTOGRAPHY_WB_MODE_CLOUDY;
		ret = TRUE;
		break;
	case controls::AwbTungsten:
		*wb_mode = GST_PHOTOGRAPHY_WB_MODE_TUNGSTEN;
		ret = TRUE;
		break;
	case controls::AwbFluorescent:
		*wb_mode = GST_PHOTOGRAPHY_WB_MODE_FLUORESCENT;
		ret = TRUE;
		break;
	case controls::AwbIncandescent:
		*wb_mode = GST_PHOTOGRAPHY_WB_MODE_WARM_FLUORESCENT;
		ret = TRUE;
		break;
	case controls::AwbIndoor:
		*wb_mode = GST_PHOTOGRAPHY_WB_MODE_SHADE;
		ret = TRUE;
		break;
	default:
		break; /* Ignore unhandled modes */
	}

	return ret;
}

static gboolean
gst_libcamera_src_set_ev_compensation(GstPhotography *p, gfloat ev_comp)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);

	if (state->cam_ != NULL) {
		/* If the camera is opened, check whether we can actually set the control */
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::ExposureValue) == infoMap.end())
			return FALSE;
	}

	if (state->exposure_value != ev_comp) {
		GST_LOG_OBJECT(self, "Setting exposure value %f", ev_comp);
		state->exposure_value = ev_comp;
		state->pendingControls_.set(controls::ExposureValue,
					    state->exposure_value);
	}

	return TRUE;
}

static gboolean
gst_libcamera_src_get_ev_compensation(GstPhotography *p, gfloat *ev_comp)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	gboolean ret = FALSE;
	GLibLocker locker(&state->lock_);
	if (state->cam_ == NULL)
		return FALSE;

	const ControlInfoMap &infoMap = state->cam_->controls();
	if (infoMap.find(&controls::ExposureValue) != infoMap.end()) {
		*ev_comp = state->exposure_value;
		ret = TRUE;
	}

	return ret;
}

static gboolean
gst_libcamera_src_set_exposure_mode(GstPhotography *p,
				    GstPhotographyExposureMode exposure_mode)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;

	controls::AeExposureModeEnum ae_mode = controls::ExposureNormal;

	switch (exposure_mode) {
	case GST_PHOTOGRAPHY_EXPOSURE_MODE_AUTO:
		ae_mode = controls::ExposureNormal;
		break;
	case GST_PHOTOGRAPHY_EXPOSURE_MODE_MANUAL:
		ae_mode = controls::ExposureCustom;
		break;
	default:
		break; /* Ignore unhandled modes */
	}

	GLibLocker locker(&state->lock_);
	if (state->cam_ != NULL) {
		/* If the camera is opened, check whether we can actually set the control */
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::AeExposureMode) == infoMap.end())
			return FALSE;
	}

	if (state->exposure_mode != ae_mode) {
		GST_LOG_OBJECT(self, "Setting auto exposure mode %u", ae_mode);

		state->exposure_mode = ae_mode;
		state->pendingControls_.set(controls::AeExposureMode,
					    state->exposure_mode);
	}

	return TRUE;
}

static gboolean
gst_libcamera_src_get_exposure_mode(GstPhotography *p,
				    GstPhotographyExposureMode *exposure_mode)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);
	gboolean ret = FALSE;

	switch (state->exposure_mode) {
	case controls::ExposureNormal:
	case controls::ExposureLong:
	case controls::ExposureShort:
		*exposure_mode = GST_PHOTOGRAPHY_EXPOSURE_MODE_AUTO;
		ret = TRUE;
		break;
	case controls::ExposureCustom:
		*exposure_mode = GST_PHOTOGRAPHY_EXPOSURE_MODE_MANUAL;
		ret = TRUE;
		break;
	}

	return ret;
}

static gboolean
gst_libcamera_src_set_analog_gain(GstPhotography *p, gfloat analog_gain)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);

	if (state->cam_ != NULL) {
		/* If the camera is opened, check whether we can actually set the control */
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::AnalogueGain) == infoMap.end())
			return FALSE;
	}

	if (state->analog_gain != analog_gain) {
		GST_LOG_OBJECT(self, "Setting analog gain %f", analog_gain);
		state->analog_gain = analog_gain;
		state->pendingControls_.set(controls::AnalogueGain, state->analog_gain);
	}

	return TRUE;
}

static gboolean
gst_libcamera_src_get_analog_gain(GstPhotography *p, gfloat *analog_gain)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	gboolean ret = FALSE;
	GLibLocker locker(&state->lock_);
	if (state->cam_ == NULL)
		return FALSE;

	const ControlInfoMap &infoMap = state->cam_->controls();
	if (infoMap.find(&controls::AnalogueGain) != infoMap.end()) {
		*analog_gain = state->analog_gain;
		ret = TRUE;
	}

	return ret;
}

static gboolean
gst_libcamera_src_set_saturation(GstPhotography *p, gfloat saturation)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	GLibLocker locker(&state->lock_);

	if (state->cam_ != NULL) {
		/* If the camera is opened, check whether we can actually set the control */
		const ControlInfoMap &infoMap = state->cam_->controls();
		if (infoMap.find(&controls::Saturation) == infoMap.end())
			return FALSE;
	}

	if (state->saturation != saturation) {
		GST_LOG_OBJECT(self, "Setting saturation %f", saturation);
		state->saturation = saturation;
		state->pendingControls_.set(controls::Saturation, state->saturation);
	}

	return TRUE;
}

static gboolean
gst_libcamera_src_get_saturation(GstPhotography *p, gfloat *saturation)
{
	GstLibcameraSrc *self = GST_LIBCAMERA_SRC(p);
	GstLibcameraSrcState *state = self->state;
	gboolean ret = FALSE;
	GLibLocker locker(&state->lock_);
	if (state->cam_ == NULL)
		return FALSE;

	const ControlInfoMap &infoMap = state->cam_->controls();
	if (infoMap.find(&controls::Saturation) != infoMap.end()) {
		*saturation = state->saturation;
		ret = TRUE;
	}

	return ret;
}

static void
gst_libcamera_src_photography_init(gpointer g_iface)
{
	GstPhotographyInterface *iface = (GstPhotographyInterface *)g_iface;

	iface->get_capabilities = gst_libcamera_src_get_capabilities;
	iface->set_autofocus = gst_libcamera_src_set_autofocus;
	iface->get_focus_mode = gst_libcamera_src_get_focus_mode;
	iface->set_focus_mode = gst_libcamera_src_set_focus_mode;

	iface->set_exposure = gst_libcamera_src_set_exposure;
	iface->get_exposure = gst_libcamera_src_get_exposure;

	iface->set_white_balance_mode = gst_libcamera_src_set_white_balance_mode;
	iface->get_white_balance_mode = gst_libcamera_src_get_white_balance_mode;

	iface->set_ev_compensation = gst_libcamera_src_set_ev_compensation;
	iface->get_ev_compensation = gst_libcamera_src_get_ev_compensation;

	iface->set_exposure_mode = gst_libcamera_src_set_exposure_mode;
	iface->get_exposure_mode = gst_libcamera_src_get_exposure_mode;

	iface->set_analog_gain = gst_libcamera_src_set_analog_gain;
	iface->get_analog_gain = gst_libcamera_src_get_analog_gain;

	iface->set_saturation = gst_libcamera_src_set_saturation;
	iface->get_saturation = gst_libcamera_src_get_saturation;
}

/* GstChildProxy implementation */
static GObject *
gst_libcamera_src_child_proxy_get_child_by_index(GstChildProxy *child_proxy,
						 guint index)
{
	GLibLocker lock(GST_OBJECT(child_proxy));
	GObject *obj = nullptr;

	obj = reinterpret_cast<GObject *>(g_list_nth_data(GST_ELEMENT(child_proxy)->srcpads, index));
	if (obj)
		gst_object_ref(obj);

	return obj;
}

static guint
gst_libcamera_src_child_proxy_get_children_count(GstChildProxy *child_proxy)
{
	GLibLocker lock(GST_OBJECT(child_proxy));
	return GST_ELEMENT_CAST(child_proxy)->numsrcpads;
}

static void
gst_libcamera_src_child_proxy_init(gpointer g_iface, [[maybe_unused]] gpointer iface_data)
{
	GstChildProxyInterface *iface = reinterpret_cast<GstChildProxyInterface *>(g_iface);
	iface->get_child_by_index = gst_libcamera_src_child_proxy_get_child_by_index;
	iface->get_children_count = gst_libcamera_src_child_proxy_get_children_count;
}
