/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcameraprovider.c - GStreamer Device Provider
 */

#include <array>

#include "gstlibcameraprovider.h"

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include "gstlibcamerasrc.h"
#include "gstlibcamera-utils.h"

using namespace libcamera;

GST_DEBUG_CATEGORY_STATIC(provider_debug);
#define GST_CAT_DEFAULT provider_debug

/**
 * \struct _GstLibcameraDevice
 * \brief libcamera GstDevice implementation
 *
 * This object is used by GstLibcameraProvider to abstract a libcamera
 * device. It also provides helpers to create and configure the
 * libcamerasrc GstElement to be used with this device. The implementation is
 * private to the plugin.
 */

enum {
	PROP_DEVICE_NAME = 1,
	PROP_AUTO_FOCUS_MODE = 2,
};

#define GST_TYPE_LIBCAMERA_DEVICE gst_libcamera_device_get_type()
G_DECLARE_FINAL_TYPE(GstLibcameraDevice, gst_libcamera_device,
		     GST_LIBCAMERA, DEVICE, GstDevice)

struct _GstLibcameraDevice {
	GstDevice parent;
	gchar *name;
	controls::AfModeEnum auto_focus_mode = controls::AfModeManual;
};

G_DEFINE_TYPE(GstLibcameraDevice, gst_libcamera_device, GST_TYPE_DEVICE)

static GstElement *
gst_libcamera_device_create_element(GstDevice *device, const gchar *name)
{
	GstElement *source = gst_element_factory_make("libcamerasrc", name);

	/*
	 * Provider and source lives in the same plugin, so making the source
	 * should never fail.
	 */
	g_assert(source);

	g_object_set(source, "camera-name", GST_LIBCAMERA_DEVICE(device)->name, nullptr);
	g_object_set(source, "auto-focus-mode", GST_LIBCAMERA_DEVICE(device)->auto_focus_mode, nullptr);

	return source;
}

static gboolean
gst_libcamera_device_reconfigure_element(GstDevice *device,
					 GstElement *element)
{
	if (!GST_LIBCAMERA_IS_SRC(element))
		return FALSE;

	g_object_set(element, "camera-name", GST_LIBCAMERA_DEVICE(device)->name, nullptr);

	return TRUE;
}

static void
gst_libcamera_device_set_property(GObject *object, guint prop_id,
				  const GValue *value, GParamSpec *pspec)
{
	GstLibcameraDevice *device = GST_LIBCAMERA_DEVICE(object);

	switch (prop_id) {
	case PROP_DEVICE_NAME:
		device->name = g_value_dup_string(value);
		break;
	case PROP_AUTO_FOCUS_MODE:
		device->auto_focus_mode = static_cast<controls::AfModeEnum>(g_value_get_enum(value));
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		break;
	}
}

static void
gst_libcamera_device_init([[maybe_unused]] GstLibcameraDevice *self)
{
}

static void
gst_libcamera_device_finalize(GObject *object)
{
	GstLibcameraDevice *self = GST_LIBCAMERA_DEVICE(object);
	gpointer klass = gst_libcamera_device_parent_class;

	g_free(self->name);

	G_OBJECT_CLASS(klass)->finalize(object);
}

static void
gst_libcamera_device_class_init(GstLibcameraDeviceClass *klass)
{
	GstDeviceClass *device_class = GST_DEVICE_CLASS(klass);
	GObjectClass *object_class = G_OBJECT_CLASS(klass);

	device_class->create_element = gst_libcamera_device_create_element;
	device_class->reconfigure_element = gst_libcamera_device_reconfigure_element;

	object_class->set_property = gst_libcamera_device_set_property;
	object_class->finalize = gst_libcamera_device_finalize;

	GParamSpec *pspec = g_param_spec_string("name", "Name",
						"The name of the camera device", "",
						(GParamFlags)(G_PARAM_STATIC_STRINGS | G_PARAM_WRITABLE |
							      G_PARAM_CONSTRUCT_ONLY));
	g_object_class_install_property(object_class, PROP_DEVICE_NAME, pspec);

	pspec = g_param_spec_enum("auto-focus-mode",
				  "Set auto-focus mode",
				  "Available options: AfModeManual, "
				  "AfModeAuto or AfModeContinuous.",
				  gst_libcamera_auto_focus_get_type(),
				  static_cast<gint>(controls::AfModeManual),
				  G_PARAM_WRITABLE);
	g_object_class_install_property(object_class, PROP_AUTO_FOCUS_MODE, pspec);
}

static GstDevice *
gst_libcamera_device_new(const std::shared_ptr<Camera> &camera)
{
	static const std::array roles{ StreamRole::VideoRecording };
	g_autoptr(GstCaps) caps = gst_caps_new_empty();
	const gchar *name = camera->id().c_str();

	std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration(roles);
	if (!config || config->size() != roles.size()) {
		GST_ERROR("Failed to generate a default configuration for %s", name);
		return nullptr;
	}

	for (const StreamConfiguration &stream_cfg : *config) {
		GstCaps *sub_caps = gst_libcamera_stream_formats_to_caps(stream_cfg.formats());
		if (sub_caps)
			gst_caps_append(caps, sub_caps);
	}

	return GST_DEVICE(g_object_new(GST_TYPE_LIBCAMERA_DEVICE,
				       /* \todo Use a unique identifier instead of camera name. */
				       "name", name,
				       "display-name", name,
				       "caps", caps,
				       "device-class", "Source/Video",
				       nullptr));
}

/**
 * \struct _GstLibcameraProvider
 * \brief libcamera GstDeviceProvider implementation
 *
 * This GstFeature is used by GstDeviceMonitor to probe the available
 * libcamera devices. The implementation is private to the plugin.
 */

struct _GstLibcameraProvider {
	GstDeviceProvider parent;
};

G_DEFINE_TYPE_WITH_CODE(GstLibcameraProvider, gst_libcamera_provider,
			GST_TYPE_DEVICE_PROVIDER,
			GST_DEBUG_CATEGORY_INIT(provider_debug, "libcamera-provider", 0,
						"libcamera Device Provider"))

static GList *
gst_libcamera_provider_probe(GstDeviceProvider *provider)
{
	GstLibcameraProvider *self = GST_LIBCAMERA_PROVIDER(provider);
	std::shared_ptr<CameraManager> cm;
	GList *devices = nullptr;
	gint ret;

	GST_INFO_OBJECT(self, "Probing cameras using libcamera");

	/* \todo Move the CameraMananger start()/stop() calls into
	 * GstDeviceProvider start()/stop() virtual function when CameraMananger
	 * gains monitoring support. Meanwhile we need to cycle start()/stop()
	 * to ensure every probe() calls return the latest list.
	 */
	cm = gst_libcamera_get_camera_manager(ret);
	if (ret) {
		GST_ERROR_OBJECT(self, "Failed to retrieve device list: %s",
				 g_strerror(-ret));
		return nullptr;
	}

	for (const std::shared_ptr<Camera> &camera : cm->cameras()) {
		GST_INFO_OBJECT(self, "Found camera '%s'", camera->id().c_str());

		GstDevice *dev = gst_libcamera_device_new(camera);
		if (!dev) {
			GST_ERROR_OBJECT(self, "Failed to add camera '%s'",
					 camera->id().c_str());
			return nullptr;
		}

		devices = g_list_append(devices,
					g_object_ref_sink(dev));
	}

	return devices;
}

static void
gst_libcamera_provider_init(GstLibcameraProvider *self)
{
	GstDeviceProvider *provider = GST_DEVICE_PROVIDER(self);

	/* Avoid devices being duplicated. */
	gst_device_provider_hide_provider(provider, "v4l2deviceprovider");
}

static void
gst_libcamera_provider_class_init(GstLibcameraProviderClass *klass)
{
	GstDeviceProviderClass *provider_class = GST_DEVICE_PROVIDER_CLASS(klass);

	provider_class->probe = gst_libcamera_provider_probe;

	gst_device_provider_class_set_metadata(provider_class,
					       "libcamera Device Provider",
					       "Source/Video",
					       "List camera device using libcamera",
					       "Nicolas Dufresne <nicolas.dufresne@collabora.com>");
}
