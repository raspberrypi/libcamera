/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Android Camera HAL Operations
 */

#include "camera_ops.h"

#include <system/camera_metadata.h>

#include "camera_device.h"

using namespace libcamera;

/*
 * Translation layer between the Android Camera HAL device operations and the
 * CameraDevice.
 */

static int hal_dev_initialize(const struct camera3_device *dev,
			      const camera3_callback_ops_t *callback_ops)
{
	if (!dev)
		return -EINVAL;

	CameraDevice *camera = reinterpret_cast<CameraDevice *>(dev->priv);
	camera->setCallbacks(callback_ops);

	return 0;
}

static int hal_dev_configure_streams(const struct camera3_device *dev,
				     camera3_stream_configuration_t *stream_list)
{
	if (!dev)
		return -EINVAL;

	CameraDevice *camera = reinterpret_cast<CameraDevice *>(dev->priv);
	return camera->configureStreams(stream_list);
}

static const camera_metadata_t *
hal_dev_construct_default_request_settings(const struct camera3_device *dev,
					   int type)
{
	if (!dev)
		return nullptr;

	CameraDevice *camera = reinterpret_cast<CameraDevice *>(dev->priv);
	return camera->constructDefaultRequestSettings(type);
}

static int hal_dev_process_capture_request(const struct camera3_device *dev,
					   camera3_capture_request_t *request)
{
	if (!dev)
		return -EINVAL;

	CameraDevice *camera = reinterpret_cast<CameraDevice *>(dev->priv);
	return camera->processCaptureRequest(request);
}

static void hal_dev_dump([[maybe_unused]] const struct camera3_device *dev,
			 [[maybe_unused]] int fd)
{
}

static int hal_dev_flush(const struct camera3_device *dev)
{
	if (!dev)
		return -EINVAL;

	CameraDevice *camera = reinterpret_cast<CameraDevice *>(dev->priv);
	camera->flush();

	return 0;
}

int hal_dev_close(hw_device_t *hw_device)
{
	if (!hw_device)
		return -EINVAL;

	camera3_device_t *dev = reinterpret_cast<camera3_device_t *>(hw_device);
	CameraDevice *camera = reinterpret_cast<CameraDevice *>(dev->priv);

	camera->close();

	return 0;
}

camera3_device_ops hal_dev_ops = {
	.initialize = hal_dev_initialize,
	.configure_streams = hal_dev_configure_streams,
	.register_stream_buffers = nullptr,
	.construct_default_request_settings = hal_dev_construct_default_request_settings,
	.process_capture_request = hal_dev_process_capture_request,
	.get_metadata_vendor_tag_ops = nullptr,
	.dump = hal_dev_dump,
	.flush = hal_dev_flush,
	.reserved = { nullptr },
};
