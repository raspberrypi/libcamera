/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_proxy.cpp - Proxy to camera devices
 */

#include "camera_proxy.h"

#include <system/camera_metadata.h>

#include "log.h"
#include "message.h"
#include "utils.h"

#include "camera_device.h"
#include "thread_rpc.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL);

/*
 * \class CameraProxy
 *
 * The CameraProxy wraps a CameraDevice and implements the camera3_device_t
 * API, bridging calls received from the camera framework to the CameraDevice.
 *
 * Bridging operation calls between the framework and the CameraDevice is
 * required as the two run in two different threads and certain operations,
 * such as queueing a new capture request to the camera, shall be called in
 * the thread that dispatches events. Other operations do not require any
 * bridging and resolve to direct function calls on the CameraDevice instance
 * instead.
 */

static int hal_dev_initialize(const struct camera3_device *dev,
			      const camera3_callback_ops_t *callback_ops)
{
	if (!dev)
		return -EINVAL;

	CameraProxy *proxy = reinterpret_cast<CameraProxy *>(dev->priv);
	proxy->initialize(callback_ops);

	return 0;
}

static int hal_dev_configure_streams(const struct camera3_device *dev,
				     camera3_stream_configuration_t *stream_list)
{
	if (!dev)
		return -EINVAL;

	CameraProxy *proxy = reinterpret_cast<CameraProxy *>(dev->priv);
	return proxy->configureStreams(stream_list);
}

static const camera_metadata_t *
hal_dev_construct_default_request_settings(const struct camera3_device *dev,
					   int type)
{
	if (!dev)
		return nullptr;

	CameraProxy *proxy = reinterpret_cast<CameraProxy *>(dev->priv);
	return proxy->constructDefaultRequestSettings(type);
}

static int hal_dev_process_capture_request(const struct camera3_device *dev,
					   camera3_capture_request_t *request)
{
	if (!dev)
		return -EINVAL;

	CameraProxy *proxy = reinterpret_cast<CameraProxy *>(dev->priv);
	return proxy->processCaptureRequest(request);
}

static void hal_dev_dump(const struct camera3_device *dev, int fd)
{
}

static int hal_dev_flush(const struct camera3_device *dev)
{
	return 0;
}

static int hal_dev_close(hw_device_t *hw_device)
{
	if (!hw_device)
		return -EINVAL;

	camera3_device_t *dev = reinterpret_cast<camera3_device_t *>(hw_device);
	CameraProxy *proxy = reinterpret_cast<CameraProxy *>(dev->priv);

	proxy->close();

	return 0;
}

static camera3_device_ops hal_dev_ops = {
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

CameraProxy::CameraProxy(unsigned int id, std::shared_ptr<Camera> camera)
	: id_(id)
{
	cameraDevice_ = new CameraDevice(id, camera);
}

CameraProxy::~CameraProxy()
{
	delete cameraDevice_;
}

int CameraProxy::open(const hw_module_t *hardwareModule)
{
	int ret = cameraDevice_->open();
	if (ret)
		return ret;

	/* Initialize the hw_device_t in the instance camera3_module_t. */
	camera3Device_.common.tag = HARDWARE_DEVICE_TAG;
	camera3Device_.common.version = CAMERA_DEVICE_API_VERSION_3_3;
	camera3Device_.common.module = (hw_module_t *)hardwareModule;
	camera3Device_.common.close = hal_dev_close;

	/*
	 * The camera device operations. These actually implement
	 * the Android Camera HALv3 interface.
	 */
	camera3Device_.ops = &hal_dev_ops;
	camera3Device_.priv = this;

	return 0;
}

void CameraProxy::close()
{
	ThreadRpc rpcRequest;
	rpcRequest.tag = ThreadRpc::Close;

	threadRpcCall(rpcRequest);
}

void CameraProxy::initialize(const camera3_callback_ops_t *callbacks)
{
	cameraDevice_->setCallbacks(callbacks);
}

const camera_metadata_t *CameraProxy::getStaticMetadata()
{
	return cameraDevice_->getStaticMetadata();
}

const camera_metadata_t *CameraProxy::constructDefaultRequestSettings(int type)
{
	return cameraDevice_->constructDefaultRequestSettings(type);
}

int CameraProxy::configureStreams(camera3_stream_configuration_t *stream_list)
{
	return cameraDevice_->configureStreams(stream_list);
}

int CameraProxy::processCaptureRequest(camera3_capture_request_t *request)
{
	ThreadRpc rpcRequest;
	rpcRequest.tag = ThreadRpc::ProcessCaptureRequest;
	rpcRequest.request = request;

	threadRpcCall(rpcRequest);

	return 0;
}

void CameraProxy::threadRpcCall(ThreadRpc &rpcRequest)
{
	std::unique_ptr<ThreadRpcMessage> message =
				utils::make_unique<ThreadRpcMessage>();
	message->rpc = &rpcRequest;

	cameraDevice_->postMessage(std::move(message));
	rpcRequest.waitDelivery();
}
