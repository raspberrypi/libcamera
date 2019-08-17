/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_proxy.h - Proxy to camera devices
 */
#ifndef __ANDROID_CAMERA_PROXY_H__
#define __ANDROID_CAMERA_PROXY_H__

#include <memory>

#include <hardware/camera3.h>

#include <libcamera/camera.h>

class CameraDevice;
class ThreadRpc;

class CameraProxy
{
public:
	CameraProxy(unsigned int id, const std::shared_ptr<libcamera::Camera> &camera);
	~CameraProxy();

	int open(const hw_module_t *hardwareModule);
	void close();

	void initialize(const camera3_callback_ops_t *callbacks);
	const camera_metadata_t *getStaticMetadata();
	const camera_metadata_t *constructDefaultRequestSettings(int type);
	int configureStreams(camera3_stream_configuration_t *stream_list);
	int processCaptureRequest(camera3_capture_request_t *request);

	unsigned int id() const { return id_; }
	camera3_device_t *camera3Device() { return &camera3Device_; }

private:
	void threadRpcCall(ThreadRpc &rpcRequest);

	unsigned int id_;
	CameraDevice *cameraDevice_;
	camera3_device_t camera3Device_;
};

#endif /* __ANDROID_CAMERA_PROXY_H__ */
