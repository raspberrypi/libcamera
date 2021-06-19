/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_capabilities.h - Camera static properties manager
 */
#ifndef __ANDROID_CAMERA_CAPABILITIES_H__
#define __ANDROID_CAMERA_CAPABILITIES_H__

#include <map>
#include <memory>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/class.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>

#include "camera_metadata.h"

class CameraCapabilities
{
public:
	CameraCapabilities() = default;

	int initialize(std::shared_ptr<libcamera::Camera> camera,
		       int orientation, int facing);

	CameraMetadata *staticMetadata() const { return staticMetadata_.get(); }
	libcamera::PixelFormat toPixelFormat(int format) const;
	unsigned int maxJpegBufferSize() const { return maxJpegBufferSize_; }

	std::unique_ptr<CameraMetadata> requestTemplatePreview() const;
	std::unique_ptr<CameraMetadata> requestTemplateVideo() const;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraCapabilities)

	struct Camera3StreamConfiguration {
		libcamera::Size resolution;
		int androidFormat;
	};

	std::vector<libcamera::Size>
	getYUVResolutions(libcamera::CameraConfiguration *cameraConfig,
			  const libcamera::PixelFormat &pixelFormat,
			  const std::vector<libcamera::Size> &resolutions);
	std::vector<libcamera::Size>
	getRawResolutions(const libcamera::PixelFormat &pixelFormat);
	int initializeStreamConfigurations();

	int initializeStaticMetadata();

	std::shared_ptr<libcamera::Camera> camera_;

	int facing_;
	int orientation_;

	std::vector<Camera3StreamConfiguration> streamConfigurations_;
	std::map<int, libcamera::PixelFormat> formatsMap_;
	std::unique_ptr<CameraMetadata> staticMetadata_;
	unsigned int maxJpegBufferSize_;
};

#endif /* __ANDROID_CAMERA_CAPABILITIES_H__ */
