/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_capabilities.h - Camera static properties manager
 */

#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>

#include <libcamera/base/class.h>

#include <libcamera/camera.h>
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

	std::unique_ptr<CameraMetadata> requestTemplateManual() const;
	std::unique_ptr<CameraMetadata> requestTemplatePreview() const;
	std::unique_ptr<CameraMetadata> requestTemplateStill() const;
	std::unique_ptr<CameraMetadata> requestTemplateVideo() const;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraCapabilities)

	struct Camera3StreamConfiguration {
		libcamera::Size resolution;
		int androidFormat;
		int64_t minFrameDurationNsec;
		int64_t maxFrameDurationNsec;
	};

	bool validateManualSensorCapability();
	bool validateManualPostProcessingCapability();
	bool validateBurstCaptureCapability();

	std::set<camera_metadata_enum_android_request_available_capabilities>
		computeCapabilities();

	void computeHwLevel(
		const std::set<camera_metadata_enum_android_request_available_capabilities> &caps);

	std::vector<libcamera::Size>
	initializeYUVResolutions(const libcamera::PixelFormat &pixelFormat,
				 const std::vector<libcamera::Size> &resolutions);
	std::vector<libcamera::Size>
	initializeRawResolutions(const libcamera::PixelFormat &pixelFormat);
	int initializeStreamConfigurations();

	int initializeStaticMetadata();

	std::shared_ptr<libcamera::Camera> camera_;

	int facing_;
	int orientation_;
	bool rawStreamAvailable_;
	int64_t maxFrameDuration_;
	camera_metadata_enum_android_info_supported_hardware_level hwLevel_;
	std::set<camera_metadata_enum_android_request_available_capabilities> capabilities_;

	std::vector<Camera3StreamConfiguration> streamConfigurations_;
	std::map<int, libcamera::PixelFormat> formatsMap_;
	std::unique_ptr<CameraMetadata> staticMetadata_;
	unsigned int maxJpegBufferSize_;

	std::set<int32_t> availableCharacteristicsKeys_;
	std::set<int32_t> availableRequestKeys_;
	std::set<int32_t> availableResultKeys_;
};
