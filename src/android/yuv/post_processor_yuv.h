/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * post_processor_yuv.h - Post Processor using libyuv
 */
#ifndef __ANDROID_POST_PROCESSOR_YUV_H__
#define __ANDROID_POST_PROCESSOR_YUV_H__

#include "../post_processor.h"

#include <libcamera/geometry.h>

class PostProcessorYuv : public PostProcessor
{
public:
	PostProcessorYuv() = default;

	int configure(const libcamera::StreamConfiguration &incfg,
		      const libcamera::StreamConfiguration &outcfg) override;
	int process(const libcamera::FrameBuffer &source,
		    CameraBuffer *destination,
		    const CameraMetadata &requestMetadata,
		    CameraMetadata *metadata) override;

private:
	bool isValidBuffers(const libcamera::FrameBuffer &source,
			    const CameraBuffer &destination) const;
	void calculateLengths(const libcamera::StreamConfiguration &inCfg,
			      const libcamera::StreamConfiguration &outCfg);

	libcamera::Size sourceSize_;
	libcamera::Size destinationSize_;
	unsigned int sourceLength_[2] = {};
	unsigned int destinationLength_[2] = {};
	unsigned int sourceStride_[2] = {};
	unsigned int destinationStride_[2] = {};
};

#endif /* __ANDROID_POST_PROCESSOR_YUV_H__ */
