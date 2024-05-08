/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Post Processor using libyuv
 */

#include "post_processor_yuv.h"

#include <libyuv/scale.h>

#include <libcamera/base/log.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(YUV)

int PostProcessorYuv::configure(const StreamConfiguration &inCfg,
				const StreamConfiguration &outCfg)
{
	if (inCfg.pixelFormat != outCfg.pixelFormat) {
		LOG(YUV, Error) << "Pixel format conversion is not supported"
				<< " (from " << inCfg.pixelFormat
				<< " to " << outCfg.pixelFormat << ")";
		return -EINVAL;
	}

	if (inCfg.size < outCfg.size) {
		LOG(YUV, Error) << "Up-scaling is not supported"
				<< " (from " << inCfg.size
				<< " to " << outCfg.size << ")";
		return -EINVAL;
	}

	if (inCfg.pixelFormat != formats::NV12) {
		LOG(YUV, Error) << "Unsupported format " << inCfg.pixelFormat
				<< " (only NV12 is supported)";
		return -EINVAL;
	}

	calculateLengths(inCfg, outCfg);
	return 0;
}

void PostProcessorYuv::process(Camera3RequestDescriptor::StreamBuffer *streamBuffer)
{
	const FrameBuffer &source = *streamBuffer->srcBuffer;
	CameraBuffer *destination = streamBuffer->dstBuffer.get();

	if (!isValidBuffers(source, *destination)) {
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	const MappedFrameBuffer sourceMapped(&source, MappedFrameBuffer::MapFlag::Read);
	if (!sourceMapped.isValid()) {
		LOG(YUV, Error) << "Failed to mmap camera frame buffer";
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	int ret = libyuv::NV12Scale(sourceMapped.planes()[0].data(),
				    sourceStride_[0],
				    sourceMapped.planes()[1].data(),
				    sourceStride_[1],
				    sourceSize_.width, sourceSize_.height,
				    destination->plane(0).data(),
				    destinationStride_[0],
				    destination->plane(1).data(),
				    destinationStride_[1],
				    destinationSize_.width,
				    destinationSize_.height,
				    libyuv::FilterMode::kFilterBilinear);
	if (ret) {
		LOG(YUV, Error) << "Failed NV12 scaling: " << ret;
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	processComplete.emit(streamBuffer, PostProcessor::Status::Success);
}

bool PostProcessorYuv::isValidBuffers(const FrameBuffer &source,
				      const CameraBuffer &destination) const
{
	if (source.planes().size() != 2) {
		LOG(YUV, Error) << "Invalid number of source planes: "
				<< source.planes().size();
		return false;
	}
	if (destination.numPlanes() != 2) {
		LOG(YUV, Error) << "Invalid number of destination planes: "
				<< destination.numPlanes();
		return false;
	}

	if (source.planes()[0].length < sourceLength_[0] ||
	    source.planes()[1].length < sourceLength_[1]) {
		LOG(YUV, Error)
			<< "The source planes lengths are too small, actual size: {"
			<< source.planes()[0].length << ", "
			<< source.planes()[1].length
			<< "}, expected size: {"
			<< sourceLength_[0] << ", "
			<< sourceLength_[1] << "}";
		return false;
	}
	if (destination.plane(0).size() < destinationLength_[0] ||
	    destination.plane(1).size() < destinationLength_[1]) {
		LOG(YUV, Error)
			<< "The destination planes lengths are too small, actual size: {"
			<< destination.plane(0).size() << ", "
			<< destination.plane(1).size()
			<< "}, expected size: {"
			<< sourceLength_[0] << ", "
			<< sourceLength_[1] << "}";
		return false;
	}

	return true;
}

void PostProcessorYuv::calculateLengths(const StreamConfiguration &inCfg,
					const StreamConfiguration &outCfg)
{
	sourceSize_ = inCfg.size;
	destinationSize_ = outCfg.size;

	const PixelFormatInfo &nv12Info = PixelFormatInfo::info(formats::NV12);
	for (unsigned int i = 0; i < 2; i++) {
		sourceStride_[i] = inCfg.stride;
		destinationStride_[i] = nv12Info.stride(destinationSize_.width, i, 1);

		sourceLength_[i] = nv12Info.planeSize(sourceSize_.height, i,
						      sourceStride_[i]);
		destinationLength_[i] = nv12Info.planeSize(destinationSize_.height, i,
							   destinationStride_[i]);
	}
}
