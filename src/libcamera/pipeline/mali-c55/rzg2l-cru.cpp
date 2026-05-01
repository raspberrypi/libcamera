/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * Pipine handler element for the Renesas RZ/G2L Camera Receiver Unit
 */

#include "rzg2l-cru.h"

#include <algorithm>
#include <map>

#include <linux/videodev2.h>

#include <libcamera/base/log.h>
#include <libcamera/base/regex.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/request.h"

namespace libcamera {

static const std::map<uint8_t, V4L2PixelFormat> bitDepthToFmt{
	{ 10, V4L2PixelFormat(V4L2_PIX_FMT_RAW_CRU10) },
	{ 12, V4L2PixelFormat(V4L2_PIX_FMT_RAW_CRU12) },
	{ 14, V4L2PixelFormat(V4L2_PIX_FMT_RAW_CRU14) },
};

LOG_DEFINE_CATEGORY(RZG2LCRU)

FrameBuffer *RZG2LCRU::queueBuffer(Request *request)
{
	FrameBuffer *buffer;

	if (availableBuffers_.empty()) {
		LOG(RZG2LCRU, Debug) << "CRU buffer underrun";
		return nullptr;
	}

	buffer = availableBuffers_.back();

	int ret = output_->queueBuffer(buffer);
	if (ret) {
		LOG(RZG2LCRU, Error) << "Failed to queue buffer to CRU";
		return nullptr;
	}

	availableBuffers_.pop_back();
	buffer->_d()->setRequest(request);

	return buffer;
}

void RZG2LCRU::returnBuffer(FrameBuffer *buffer)
{
	auto it = std::find_if(buffers_.begin(), buffers_.end(),
			       [&](const auto &b) { return b.get() == buffer; });
	ASSERT(it != buffers_.end());

	availableBuffers_.push_back(buffer);
}

int RZG2LCRU::start(unsigned int bufferCount)
{
	int ret = output_->exportBuffers(bufferCount, &buffers_);
	if (ret < 0)
		return ret;

	utils::scope_exit bufferGuard([&] { freeBuffers(); });

	ret = output_->importBuffers(bufferCount);
	if (ret)
		return ret;

	for (std::unique_ptr<FrameBuffer> &buffer : buffers_)
		availableBuffers_.push_back(buffer.get());

	ret = output_->streamOn();
	if (ret)
		return ret;

	bufferGuard.release();

	return 0;
}

int RZG2LCRU::stop()
{
	return output_->streamOff();
}

int RZG2LCRU::freeBuffers()
{
	availableBuffers_.clear();
	buffers_.clear();

	return output_->releaseBuffers();
}

int RZG2LCRU::configure(V4L2SubdeviceFormat *subdevFormat, V4L2DeviceFormat *inputFormat)
{
	/*
	 * Set format on the sensor and propagate it up to the CRU video
	 * device.
	 */

	int ret = sensor_->setFormat(subdevFormat);
	if (ret)
		return ret;

	ret = csi2_->setFormat(0, subdevFormat);
	if (ret)
		return ret;

	ret = csi2_->getFormat(1, subdevFormat);
	if (ret)
		return ret;

	ret = cru_->setFormat(0, subdevFormat);
	if (ret)
		return ret;

	ret = cru_->getFormat(1, subdevFormat);
	if (ret)
		return ret;

	/*
	 * The capture device needs to be set with a format that can be produced
	 * from the mbus code of the subdevFormat. The CRU and IVC use bayer
	 * order agnostic pixel formats, so all we need to do is find the right
	 * bitdepth and select the appropriate format.
	 */
	BayerFormat bayerFormat = BayerFormat::fromMbusCode(subdevFormat->code);
	if (!bayerFormat.isValid())
		return -EINVAL;

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = bitDepthToFmt.at(bayerFormat.bitDepth);
	captureFormat.size = subdevFormat->size;

	ret = output_->setFormat(&captureFormat);
	if (ret)
		return ret;

	/*
	 * We return the format that we set against the output device, as the
	 * same format will also need to be set against the Input Video Control
	 * Block device.
	 */
	*inputFormat = captureFormat;

	return 0;
}

void RZG2LCRU::initCRUSizes()
{
	Size maxCSI2Size;

	/*
	 * Get the maximum supported size on the CSI-2 receiver. We need to
	 * query the kernel interface as the size limits differ from RZ/G2L
	 * (2800x4095) and RZ/V2H (4096x4096).
	 */
	V4L2Subdevice::Formats csi2Formats = csi2_->formats(0);
	if (csi2Formats.empty())
		return;

	for (const auto &format : csi2Formats) {
		for (const auto &range : format.second) {
			if (range.max > maxCSI2Size)
				maxCSI2Size = range.max;
		}
	}

	/*
	 * Enumerate the sensor supported resolutions and filter out the ones
	 * larger than the maximum supported CSI-2 receiver input size.
	 */
	V4L2Subdevice::Formats formats = sensor_->device()->formats(0);
	if (formats.empty())
		return;

	for (const auto &format : formats) {
		for (const auto &range : format.second) {
			const Size &max = range.max;

			if (max.width > maxCSI2Size.width ||
			    max.height > maxCSI2Size.height)
				continue;

			csi2Sizes_.push_back(max);
		}
	}

	/* Sort in increasing order and remove duplicates. */
	std::sort(csi2Sizes_.begin(), csi2Sizes_.end());
	auto last = std::unique(csi2Sizes_.begin(), csi2Sizes_.end());
	csi2Sizes_.erase(last, csi2Sizes_.end());

	csi2Resolution_ = csi2Sizes_.back();
}

int RZG2LCRU::init(const MediaDevice *media)
{
	static const std::regex cruRegex("cru-ip-[0-9a-f]{8}.cru[0-9]");
	static const std::regex csi2Regex("csi-[0-9a-f]{8}.csi2");

	csi2_ = V4L2Subdevice::fromEntityName(media, csi2Regex);
	if (!csi2_)
		return -ENODEV;

	int ret = csi2_->open();
	if (ret)
		return ret;

	const std::vector<MediaPad *> &pads = csi2_->entity()->pads();
	if (pads.empty())
		return -ENODEV;

	/* The receiver has a single sink pad at index 0 */
	MediaPad *sink = pads[0];
	const std::vector<MediaLink *> &links = sink->links();
	if (links.empty())
		return -ENODEV;

	MediaLink *link = links[0];
	sensor_ = CameraSensorFactoryBase::create(link->source()->entity());
	if (!sensor_)
		return -ENODEV;

	cru_ = V4L2Subdevice::fromEntityName(media, cruRegex);
	ret = cru_->open();
	if (ret)
		return ret;

	output_ = V4L2VideoDevice::fromEntityName(media, "CRU output");
	ret = output_->open();
	if (ret)
		return ret;

	initCRUSizes();

	return 0;
}

} /* namespace libcamera */
