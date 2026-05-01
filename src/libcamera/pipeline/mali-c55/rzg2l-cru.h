/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * Pipine handler element for the Renesas RZ/G2L Camera Receiver Unit
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/base/signal.h>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CameraSensor;
class FrameBuffer;
class MediaDevice;
class Request;
class Size;

class RZG2LCRU
{
public:
	RZG2LCRU() = default;

	const std::vector<Size> &sizes() const
	{
		return csi2Sizes_;
	}

	int init(const MediaDevice *media);
	const Size &resolution() const
	{
		return csi2Resolution_;
	}

	CameraSensor *sensor() const { return sensor_.get(); }
	V4L2Subdevice *csi2() const { return csi2_.get(); }
	V4L2Subdevice *cru() const { return cru_.get(); }
	V4L2VideoDevice *output() { return output_.get(); }

	int configure(V4L2SubdeviceFormat *subdevFormat, V4L2DeviceFormat *inputFormat);
	FrameBuffer *queueBuffer(Request *request);
	void returnBuffer(FrameBuffer *buffer);
	int freeBuffers();

	int start(unsigned int bufferCount);
	int stop();

private:
	void initCRUSizes();

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<V4L2Subdevice> csi2_;
	std::unique_ptr<V4L2Subdevice> cru_;
	std::unique_ptr<V4L2VideoDevice> output_;

	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
	std::vector<FrameBuffer *> availableBuffers_;

	std::vector<Size> csi2Sizes_;
	Size csi2Resolution_;
};

} /* namespace libcamera */
