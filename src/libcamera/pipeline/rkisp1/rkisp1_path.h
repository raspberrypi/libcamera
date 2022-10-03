/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * rkisp1path.h - Rockchip ISP1 path helper
 */

#pragma once

#include <memory>
#include <set>
#include <vector>

#include <libcamera/base/signal.h>
#include <libcamera/base/span.h>

#include <libcamera/camera.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/media_object.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CameraSensor;
class MediaDevice;
class V4L2Subdevice;
struct StreamConfiguration;
struct V4L2SubdeviceFormat;

class RkISP1Path
{
public:
	RkISP1Path(const char *name, const Span<const PixelFormat> &formats,
		   const Size &minResolution, const Size &maxResolution);

	bool init(MediaDevice *media);

	int setEnabled(bool enable) { return link_->setEnabled(enable); }
	bool isEnabled() const { return link_->flags() & MEDIA_LNK_FL_ENABLED; }

	StreamConfiguration generateConfiguration(const CameraSensor *sensor,
						  StreamRole role);
	CameraConfiguration::Status validate(const CameraSensor *sensor,
					     StreamConfiguration *cfg);

	int configure(const StreamConfiguration &config,
		      const V4L2SubdeviceFormat &inputFormat);

	int exportBuffers(unsigned int bufferCount,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers)
	{
		return video_->exportBuffers(bufferCount, buffers);
	}

	int start();
	void stop();

	int queueBuffer(FrameBuffer *buffer) { return video_->queueBuffer(buffer); }
	Signal<FrameBuffer *> &bufferReady() { return video_->bufferReady; }

private:
	void populateFormats();

	static constexpr unsigned int RKISP1_BUFFER_COUNT = 4;

	const char *name_;
	bool running_;

	const Span<const PixelFormat> formats_;
	std::set<PixelFormat> streamFormats_;
	Size minResolution_;
	Size maxResolution_;

	std::unique_ptr<V4L2Subdevice> resizer_;
	std::unique_ptr<V4L2VideoDevice> video_;
	MediaLink *link_;
};

class RkISP1MainPath : public RkISP1Path
{
public:
	RkISP1MainPath();
};

class RkISP1SelfPath : public RkISP1Path
{
public:
	RkISP1SelfPath();
};

} /* namespace libcamera */
