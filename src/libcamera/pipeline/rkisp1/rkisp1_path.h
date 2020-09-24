/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * rkisp1path.h - Rockchip ISP1 path helper
 */
#ifndef __LIBCAMERA_PIPELINE_RKISP1_PATH_H__
#define __LIBCAMERA_PIPELINE_RKISP1_PATH_H__

#include <vector>

#include <libcamera/camera.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/span.h>

namespace libcamera {

class MediaDevice;
class V4L2Subdevice;
class V4L2VideoDevice;
struct StreamConfiguration;
struct V4L2SubdeviceFormat;

class RkISP1Path
{
public:
	RkISP1Path(const char *name, const Span<const PixelFormat> &formats,
		   const Size &minResolution, const Size &maxResolution);
	~RkISP1Path();

	bool init(MediaDevice *media);

	StreamConfiguration generateConfiguration(const Size &resolution);
	CameraConfiguration::Status validate(StreamConfiguration *cfg);

	int configure(const StreamConfiguration &config,
		      const V4L2SubdeviceFormat &inputFormat);

	/* \todo Make video private. */
	V4L2VideoDevice *video_;

private:
	static constexpr unsigned int RKISP1_BUFFER_COUNT = 4;

	const char *name_;

	const Span<const PixelFormat> formats_;
	const Size minResolution_;
	const Size maxResolution_;

	V4L2Subdevice *resizer_;
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

#endif /* __LIBCAMERA_PIPELINE_RKISP1_PATH_H__ */
