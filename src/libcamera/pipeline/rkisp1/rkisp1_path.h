/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * rkisp1path.h - Rockchip ISP1 path helper
 */
#ifndef __LIBCAMERA_PIPELINE_RKISP1_PATH_H__
#define __LIBCAMERA_PIPELINE_RKISP1_PATH_H__

namespace libcamera {

class MediaDevice;
class V4L2Subdevice;
class V4L2VideoDevice;

class RkISP1Path
{
public:
	RkISP1Path(const char *name);
	~RkISP1Path();

	bool init(MediaDevice *media);

	/* \todo Make resizer and video private. */
	V4L2Subdevice *resizer_;
	V4L2VideoDevice *video_;

private:
	const char *name_;
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
