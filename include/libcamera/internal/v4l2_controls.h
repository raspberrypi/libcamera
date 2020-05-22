/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_controls.h - V4L2 Controls Support
 */

#ifndef __LIBCAMERA_INTERNAL_V4L2_CONTROLS_H__
#define __LIBCAMERA_INTERNAL_V4L2_CONTROLS_H__

#include <linux/videodev2.h>

#include <libcamera/controls.h>

namespace libcamera {

class V4L2ControlId : public ControlId
{
public:
	V4L2ControlId(const struct v4l2_query_ext_ctrl &ctrl);
};

class V4L2ControlInfo : public ControlInfo
{
public:
	V4L2ControlInfo(const struct v4l2_query_ext_ctrl &ctrl);
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_V4L2_CONTROLS_H__ */
