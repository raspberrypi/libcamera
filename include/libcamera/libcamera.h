/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * libcamera.h - libcamera public API
 */
#ifndef __LIBCAMERA_LIBCAMERA_H__
#define __LIBCAMERA_LIBCAMERA_H__

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

namespace libcamera {

class libcamera
{
public:
	void init_lib(void);
};

};

#endif /* __LIBCAMERA_LIBCAMERA_H__ */
