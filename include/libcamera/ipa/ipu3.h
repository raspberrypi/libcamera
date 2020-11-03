/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipu3.h - Image Processing Algorithm interface for IPU3
 */
#ifndef __LIBCAMERA_IPA_INTERFACE_IPU3_H__
#define __LIBCAMERA_IPA_INTERFACE_IPU3_H__

#ifndef __DOXYGEN__

enum IPU3Operations {
	IPU3_IPA_ACTION_SET_SENSOR_CONTROLS = 1,
	IPU3_IPA_ACTION_PARAM_FILLED = 2,
	IPU3_IPA_ACTION_METADATA_READY = 3,
	IPU3_IPA_EVENT_PROCESS_CONTROLS = 4,
	IPU3_IPA_EVENT_STAT_READY = 5,
	IPU3_IPA_EVENT_FILL_PARAMS = 6,
};

#endif /* __DOXYGEN__ */

#endif /* __LIBCAMERA_IPA_INTERFACE_IPU3_H__ */
