/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * rkisp1.h - Image Processing Algorithm interface for RkISP1
 */
#ifndef __LIBCAMERA_IPA_INTERFACE_RKISP1_H__
#define __LIBCAMERA_IPA_INTERFACE_RKISP1_H__

enum RkISP1Operations {
	RKISP1_IPA_ACTION_V4L2_SET = 1,
	RKISP1_IPA_ACTION_PARAM_FILLED = 2,
	RKISP1_IPA_ACTION_METADATA = 3,
	RKISP1_IPA_EVENT_SIGNAL_STAT_BUFFER = 4,
	RKISP1_IPA_EVENT_QUEUE_REQUEST = 5,
};

#endif /* __LIBCAMERA_IPA_INTERFACE_RKISP1_H__ */
