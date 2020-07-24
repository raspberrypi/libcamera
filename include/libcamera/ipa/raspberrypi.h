/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2020, Raspberry Pi Ltd.
 *
 * raspberrypi.h - Image Processing Algorithm interface for Raspberry Pi
 */
#ifndef __LIBCAMERA_IPA_INTERFACE_RASPBERRYPI_H__
#define __LIBCAMERA_IPA_INTERFACE_RASPBERRYPI_H__

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

enum RPiConfigParameters {
	RPI_IPA_CONFIG_LS_TABLE = (1 << 0),
	RPI_IPA_CONFIG_STAGGERED_WRITE = (1 << 1),
	RPI_IPA_CONFIG_SENSOR = (1 << 2),
};

enum RPiOperations {
	RPI_IPA_ACTION_V4L2_SET_STAGGERED = 1,
	RPI_IPA_ACTION_V4L2_SET_ISP,
	RPI_IPA_ACTION_STATS_METADATA_COMPLETE,
	RPI_IPA_ACTION_RUN_ISP,
	RPI_IPA_ACTION_RUN_ISP_AND_DROP_FRAME,
	RPI_IPA_ACTION_EMBEDDED_COMPLETE,
	RPI_IPA_EVENT_SIGNAL_STAT_READY,
	RPI_IPA_EVENT_SIGNAL_ISP_PREPARE,
	RPI_IPA_EVENT_QUEUE_REQUEST,
};

enum RPiIpaMask {
	ID		= 0x0ffff,
	STATS		= 0x10000,
	EMBEDDED_DATA	= 0x20000,
	BAYER_DATA	= 0x40000
};

/* Size of the LS grid allocation. */
#define MAX_LS_GRID_SIZE (32 << 10)

namespace libcamera {

/* List of controls handled by the Raspberry Pi IPA */
static const ControlInfoMap RPiControls = {
	{ &controls::AeEnable, ControlInfo(false, true) },
	{ &controls::ExposureTime, ControlInfo(0, 999999) },
	{ &controls::AnalogueGain, ControlInfo(1.0f, 32.0f) },
	{ &controls::AeMeteringMode, ControlInfo(0, static_cast<int32_t>(controls::MeteringModeMax)) },
	{ &controls::AeConstraintMode, ControlInfo(0, static_cast<int32_t>(controls::ConstraintModeMax)) },
	{ &controls::AeExposureMode, ControlInfo(0, static_cast<int32_t>(controls::ExposureModeMax)) },
	{ &controls::ExposureValue, ControlInfo(0.0f, 16.0f) },
	{ &controls::AwbEnable, ControlInfo(false, true) },
	{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
	{ &controls::AwbMode, ControlInfo(0, static_cast<int32_t>(controls::AwbModeMax)) },
	{ &controls::Brightness, ControlInfo(-1.0f, 1.0f) },
	{ &controls::Contrast, ControlInfo(0.0f, 32.0f) },
	{ &controls::Saturation, ControlInfo(0.0f, 32.0f) },
	{ &controls::Sharpness, ControlInfo(0.0f, 16.0f, 1.0f) },
	{ &controls::ColourCorrectionMatrix, ControlInfo(-16.0f, 16.0f) },
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_INTERFACE_RASPBERRYPI_H__ */
