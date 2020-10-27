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

#ifndef __DOXYGEN__

namespace libcamera {

namespace RPi {

enum ConfigParameters {
	IPA_CONFIG_LS_TABLE = (1 << 0),
	IPA_CONFIG_STAGGERED_WRITE = (1 << 1),
	IPA_CONFIG_SENSOR = (1 << 2),
	IPA_CONFIG_DROP_FRAMES = (1 << 3),
};

enum Operations {
	IPA_ACTION_V4L2_SET_STAGGERED = 1,
	IPA_ACTION_V4L2_SET_ISP,
	IPA_ACTION_STATS_METADATA_COMPLETE,
	IPA_ACTION_RUN_ISP,
	IPA_ACTION_EMBEDDED_COMPLETE,
	IPA_EVENT_SIGNAL_STAT_READY,
	IPA_EVENT_SIGNAL_ISP_PREPARE,
	IPA_EVENT_QUEUE_REQUEST,
};

enum BufferMask {
	ID		= 0x00ffff,
	STATS		= 0x010000,
	EMBEDDED_DATA	= 0x020000,
	BAYER_DATA	= 0x040000,
	EXTERNAL_BUFFER	= 0x100000,
};

/* Size of the LS grid allocation. */
static constexpr unsigned int MaxLsGridSize = 32 << 10;

/* List of controls handled by the Raspberry Pi IPA */
static const ControlInfoMap Controls = {
	{ &controls::AeEnable, ControlInfo(false, true) },
	{ &controls::ExposureTime, ControlInfo(0, 999999) },
	{ &controls::AnalogueGain, ControlInfo(1.0f, 32.0f) },
	{ &controls::AeMeteringMode, ControlInfo(controls::AeMeteringModeValues) },
	{ &controls::AeConstraintMode, ControlInfo(controls::AeConstraintModeValues) },
	{ &controls::AeExposureMode, ControlInfo(controls::AeExposureModeValues) },
	{ &controls::ExposureValue, ControlInfo(0.0f, 16.0f) },
	{ &controls::AwbEnable, ControlInfo(false, true) },
	{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
	{ &controls::AwbMode, ControlInfo(controls::AwbModeValues) },
	{ &controls::Brightness, ControlInfo(-1.0f, 1.0f) },
	{ &controls::Contrast, ControlInfo(0.0f, 32.0f) },
	{ &controls::Saturation, ControlInfo(0.0f, 32.0f) },
	{ &controls::Sharpness, ControlInfo(0.0f, 16.0f, 1.0f) },
	{ &controls::ColourCorrectionMatrix, ControlInfo(-16.0f, 16.0f) },
	{ &controls::ScalerCrop, ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535), Rectangle{}) },
};

} /* namespace RPi */

} /* namespace libcamera */

#endif /* __DOXYGEN__ */

#endif /* __LIBCAMERA_IPA_INTERFACE_RASPBERRYPI_H__ */
