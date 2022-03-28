/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2020, Raspberry Pi Ltd.
 *
 * raspberrypi.h - Image Processing Algorithm interface for Raspberry Pi
 */

#pragma once

#include <stdint.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#ifndef __DOXYGEN__

namespace libcamera {

namespace RPi {

/*
 * List of controls handled by the Raspberry Pi IPA
 *
 * \todo This list will need to be built dynamically from the control
 * algorithms loaded by the json file, once this is supported. At that
 * point applications should check first whether a control is supported,
 * and the pipeline handler may be reverted so that it aborts when an
 * unsupported control is encountered.
 */
static const ControlInfoMap Controls({
		{ &controls::AeEnable, ControlInfo(false, true) },
		{ &controls::ExposureTime, ControlInfo(0, 999999) },
		{ &controls::AnalogueGain, ControlInfo(1.0f, 32.0f) },
		{ &controls::AeMeteringMode, ControlInfo(controls::AeMeteringModeValues) },
		{ &controls::AeConstraintMode, ControlInfo(controls::AeConstraintModeValues) },
		{ &controls::AeExposureMode, ControlInfo(controls::AeExposureModeValues) },
		{ &controls::ExposureValue, ControlInfo(-8.0f, 8.0f, 0.0f) },
		{ &controls::AwbEnable, ControlInfo(false, true) },
		{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
		{ &controls::AwbMode, ControlInfo(controls::AwbModeValues) },
		{ &controls::Brightness, ControlInfo(-1.0f, 1.0f, 0.0f) },
		{ &controls::Contrast, ControlInfo(0.0f, 32.0f, 1.0f) },
		{ &controls::Saturation, ControlInfo(0.0f, 32.0f, 1.0f) },
		{ &controls::Sharpness, ControlInfo(0.0f, 16.0f, 1.0f) },
		{ &controls::ColourCorrectionMatrix, ControlInfo(-16.0f, 16.0f) },
		{ &controls::ScalerCrop, ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535), Rectangle{}) },
		{ &controls::FrameDurationLimits, ControlInfo(INT64_C(1000), INT64_C(1000000000)) },
		{ &controls::draft::NoiseReductionMode, ControlInfo(controls::draft::NoiseReductionModeValues) }
	}, controls::controls);

} /* namespace RPi */

} /* namespace libcamera */

#endif /* __DOXYGEN__ */
