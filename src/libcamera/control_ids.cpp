/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_ids.cpp : Control ID list
 */

#include <libcamera/control_ids.h>

/**
 * \file control_ids.h
 * \brief Camera control identifiers
 */

namespace libcamera {

namespace controls {

/**
 * \brief Enables or disables the AWB.
 * \sa ManualGain
 */
extern const Control<bool> AwbEnable(AWB_ENABLE, "AwbEnable");

/**
 * \brief Specify a fixed brightness parameter.
 */
extern const Control<int32_t> Brightness(BRIGHTNESS, "Brightness");

/**
 * \brief Specify a fixed contrast parameter.
 */
extern const Control<int32_t> Contrast(CONTRAST, "Contrast");

/**
 * \brief Specify a fixed saturation parameter.
 */
extern const Control<int32_t> Saturation(SATURATION, "Saturation");

/**
 * \brief Specify a fixed exposure time in milli-seconds
 */
extern const Control<int32_t> ManualExposure(MANUAL_EXPOSURE, "ManualExposure");

/**
 * \brief Specify a fixed gain parameter
 */
extern const Control<int32_t> ManualGain(MANUAL_GAIN, "ManualGain");

} /* namespace controls */

} /* namespace libcamera */
