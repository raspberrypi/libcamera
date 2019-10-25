/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * pixelformats.cpp - libcamera pixel formats
 */

#include <libcamera/pixelformats.h>

/**
 * \file pixelformats.h
 * \brief libcamera pixel formats
 */

namespace libcamera {

/**
 * \typedef PixelFormat
 * \brief libcamera image pixel format
 *
 * The PixelFormat type describes the format of images in the public libcamera
 * API. It stores a FourCC value as a 32-bit unsigned integer. The values are
 * defined in the Linux kernel DRM/KMS API (see linux/drm_fourcc.h).
 *
 * \todo Add support for format modifiers
 */

} /* namespace libcamera */
