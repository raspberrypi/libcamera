/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Private Header Validation
 *
 * A selection of internal libcamera headers are installed as part
 * of the libcamera package to allow sharing of a select subset of
 * internal functionality with IPA module only.
 *
 * This functionality is not considered part of the public libcamera
 * API, and can therefore potentially face ABI instabilities which
 * should not be exposed to applications. IPA modules however should be
 * versioned and more closely matched to the libcamera installation.
 *
 * Components which include this file can not be included in any file
 * which forms part of the libcamera API.
 */

#ifndef LIBCAMERA_BASE_PRIVATE
#error "Private headers must not be included in the libcamera API"
#endif
