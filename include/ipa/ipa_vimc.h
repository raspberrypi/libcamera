/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_vimc.h - Vimc Image Processing Algorithm module
 */

#ifndef __LIBCAMERA_IPA_VIMC_H__
#define __LIBCAMERA_IPA_VIMC_H__

namespace libcamera {

#define VIMC_IPA_FIFO_PATH "/tmp/libcamera_ipa_vimc_fifo"

enum IPAOperationCode {
	IPAOperationNone,
	IPAOperationInit,
	IPAOperationStart,
	IPAOperationStop,
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_VIMC_H__ */
