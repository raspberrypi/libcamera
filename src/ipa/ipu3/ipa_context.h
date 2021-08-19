/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * ipu3_ipa_context.h - IPU3 IPA Context
 *
 */
#ifndef __LIBCAMERA_IPU3_IPA_CONTEXT_H__
#define __LIBCAMERA_IPU3_IPA_CONTEXT_H__

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

namespace libcamera {

namespace ipa::ipu3 {

struct IPASessionConfiguration {
	struct {
		ipu3_uapi_grid_config bdsGrid;
		Size bdsOutputSize;
	} grid;
};

struct IPAFrameContext {
	struct {
		struct ipu3_uapi_gamma_corr_lut gammaCorrection;
	} toneMapping;
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAFrameContext frameContext;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera*/

#endif /* __LIBCAMERA_IPU3_IPA_CONTEXT_H__ */
