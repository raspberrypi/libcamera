/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google inc.
 *
 * black_correction.h - IPU3 Black Level Correction control
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_BLC_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_BLC_H__

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class BlackLevelCorrection : public Algorithm
{
public:
	BlackLevelCorrection();

	void prepare(IPAContext &context, ipu3_uapi_params *params) override;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_BLC_H__ */
