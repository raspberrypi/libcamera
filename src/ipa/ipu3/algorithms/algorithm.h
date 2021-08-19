/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - IPU3 control algorithm interface
 */
#ifndef __LIBCAMERA_IPA_IPU3_ALGORITHM_H__
#define __LIBCAMERA_IPA_IPU3_ALGORITHM_H__

#include <libcamera/ipa/ipu3_ipa_interface.h>

#include "ipa_context.h"

namespace libcamera {

namespace ipa::ipu3 {

class Algorithm
{
public:
	virtual ~Algorithm() {}

	virtual int configure(IPAContext &context, const IPAConfigInfo &configInfo);
	virtual void prepare(IPAContext &context, ipu3_uapi_params *params);
	virtual void process(IPAContext &context, const ipu3_uapi_stats_3a *stats);
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_IPU3_ALGORITHM_H__ */
