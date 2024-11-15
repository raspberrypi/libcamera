/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * awb.h - Mali C55 grey world auto white balance algorithm
 */

#include "algorithm.h"
#include "ipa_context.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

class Awb : public Algorithm
{
public:
	Awb();
	~Awb() = default;

	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     mali_c55_params_buffer *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mali_c55_stats_buffer *stats,
		     ControlList &metadata) override;

private:
	size_t fillGainsParamBlock(mali_c55_params_block block,
				   IPAContext &context,
				   IPAFrameContext &frameContext);
	size_t fillConfigParamBlock(mali_c55_params_block block);
};

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
