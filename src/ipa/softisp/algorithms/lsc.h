/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Lens shading correction
 */

#pragma once

#include <libipa/interpolator.h>

#include "libipa/fixedpoint.h"
#include "libipa/lsc.h"

#include "algorithm.h"
#include "ipa_context.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Lsc : public Algorithm
{
public:
	Lsc() = default;
	~Lsc() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
			  IPAFrameContext &frameContext, const ControlList &controls) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process([[maybe_unused]] IPAContext &context,
		     [[maybe_unused]] const uint32_t frame,
		     IPAFrameContext &frameContext,
		     [[maybe_unused]] const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	LscAlgorithm<UQ<2, 6>> lscAlgo_;

	std::vector<double> gridPos_;

	unsigned int lastAppliedCt_ = 0;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
