/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * Mali-C55 Lens shading correction algorithm
 */

#include <vector>
#include <tuple>

#include <linux/media/arm/mali-c55-config.h>

#include "libcamera/internal/value_node.h"

#include "libipa/fixedpoint.h"
#include "libipa/lsc.h"

#include "algorithm.h"
#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

class Lsc : public Algorithm
{
public:
	Lsc() = default;
	~Lsc() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     MaliC55Params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mali_c55_stats_buffer *stats,
		     ControlList &metadata) override;
private:
	std::vector<double> segmentsToPosition() const;
	void fillConfigParamsBlock(MaliC55Params *params) const;
	void fillSelectionParamsBlock(MaliC55Params *params,
				      uint8_t bank, uint8_t alpha) const;
	std::tuple<uint8_t, uint8_t> findBankAndAlpha(uint32_t ct) const;

	std::vector<uint32_t> colourTemperatures_;
	std::vector<uint32_t> mesh_;

	std::vector<double> gridPos_;

	LscAlgorithm<UQ<2, 6>> lscAlgo_;
};

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
