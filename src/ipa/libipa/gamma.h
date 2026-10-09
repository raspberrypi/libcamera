/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy
 *
 * libIPA Gamma correction algorithm
 */

#pragma once

#include <cmath>
#include <span>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/value_node.h"

#include "fixedpoint.h"

namespace libcamera {

namespace ipa {

LOG_DECLARE_CATEGORY(Gamma)

namespace gamma {

struct ActiveState {
	float gamma;
};

struct FrameContext {
	float gamma;
	bool update;
};

} /* namespace gamma */

class GammaAlgorithmBase
{
public:
	GammaAlgorithmBase(unsigned int nLutNodes)
		: nLutNodes_(nLutNodes), kneePoints_(nLutNodes)
	{
	}

	int init(ControlInfoMap::Map &controls, const ValueNode &tuningData,
		 std::span<unsigned int> segments = {});

	void configure(gamma::ActiveState &state);
	void queueRequest(gamma::ActiveState &state, const uint32_t frame,
			  gamma::FrameContext &context, const ControlList &controls);
	void process(gamma::FrameContext &context, ControlList &metadata);

protected:
	unsigned int nLutNodes_;
	float defaultGamma_;
	std::vector<float> kneePoints_;
};

template<unsigned int NLutNodes, typename UQ>
class GammaAlgorithm : public GammaAlgorithmBase
{
public:
	GammaAlgorithm()
		: GammaAlgorithmBase(NLutNodes)
	{
	}

	template<typename T>
	void prepare(gamma::FrameContext &context, std::span<T, NLutNodes> lut)
	{
		for (unsigned int i = 0; i < nLutNodes_; i++) {
			float gamma = std::pow(kneePoints_[i], 1.0f / context.gamma);
			lut[i] = UQ(gamma).quantized();

			LOG(Gamma, Debug) << "LUT[" << i << "]=" << gamma
					  << "(" << lut[i] << ")";
		}
	}
};

} /* namespace ipa */

} /* namespace libcamera */
