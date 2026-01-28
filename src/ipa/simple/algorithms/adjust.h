/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Color correction matrix
 */

#pragma once

#include <optional>

#include "libcamera/internal/matrix.h"

#include <libipa/interpolator.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

constexpr float kDefaultGamma = 2.2f;

class Adjust : public Algorithm
{
public:
	Adjust() = default;
	~Adjust() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPAConfigInfo &configInfo) override;
	void queueRequest(typename Module::Context &context,
			  const uint32_t frame,
			  typename Module::FrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void applySaturation(Matrix<float, 3, 3> &ccm, float saturation);

	std::optional<float> lastSaturation_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
