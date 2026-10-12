/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas on Board, Oy
 *
 * IPU3 Lens Shading Correction algorithm
 */

#pragma once

#include <vector>

#include "libipa/fixedpoint.h"
#include "libipa/lsc.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class Lsc : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     ipu3_uapi_params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	std::vector<double> calculatePositions(unsigned int dimension);

	unsigned int numHCells_;
	unsigned int numVCells_;
	unsigned int blockWidthLog2_;
	unsigned int blockHeightLog2_;

	unsigned int lastAppliedCt_;
	unsigned int lastAppliedQuantizedCt_;

	unsigned int sensorWidth_;
	unsigned int sensorHeight_;
	unsigned int cropWidth_;
	unsigned int cropHeight_;

	bool polynomial_;

	LscAlgorithm<UQ<2, 10>> lscAlgo_;
};

} // namespace ipa::ipu3::algorithms

} // namespace libcamera
