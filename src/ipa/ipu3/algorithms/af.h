/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * af.h - IPU3 Af algorithm
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class Af : public Algorithm
{
	/* The format of y_table. From ipu3-ipa repo */
	typedef struct __attribute__((packed)) y_table_item {
		uint16_t y1_avg;
		uint16_t y2_avg;
	} y_table_item_t;
public:
	Af();
	~Af() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     ipu3_uapi_params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	void afCoarseScan(IPAContext &context);
	void afFineScan(IPAContext &context);
	bool afScan(IPAContext &context, int min_step);
	void afReset(IPAContext &context);
	bool afNeedIgnoreFrame();
	void afIgnoreFrameReset();
	double afEstimateVariance(Span<const y_table_item_t> y_items, bool isY1);

	bool afIsOutOfFocus(IPAContext &context);

	/* VCM step configuration. It is the current setting of the VCM step. */
	uint32_t focus_;
	/* The best VCM step. It is a local optimum VCM step during scanning. */
	uint32_t bestFocus_;
	/* Current AF statistic variance. */
	double currentVariance_;
	/* The frames are ignore before starting measuring. */
	uint32_t ignoreCounter_;
	/* It is used to determine the derivative during scanning */
	double previousVariance_;
	/* The designated maximum range of focus scanning. */
	uint32_t maxStep_;
	/* If the coarse scan completes, it is set to true. */
	bool coarseCompleted_;
	/* If the fine scan completes, it is set to true. */
	bool fineCompleted_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
