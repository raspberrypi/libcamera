/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.h - IPU3 AGC/AEC control algorithm
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const ipu3_uapi_stats_3a *stats) override;

private:
	void processBrightness(const ipu3_uapi_stats_3a *stats,
			       const ipu3_uapi_grid_config &grid);
	void filterExposure();
	void lockExposureGain(uint32_t &exposure, double &gain);

	uint64_t frameCount_;
	uint64_t lastFrame_;

	double iqMean_;

	utils::Duration lineDuration_;
	utils::Duration maxExposureTime_;

	utils::Duration prevExposure_;
	utils::Duration prevExposureNoDg_;
	utils::Duration currentExposure_;
	utils::Duration currentExposureNoDg_;

	uint32_t stride_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__ */
