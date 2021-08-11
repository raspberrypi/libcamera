/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.h - IPU3 AGC/AEC control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AGC_H__
#define __LIBCAMERA_IPU3_AGC_H__

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithms/algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3 {

using utils::Duration;

class IPU3Agc : public Algorithm
{
public:
	IPU3Agc();
	~IPU3Agc() = default;

	void initialise(struct ipu3_uapi_grid_config &bdsGrid, const IPACameraSensorInfo &sensorInfo);
	void process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, double &gain);
	bool converged() { return converged_; }
	bool updateControls() { return updateControls_; }
	/* \todo Use a metadata exchange between IPAs */
	double gamma() { return gamma_; }

private:
	void processBrightness(const ipu3_uapi_stats_3a *stats);
	void filterExposure();
	void lockExposureGain(uint32_t &exposure, double &gain);

	struct ipu3_uapi_grid_config aeGrid_;

	uint64_t frameCount_;
	uint64_t lastFrame_;

	bool converged_;
	bool updateControls_;

	double iqMean_;
	double gamma_;

	Duration lineDuration_;
	Duration maxExposureTime_;

	Duration prevExposure_;
	Duration prevExposureNoDg_;
	Duration currentExposure_;
	Duration currentExposureNoDg_;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_AGC_H__ */
