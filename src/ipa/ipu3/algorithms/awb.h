/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * IPU3 AWB control algorithm
 */

#pragma once

#include <vector>

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/vector.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

/* Region size for the statistics generation algorithm */
static constexpr uint32_t kAwbStatsSizeX = 16;
static constexpr uint32_t kAwbStatsSizeY = 12;

struct Accumulator {
	unsigned int counted;
	struct {
		uint64_t red;
		uint64_t green;
		uint64_t blue;
	} sum;
};

class Awb : public Algorithm
{
public:
	Awb();
	~Awb();

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     ipu3_uapi_params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	struct AwbStatus {
		double temperatureK;
		double redGain;
		double greenGain;
		double blueGain;
	};

private:
	void calculateWBGains(const ipu3_uapi_stats_3a *stats);
	void generateZones();
	void generateAwbStats(const ipu3_uapi_stats_3a *stats);
	void clearAwbStats();
	void awbGreyWorld();
	static constexpr uint16_t threshold(float value);
	static constexpr uint16_t gainValue(double gain);

	std::vector<RGB<double>> zones_;
	Accumulator awbStats_[kAwbStatsSizeX * kAwbStatsSizeY];
	AwbStatus asyncResults_;

	uint32_t stride_;
	uint32_t cellsPerZoneX_;
	uint32_t cellsPerZoneY_;
	uint32_t cellsPerZoneThreshold_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera*/
