/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * awb.h - IPU3 AWB control algorithm
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_AWB_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_AWB_H__

#include <vector>

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

/* Region size for the statistics generation algorithm */
static constexpr uint32_t kAwbStatsSizeX = 16;
static constexpr uint32_t kAwbStatsSizeY = 12;

/* \todo Move the cell layout into intel-ipu3.h kernel header */
struct Ipu3AwbCell {
	unsigned char greenRedAvg;
	unsigned char redAvg;
	unsigned char blueAvg;
	unsigned char greenBlueAvg;
	unsigned char satRatio;
	unsigned char padding[3];
} __attribute__((packed));

struct Accumulator {
	unsigned int counted;
	unsigned long long rSum;
	unsigned long long gSum;
	unsigned long long bSum;
};

class Awb : public Algorithm
{
public:
	Awb();
	~Awb();

	void prepare(IPAContext &context, ipu3_uapi_params *params) override;
	void process(IPAContext &context, const ipu3_uapi_stats_3a *stats) override;

	/* \todo Make these structs available to all the ISPs ? */
	struct RGB {
		RGB(double _R = 0, double _G = 0, double _B = 0)
			: R(_R), G(_G), B(_B)
		{
		}
		double R, G, B;
		RGB &operator+=(RGB const &other)
		{
			R += other.R, G += other.G, B += other.B;
			return *this;
		}
	};

	struct AwbStatus {
		double temperatureK;
		double redGain;
		double greenGain;
		double blueGain;
	};

private:
	void calculateWBGains(const ipu3_uapi_stats_3a *stats,
			      const ipu3_uapi_grid_config &grid);
	void generateZones(std::vector<RGB> &zones);
	void generateAwbStats(const ipu3_uapi_stats_3a *stats,
			      const ipu3_uapi_grid_config &grid);
	void clearAwbStats();
	void awbGreyWorld();
	uint32_t estimateCCT(double red, double green, double blue);

	std::vector<RGB> zones_;
	Accumulator awbStats_[kAwbStatsSizeX * kAwbStatsSizeY];
	AwbStatus asyncResults_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera*/
#endif /* __LIBCAMERA_IPU3_ALGORITHMS_AWB_H__ */
