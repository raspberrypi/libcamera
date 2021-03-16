/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_awb.h - IPU3 AWB control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AWB_H__
#define __LIBCAMERA_IPU3_AWB_H__

#include <vector>

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include "libipa/algorithm.h"

namespace libcamera {

namespace ipa::ipu3 {

/* Region size for the statistics generation algorithm */
static constexpr uint32_t kAwbStatsSizeX = 16;
static constexpr uint32_t kAwbStatsSizeY = 12;

class IPU3Awb : public Algorithm
{
public:
	IPU3Awb();
	~IPU3Awb();

	void initialise(ipu3_uapi_params &params, const Size &bdsOutputSize, struct ipu3_uapi_grid_config &bdsGrid);
	void calculateWBGains(const ipu3_uapi_stats_3a *stats);
	void updateWbParameters(ipu3_uapi_params &params, double agcGamma);

	struct Ipu3AwbCell {
		unsigned char greenRedAvg;
		unsigned char redAvg;
		unsigned char blueAvg;
		unsigned char greenBlueAvg;
		unsigned char satRatio;
		unsigned char padding[3];
	} __attribute__((packed));

	/* \todo Make these three structs available to all the ISPs ? */
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

	struct IspStatsRegion {
		unsigned int counted;
		unsigned int uncounted;
		unsigned long long rSum;
		unsigned long long gSum;
		unsigned long long bSum;
	};

	struct AwbStatus {
		double temperatureK;
		double redGain;
		double greenGain;
		double blueGain;
	};

private:
	void generateZones(std::vector<RGB> &zones);
	void generateAwbStats(const ipu3_uapi_stats_3a *stats);
	void clearAwbStats();
	void awbGreyWorld();
	uint32_t estimateCCT(double red, double green, double blue);

	struct ipu3_uapi_grid_config awbGrid_;

	std::vector<RGB> zones_;
	IspStatsRegion awbStats_[kAwbStatsSizeX * kAwbStatsSizeY];
	AwbStatus asyncResults_;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera*/
#endif /* __LIBCAMERA_IPU3_AWB_H__ */
