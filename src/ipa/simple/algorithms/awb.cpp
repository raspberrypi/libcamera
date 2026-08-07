/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat Inc.
 *
 * Auto white balance
 */

#include "awb.h"

#include <numeric>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAwb)

namespace ipa::soft::algorithms {

/*
 * \todo Replace it with a proper Lux algorithm
 */
static constexpr unsigned int kDefaultLux = 500;

class SimpleAwbStats final : public AwbStats
{
public:
	SimpleAwbStats() = default;

	SimpleAwbStats(const RGB<double> &rgbMeans)
	{
		rgbMeans_ = rgbMeans;

		rg_ = rgbMeans_.r() / rgbMeans_.g();
		bg_ = rgbMeans_.b() / rgbMeans_.g();
	}

	double computeColourError(const RGB<double> &gains) const override
	{
		/*
		 * Compute the sum of the squared colour error (non-greyness) as
		 * it appears in the log likelihood equation.
		 */
		double deltaR = gains.r() * rg_ - 1.0;
		double deltaB = gains.b() * bg_ - 1.0;
		double delta2 = deltaR * deltaR + deltaB * deltaB;

		return delta2;
	}

	RGB<double> rgbMeans() const override
	{
		return rgbMeans_;
	}

	bool valid() const override
	{
		/* Minimum mean value below which AWB can't operate. */
		constexpr double minValue = 0.2;

		return rgbMeans_.r() > minValue || rgbMeans_.g() > minValue ||
		       rgbMeans_.b() > minValue;
	}

private:
	RGB<double> rgbMeans_;
	double rg_;
	double bg_;
};

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Awb::init(IPAContext &context, const ValueNode &tuningData)
{
	return awbAlgo_.init(tuningData, context.ctrlMap);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	return awbAlgo_.configure(context.activeState.awb);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Awb::queueRequest(IPAContext &context, const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	awbAlgo_.queueRequest(context.activeState.awb, frame, frameContext.awb,
			      controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	awbAlgo_.prepare(context.activeState.awb, frameContext.awb);

	params->gains = frameContext.awb.gains;
}

SimpleAwbStats Awb::calculateRgbMeans(IPAContext &context,
				      const SwIspStats *stats) const
{
	if (!stats->valid)
		return {};

	const SwIspStats::Histogram &histogram = stats->yHistogram;
	const uint8_t blackLevel = context.activeState.blc.level;

	/*
	 * Black level must be subtracted to get the correct AWB ratios, they
	 * would be off if they were computed from the whole brightness range
	 * rather than from the sensor range.
	 */
	const uint64_t nPixels = std::accumulate(
		histogram.begin(), histogram.end(), uint64_t(0));
	const uint64_t offset = blackLevel * nPixels;
	const uint64_t minValid = 1;

	/*
	 * Make sure the sums are at least minValid, while preventing unsigned
	 * integer underflow.
	 */
	const RGB<uint64_t> sum = stats->sum_.max(offset + minValid) - offset;

	RGB<double> rgbMeans = { { static_cast<double>(sum.r()) / nPixels,
				   static_cast<double>(sum.g()) / nPixels,
				   static_cast<double>(sum.b()) / nPixels } };

	return SimpleAwbStats(rgbMeans);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Awb::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, const SwIspStats *stats,
		  ControlList &metadata)
{
	SimpleAwbStats awbStats = calculateRgbMeans(context, stats);

	awbAlgo_.process(context.activeState.awb, frameContext.awb, awbStats,
			 kDefaultLux, metadata);
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
