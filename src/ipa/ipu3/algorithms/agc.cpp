/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <span>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/colours.h"
#include "libipa/histogram.h"

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 *
 * This algorithm calculates an exposure time and an analogue gain so that the
 * average value of the green channel of the brightest 2% of pixels approaches
 * 0.5. The AWB gains are not used here, and all cells in the grid have the same
 * weight, like an average-metering case. In this metering mode, the camera uses
 * light information from the entire scene and creates an average for the final
 * exposure setting, giving no weighting to any particular portion of the
 * metered area.
 *
 * Reference: Battiato, Messina & Castorina. (2008). Exposure
 * Correction for Imaging Devices: An Overview. 10.1201/9781420054538.ch12.
 */

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;

Agc::Agc()
{
}

/**
 * \brief Initialise the AGC algorithm from tuning files
 * \param[in] context The shared IPA context
 * \param[in] tuningData The ValueNode containing Agc tuning data
 *
 * This function calls the base class' tuningData parsers to discover which
 * control values are supported.
 *
 * \return 0 on success or errors from the base class
 */
int Agc::init(IPAContext &context, const ValueNode &tuningData)
{
	return agc_.init(tuningData, context.camHelper.get(), {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	stride_ =  context.configuration.grid.stride;
	bdsGrid_ = context.configuration.grid.bdsGrid;

	return agc_.configure(context.configuration.agc, context.activeState.agc, {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Agc::queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext, const ControlList &controls)
{
	agc_.queueRequest(context.configuration.agc, context.activeState.agc,
			  frameContext.agc, controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] ipu3_uapi_params *params)
{
	agc_.prepare(context.activeState.agc, frameContext.agc);
}

Histogram Agc::parseStatistics(const ipu3_uapi_stats_3a *stats,
			       const ipu3_uapi_grid_config &grid)
{
	uint32_t hist[knumHistogramBins] = { 0 };

	rgbTriples_.clear();

	for (unsigned int cellY = 0; cellY < grid.height; cellY++) {
		for (unsigned int cellX = 0; cellX < grid.width; cellX++) {
			uint32_t cellPosition = cellY * stride_ + cellX;

			const ipu3_uapi_awb_set_item *cell =
				reinterpret_cast<const ipu3_uapi_awb_set_item *>(
					&stats->awb_raw_buffer.meta_data[cellPosition]);

			rgbTriples_.push_back({
				cell->R_avg,
				(cell->Gr_avg + cell->Gb_avg) / 2,
				cell->B_avg
			});

			/*
			 * Store the average green value to estimate the
			 * brightness. Even the overexposed pixels are
			 * taken into account.
			 */
			hist[(cell->Gr_avg + cell->Gb_avg) / 2]++;
		}
	}

	return Histogram(std::span<uint32_t>(hist));
}

namespace {

class AgcTraits final : public AgcMeanLuminance::Traits
{
public:
	AgcTraits(std::span<const std::tuple<uint8_t, uint8_t, uint8_t>> rgbTriples,
		  RGB<double> gains, const ipu3_uapi_grid_config &bdsGrid)
		: rgbTriples_(rgbTriples), gains_(gains), bdsGrid_(bdsGrid)
	{
	}

	/**
	 * \brief Estimate the relative luminance of the frame with a given gain
	 * \param[in] gain The gain to apply in estimating luminance
	 *
	 * The estimation is based on the AWB statistics for the current frame. Red,
	 * green and blue averages for all cells are first multiplied by the gain, and
	 * then saturated to approximate the sensor behaviour at high brightness
	 * values. The approximation is quite rough, as it doesn't take into account
	 * non-linearities when approaching saturation.
	 *
	 * The relative luminance (Y) is computed from the linear RGB components using
	 * the Rec. 601 formula. The values are normalized to the [0.0, 1.0] range,
	 * where 1.0 corresponds to a theoretical perfect reflector of 100% reference
	 * white.
	 *
	 * More detailed information can be found in:
	 * https://en.wikipedia.org/wiki/Relative_luminance
	 *
	 * \return The relative luminance of the frame
	 */

	double estimateLuminance(double gain) const override
	{
		RGB<double> sum{ 0.0 };

		for (unsigned int i = 0; i < rgbTriples_.size(); i++) {
			sum.r() += std::min(std::get<0>(rgbTriples_[i]) * gain, 255.0);
			sum.g() += std::min(std::get<1>(rgbTriples_[i]) * gain, 255.0);
			sum.b() += std::min(std::get<2>(rgbTriples_[i]) * gain, 255.0);
		}

		double ySum = rec601LuminanceFromRGB(sum * gains_);
		return ySum / (bdsGrid_.height * bdsGrid_.width) / 255;
	}

private:
	std::span<const std::tuple<uint8_t, uint8_t, uint8_t>> rgbTriples_;
	RGB<double> gains_;
	const ipu3_uapi_grid_config &bdsGrid_;
};

} /* namespace */

/**
 * \brief Process IPU3 statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] frame The current frame sequence number
 * \param[in] frameContext The current frame context
 * \param[in] stats The IPU3 statistics and ISP results
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const ipu3_uapi_stats_3a *stats,
		  ControlList &metadata)
{
	Histogram hist = parseStatistics(stats, context.configuration.grid.bdsGrid);
	ipa::awb::Context awbContext;

	if (context.activeState.awb.autoEnabled)
		awbContext = context.activeState.awb.automatic;
	else
		awbContext = context.activeState.awb.manual;

	agc_.process(context.configuration.agc, context.activeState.agc, frameContext.agc, {{
		.traits = AgcTraits{rgbTriples_, awbContext.gains, bdsGrid_},
		.yHist = hist,
		.exposure = frameContext.sensor.exposure,
		.gain = frameContext.sensor.gain,
	}}, metadata);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
