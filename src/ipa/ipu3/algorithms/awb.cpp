/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * AWB control algorithm
 */
#include "awb.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

/**
 * \file awb.h
 */

namespace libcamera {

namespace ipa::ipu3::algorithms {

LOG_DEFINE_CATEGORY(IPU3Awb)

/*
 * \todo IPU3 doesn't support the Lux algorithm.
 */
static constexpr unsigned int kDefaultLux = 500;

/**
 * \brief The IPU3 specific implementation of AwbStats
 */
class Ipu3AwbStats final : public AwbStats
{
public:
	Ipu3AwbStats() = default;

	/**
	 * \brief Constructs an Ipu3AwbStats object with the given RGB means
	 *
	 * \param rgbMeans A vector of doubles representing the RGB mean values
	 */
	Ipu3AwbStats(const RGB<double> &rgbMeans)
		: rgbMeans_(rgbMeans)
	{
		rg_ = rgbMeans_.r() / rgbMeans_.g();
		bg_ = rgbMeans_.b() / rgbMeans_.g();
	}

	double computeColourError(const RGB<double> &gains) const override;
	bool valid() const override;
	RGB<double> rgbMeans() const override;

private:
	RGB<double> rgbMeans_;
	double rg_;
	double bg_;
};

double Ipu3AwbStats::computeColourError(const RGB<double> &gains) const
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

bool Ipu3AwbStats::valid() const
{
	/*
	 * This validity assessment is designed to allow libipa to decide
	 * whether there's enough information in the statistics for a frame to
	 * be useful. The IPU3 implementation already drops any statistics zone
	 * with an average value below a threshold though so we don't need to do
	 * it in libipa. Report all stats as valid.
	 */

	return true;
}

RGB<double> Ipu3AwbStats::rgbMeans() const
{
	return rgbMeans_;
}

/*
 * Zones are only considered if their average green value is at least
 * kMinGreenLevelInZone/255 (after black level subtraction) to exclude zones
 * that are too dark and don't provide relevant colour information (on the
 * opposite side of the spectrum, saturated regions are excluded by the ImgU
 * statistics engine).
 */
static constexpr uint32_t kMinGreenLevelInZone = 16;

/*
 * Minimum proportion of non-saturated cells in a zone for the zone to be used
 * by the AWB algorithm.
 */
static constexpr double kMaxCellSaturationRatio = 0.8;

/*
 * Maximum ratio of saturated pixels in a cell for the cell to be considered
 * non-saturated and counted by the AWB algorithm.
 */
static constexpr uint32_t kMinCellsPerZoneRatio = 255 * 90 / 100;

/**
 * \struct Accumulator
 * \brief RGB statistics for a given zone
 *
 * Accumulate red, green and blue values for each non-saturated item over a
 * zone. Items can for instance be pixels, but also the average of groups of
 * pixels, depending on who uses the accumulator.
 * \todo move this description and structure into a common header
 *
 * Zones which are saturated beyond the threshold defined in
 * ipu3_uapi_awb_config_s are not included in the average.
 *
 * \var Accumulator::counted
 * \brief Number of unsaturated cells used to calculate the sums
 *
 * \var Accumulator::sum
 * \brief A structure containing the average red, green and blue sums
 *
 * \var Accumulator::sum.red
 * \brief Sum of the average red values of each unsaturated cell in the zone
 *
 * \var Accumulator::sum.green
 * \brief Sum of the average green values of each unsaturated cell in the zone
 *
 * \var Accumulator::sum.blue
 * \brief Sum of the average blue values of each unsaturated cell in the zone
 */

/* Default settings for Bayer noise reduction replicated from the Kernel */
static const struct ipu3_uapi_bnr_static_config imguCssBnrDefaults = {
	.wb_gains = { 16, 16, 16, 16 },
	.wb_gains_thr = { 255, 255, 255, 255 },
	.thr_coeffs = { 1700, 0, 31, 31, 0, 16 },
	.thr_ctrl_shd = { 26, 26, 26, 26 },
	.opt_center = { -648, 0, -366, 0 },
	.lut = {
		{ 17, 23, 28, 32, 36, 39, 42, 45,
		  48, 51, 53, 55, 58, 60, 62, 64,
		  66, 68, 70, 72, 73, 75, 77, 78,
		  80, 82, 83, 85, 86, 88, 89, 90 } },
	.bp_ctrl = { 20, 0, 1, 40, 0, 6, 0, 6, 0 },
	.dn_detect_ctrl = { 9, 3, 4, 0, 8, 0, 1, 1, 1, 1, 0 },
	.column_size = 1296,
	.opt_center_sqr = { 419904, 133956 },
};

/**
 * \class Awb
 * \brief The IPU3 white balance correction algorithm implementation
 *
 * The IPU3 generates statistics from the Bayer Down Scaler output into a grid
 * defined in the ipu3_uapi_awb_config_s structure.
 *
 * - Cells are defined in Pixels
 * - Zones are defined in Cells
 *
 *                             80 cells
 *            /───────────── 1280 pixels ───────────\
 *                             16 zones
 *             16
 *           ┌────┬────┬────┬────┬────┬─  ──────┬────┐   \
 *           │Cell│    │    │    │    │    |    │    │   │
 *        16 │ px │    │    │    │    │    |    │    │   │
 *           ├────┼────┼────┼────┼────┼─  ──────┼────┤   │
 *           │    │    │    │    │    │    |    │    │
 *           │    │    │    │    │    │    |    │    │   7
 *           │ ── │ ── │ ── │ ── │ ── │ ──  ── ─┤ ── │ 1 2 4
 *           │    │    │    │    │    │    |    │    │ 2 0 5
 *
 *           │    │    │    │    │    │    |    │    │ z p c
 *           ├────┼────┼────┼────┼────┼─  ──────┼────┤ o i e
 *           │    │    │    │    │    │    |    │    │ n x l
 *           │                        │    |    │    │ e e l
 *           ├───                  ───┼─  ──────┼────┤ s l s
 *           │                        │    |    │    │   s
 *           │                        │    |    │    │
 *           ├───   Zone of Cells  ───┼─  ──────┼────┤   │
 *           │        (5 x 4)         │    |    │    │   │
 *           │                        │    |    │    │   │
 *           ├──                   ───┼─  ──────┼────┤   │
 *           │                   │    │    |    │    │   │
 *           │    │    │    │    │    │    |    │    │   │
 *           └────┴────┴────┴────┴────┴─  ──────┴────┘   /
 *
 *
 * In each cell, the ImgU computes for each colour component the average of all
 * unsaturated pixels (below a programmable threshold). It also provides the
 * ratio of saturated pixels in the cell.
 *
 * The AWB algorithm operates on a coarser grid, made by grouping cells from the
 * hardware grid into zones. The number of zones is fixed to \a kAwbStatsSizeX x
 * \a kAwbStatsSizeY. For example, a frame of 1280x720 is divided into 80x45
 * cells of [16x16] pixels and 16x12 zones of [5x4] cells each
 * (\a kAwbStatsSizeX=16 and \a kAwbStatsSizeY=12). If the number of cells isn't
 * an exact multiple of the number of zones, the right-most and bottom-most
 * cells are ignored. The grid configuration is computed by
 * IPAIPU3::calculateBdsGrid().
 *
 * Before running the AWB algorithm, we aggregate the cell averages for each
 * zone in generateAwbStats(). Cells that have a too high ratio of saturated
 * pixels are ignored, and only zones that contain enough non-saturated cells
 * are then used by the algorithm.
 */

Awb::Awb()
	: Algorithm()
{
	zones_.reserve(kAwbStatsSizeX * kAwbStatsSizeY);
}

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
	const ipu3_uapi_grid_config &grid = context.configuration.grid.bdsGrid;
	stride_ = context.configuration.grid.stride;

	awbAlgo_.configure(context.activeState.awb);

	cellsPerZoneX_ = std::round(grid.width / static_cast<double>(kAwbStatsSizeX));
	cellsPerZoneY_ = std::round(grid.height / static_cast<double>(kAwbStatsSizeY));

	/*
	 * Configure the minimum proportion of cells counted within a zone
	 * for it to be used.
	 * \todo This proportion could be configured.
	 */
	cellsPerZoneThreshold_ = cellsPerZoneX_ * cellsPerZoneY_ * kMaxCellSaturationRatio;
	LOG(IPU3Awb, Debug) << "Threshold for AWB is set to " << cellsPerZoneThreshold_;

	return 0;
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

constexpr uint16_t Awb::threshold(float value)
{
	/* AWB thresholds are in the range [0, 8191] */
	return value * 8191;
}

constexpr uint16_t Awb::gainValue(double gain)
{
	/*
	 * The colour gains applied by the BNR for the four channels (Gr, R, B
	 * and Gb) are expressed in the parameters structure as 16-bit integers
	 * that store a fixed-point U3.13 value in the range [0, 8[.
	 *
	 * The real gain value is equal to the gain parameter plus one, i.e.
	 *
	 * Pout = Pin * (1 + gain / 8192)
	 *
	 * where 'Pin' is the input pixel value, 'Pout' the output pixel value,
	 * and 'gain' the gain in the parameters structure as a 16-bit integer.
	 */
	return std::clamp((gain - 1.0) * 8192, 0.0, 65535.0);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] ipu3_uapi_params *params)
{
	awbAlgo_.prepare(context.activeState.awb, frameContext.awb);

	/*
	 * Green saturation thresholds are reduced because we are using the
	 * green channel only in the exposure computation.
	 */
	params->acc_param.awb.config.rgbs_thr_r = threshold(1.0);
	params->acc_param.awb.config.rgbs_thr_gr = threshold(0.9);
	params->acc_param.awb.config.rgbs_thr_gb = threshold(0.9);
	params->acc_param.awb.config.rgbs_thr_b = threshold(1.0);

	/*
	 * Enable saturation inclusion on thr_b for ImgU to update the
	 * ipu3_uapi_awb_set_item->sat_ratio field.
	 */
	params->acc_param.awb.config.rgbs_thr_b |= IPU3_UAPI_AWB_RGBS_THR_B_INCL_SAT |
						   IPU3_UAPI_AWB_RGBS_THR_B_EN;

	const ipu3_uapi_grid_config &grid = context.configuration.grid.bdsGrid;

	params->acc_param.awb.config.grid = context.configuration.grid.bdsGrid;

	/*
	 * Optical center is column start (respectively row start) of the
	 * cell of interest minus its X center (respectively Y center).
	 *
	 * For the moment use BDS as a first approximation, but it should
	 * be calculated based on Shading (SHD) parameters.
	 */
	params->acc_param.bnr = imguCssBnrDefaults;
	Size &bdsOutputSize = context.configuration.grid.bdsOutputSize;
	params->acc_param.bnr.column_size = bdsOutputSize.width;
	params->acc_param.bnr.opt_center.x_reset = grid.x_start - (bdsOutputSize.width / 2);
	params->acc_param.bnr.opt_center.y_reset = grid.y_start - (bdsOutputSize.height / 2);
	params->acc_param.bnr.opt_center_sqr.x_sqr_reset = params->acc_param.bnr.opt_center.x_reset
							* params->acc_param.bnr.opt_center.x_reset;
	params->acc_param.bnr.opt_center_sqr.y_sqr_reset = params->acc_param.bnr.opt_center.y_reset
							* params->acc_param.bnr.opt_center.y_reset;


	params->acc_param.bnr.wb_gains.gr = gainValue(frameContext.awb.gains.g());
	params->acc_param.bnr.wb_gains.r = gainValue(frameContext.awb.gains.r());
	params->acc_param.bnr.wb_gains.b = gainValue(frameContext.awb.gains.b());
	params->acc_param.bnr.wb_gains.gb = gainValue(frameContext.awb.gains.g());

	params->use.acc_awb = 1;
	params->use.acc_bnr = 1;
}

/* Generate an RGB vector with the average values for each zone */
void Awb::generateZones()
{
	zones_.clear();

	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		double counted = awbStats_[i].counted;
		if (counted >= cellsPerZoneThreshold_) {
			RGB<double> zone{{
				static_cast<double>(awbStats_[i].sum.red),
				static_cast<double>(awbStats_[i].sum.green),
				static_cast<double>(awbStats_[i].sum.blue)
			}};

			zone /= counted;

			if (zone.g() >= kMinGreenLevelInZone)
				zones_.push_back(zone);
		}
	}
}

/* Translate the IPU3 statistics into the default statistics zone array */
void Awb::generateAwbStats(const ipu3_uapi_stats_3a *stats)
{
	/*
	 * Generate a (kAwbStatsSizeX x kAwbStatsSizeY) array from the IPU3 grid which is
	 * (grid.width x grid.height).
	 */
	for (unsigned int cellY = 0; cellY < kAwbStatsSizeY * cellsPerZoneY_; cellY++) {
		for (unsigned int cellX = 0; cellX < kAwbStatsSizeX * cellsPerZoneX_; cellX++) {
			uint32_t cellPosition = cellY * stride_ + cellX;
			uint32_t zoneX = cellX / cellsPerZoneX_;
			uint32_t zoneY = cellY / cellsPerZoneY_;

			uint32_t awbZonePosition = zoneY * kAwbStatsSizeX + zoneX;

			/* Cast the initial IPU3 structure to simplify the reading */
			const ipu3_uapi_awb_set_item *currentCell =
				reinterpret_cast<const ipu3_uapi_awb_set_item *>(
					&stats->awb_raw_buffer.meta_data[cellPosition]
				);

			/*
			 * Use cells which have less than 90%
			 * saturation as an initial means to include
			 * otherwise bright cells which are not fully
			 * saturated.
			 *
			 * \todo The 90% saturation rate may require
			 * further empirical measurements and
			 * optimisation during camera tuning phases.
			 */
			if (currentCell->sat_ratio <= kMinCellsPerZoneRatio) {
				/* The cell is not saturated, use the current cell */
				awbStats_[awbZonePosition].counted++;
				uint32_t greenValue = currentCell->Gr_avg + currentCell->Gb_avg;
				awbStats_[awbZonePosition].sum.green += greenValue / 2;
				awbStats_[awbZonePosition].sum.red += currentCell->R_avg;
				awbStats_[awbZonePosition].sum.blue += currentCell->B_avg;
			}
		}
	}
}

void Awb::clearAwbStats()
{
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		awbStats_[i].sum.blue = 0;
		awbStats_[i].sum.red = 0;
		awbStats_[i].sum.green = 0;
		awbStats_[i].counted = 0;
	}
}

Ipu3AwbStats Awb::calculateRgbMeans(const ipu3_uapi_stats_3a *stats)
{
	ASSERT(stats->stats_3a_status.awb_en);

	clearAwbStats();
	generateAwbStats(stats);
	generateZones();

	if (zones_.size() <= 10)
		return {};

	/*
	 * Make a separate list of the derivatives for each of red and blue, so
	 * that we can sort them to exclude the extreme gains. We could
	 * consider some variations, such as normalising all the zones first, or
	 * doing an L2 average etc.
	 */
	std::vector<RGB<double>> &redDerivative(zones_);
	std::vector<RGB<double>> blueDerivative(redDerivative);
	std::sort(redDerivative.begin(), redDerivative.end(),
		  [](RGB<double> const &a, RGB<double> const &b) {
			  return a.g() * b.r() < b.g() * a.r();
		  });
	std::sort(blueDerivative.begin(), blueDerivative.end(),
		  [](RGB<double> const &a, RGB<double> const &b) {
			  return a.g() * b.b() < b.g() * a.b();
		  });

	/* Average the middle half of the values. */
	int discard = redDerivative.size() / 4;

	RGB<double> sumRed{ 0.0 };
	RGB<double> sumBlue{ 0.0 };
	for (auto ri = redDerivative.begin() + discard,
		  bi = blueDerivative.begin() + discard;
	     ri != redDerivative.end() - discard; ri++, bi++)
		sumRed += *ri, sumBlue += *bi;

	double redGain = sumRed.g() / (sumRed.r() + 1),
	       blueGain = sumBlue.g() / (sumBlue.b() + 1);

	return Ipu3AwbStats({ { 1.0 / redGain, 1.0, 1.0 / blueGain } });
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Awb::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const ipu3_uapi_stats_3a *stats,
		  ControlList &metadata)
{
	Ipu3AwbStats awbStats = calculateRgbMeans(stats);

	awbAlgo_.process(context.activeState.awb, frameContext.awb, awbStats,
			 kDefaultLux, metadata);
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
