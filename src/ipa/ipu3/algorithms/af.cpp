/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * af.cpp - IPU3 auto focus algorithm
 */

#include "af.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <numeric>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

/**
 * \file af.h
 */

/*
 * Static variables from ChromiumOS Intel Camera HAL and ia_imaging library:
 * - https://chromium.googlesource.com/chromiumos/platform/arc-camera/+/master/hal/intel/psl/ipu3/statsConverter/ipu3-stats.h
 * - https://chromium.googlesource.com/chromiumos/platform/camera/+/refs/heads/main/hal/intel/ipu3/include/ia_imaging/af_public.h
 */

/** The minimum horizontal grid dimension. */
static constexpr uint8_t kAfMinGridWidth = 16;
/** The minimum vertical grid dimension. */
static constexpr uint8_t kAfMinGridHeight = 16;
/** The maximum horizontal grid dimension. */
static constexpr uint8_t kAfMaxGridWidth = 32;
/** The maximum vertical grid dimension. */
static constexpr uint8_t kAfMaxGridHeight = 24;
/** The minimum value of Log2 of the width of the grid cell. */
static constexpr uint16_t kAfMinGridBlockWidth = 4;
/** The minimum value of Log2 of the height of the grid cell. */
static constexpr uint16_t kAfMinGridBlockHeight = 3;
/** The maximum value of Log2 of the width of the grid cell. */
static constexpr uint16_t kAfMaxGridBlockWidth = 6;
/** The maximum value of Log2 of the height of the grid cell. */
static constexpr uint16_t kAfMaxGridBlockHeight = 6;
/** The number of blocks in vertical axis per slice. */
static constexpr uint16_t kAfDefaultHeightPerSlice = 2;

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

LOG_DEFINE_CATEGORY(IPU3Af)

/**
 * Maximum focus steps of the VCM control
 * \todo should be obtained from the VCM driver
 */
static constexpr uint32_t kMaxFocusSteps = 1023;

/* Minimum focus step for searching appropriate focus */
static constexpr uint32_t kCoarseSearchStep = 30;
static constexpr uint32_t kFineSearchStep = 1;

/* Max ratio of variance change, 0.0 < kMaxChange < 1.0 */
static constexpr double kMaxChange = 0.5;

/* The numbers of frame to be ignored, before performing focus scan. */
static constexpr uint32_t kIgnoreFrame = 10;

/* Fine scan range 0 < kFineRange < 1 */
static constexpr double kFineRange = 0.05;

/* Settings for IPU3 AF filter */
static struct ipu3_uapi_af_filter_config afFilterConfigDefault = {
	.y1_coeff_0 = { 0, 1, 3, 7 },
	.y1_coeff_1 = { 11, 13, 1, 2 },
	.y1_coeff_2 = { 8, 19, 34, 242 },
	.y1_sign_vec = 0x7fdffbfe,
	.y2_coeff_0 = { 0, 1, 6, 6 },
	.y2_coeff_1 = { 13, 25, 3, 0 },
	.y2_coeff_2 = { 25, 3, 177, 254 },
	.y2_sign_vec = 0x4e53ca72,
	.y_calc = { 8, 8, 8, 8 },
	.nf = { 0, 9, 0, 9, 0 },
};

/**
 * \class Af
 * \brief An auto-focus algorithm based on IPU3 statistics
 *
 * This algorithm is used to determine the position of the lens to make a
 * focused image. The IPU3 AF processing block computes the statistics that
 * are composed by two types of filtered value and stores in a AF buffer.
 * Typically, for a clear image, it has a relatively higher contrast than a
 * blurred one. Therefore, if an image with the highest contrast can be
 * found through the scan, the position of the len indicates to a clearest
 * image.
 */
Af::Af()
	: focus_(0), bestFocus_(0), currentVariance_(0.0), previousVariance_(0.0),
	  coarseCompleted_(false), fineCompleted_(false)
{
}

/**
 * \brief Configure the Af given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 * \return 0 on success, a negative error code otherwise
 */
int Af::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	struct ipu3_uapi_grid_config &grid = context.configuration.af.afGrid;
	grid.width = kAfMinGridWidth;
	grid.height = kAfMinGridHeight;
	grid.block_width_log2 = kAfMinGridBlockWidth;
	grid.block_height_log2 = kAfMinGridBlockHeight;

	/*
	 * \todo - while this clamping code is effectively a no-op, it satisfies
	 * the compiler that the constant definitions of the hardware limits
	 * are used, and paves the way to support dynamic grid sizing in the
	 * future. While the block_{width,height}_log2 remain assigned to the
	 * minimum, this code should be optimized out by the compiler.
	 */
	grid.width = std::clamp(grid.width, kAfMinGridWidth, kAfMaxGridWidth);
	grid.height = std::clamp(grid.height, kAfMinGridHeight, kAfMaxGridHeight);

	grid.block_width_log2 = std::clamp(grid.block_width_log2,
					   kAfMinGridBlockWidth,
					   kAfMaxGridBlockWidth);

	grid.block_height_log2 = std::clamp(grid.block_height_log2,
					    kAfMinGridBlockHeight,
					    kAfMaxGridBlockHeight);

	grid.height_per_slice = kAfDefaultHeightPerSlice;

	/* Position the AF grid in the center of the BDS output. */
	Rectangle bds(configInfo.bdsOutputSize);
	Size gridSize(grid.width << grid.block_width_log2,
		      grid.height << grid.block_height_log2);

	/*
	 * \todo - Support request metadata
	 * - Set the ROI based on any input controls in the request
	 * - Return the AF ROI as metadata in the Request
	 */
	Rectangle roi = gridSize.centeredTo(bds.center());
	Point start = roi.topLeft();

	/* x_start and y_start should be even */
	grid.x_start = utils::alignDown(start.x, 2);
	grid.y_start = utils::alignDown(start.y, 2);
	grid.y_start |= IPU3_UAPI_GRID_Y_START_EN;

	/* Initial max focus step */
	maxStep_ = kMaxFocusSteps;

	/* Initial frame ignore counter */
	afIgnoreFrameReset();

	/* Initial focus value */
	context.activeState.af.focus = 0;
	/* Maximum variance of the AF statistics */
	context.activeState.af.maxVariance = 0;
	/* The stable AF value flag. if it is true, the AF should be in a stable state. */
	context.activeState.af.stable = false;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 ipu3_uapi_params *params)
{
	const struct ipu3_uapi_grid_config &grid = context.configuration.af.afGrid;
	params->acc_param.af.grid_cfg = grid;
	params->acc_param.af.filter_config = afFilterConfigDefault;

	/* Enable AF processing block */
	params->use.acc_af = 1;
}

/**
 * \brief AF coarse scan
 * \param[in] context The shared IPA context
 *
 * Find a near focused image using a coarse step. The step is determined by
 * kCoarseSearchStep.
 */
void Af::afCoarseScan(IPAContext &context)
{
	if (coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(context, kCoarseSearchStep)) {
		coarseCompleted_ = true;
		context.activeState.af.maxVariance = 0;
		focus_ = context.activeState.af.focus -
			 (context.activeState.af.focus * kFineRange);
		context.activeState.af.focus = focus_;
		previousVariance_ = 0;
		maxStep_ = std::clamp(focus_ + static_cast<uint32_t>((focus_ * kFineRange)),
				      0U, kMaxFocusSteps);
	}
}

/**
 * \brief AF fine scan
 * \param[in] context The shared IPA context
 *
 * Find an optimum lens position with moving 1 step for each search.
 */
void Af::afFineScan(IPAContext &context)
{
	if (!coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(context, kFineSearchStep)) {
		context.activeState.af.stable = true;
		fineCompleted_ = true;
	}
}

/**
 * \brief AF reset
 * \param[in] context The shared IPA context
 *
 * Reset all the parameters to start over the AF process.
 */
void Af::afReset(IPAContext &context)
{
	if (afNeedIgnoreFrame())
		return;

	context.activeState.af.maxVariance = 0;
	context.activeState.af.focus = 0;
	focus_ = 0;
	context.activeState.af.stable = false;
	ignoreCounter_ = kIgnoreFrame;
	previousVariance_ = 0.0;
	coarseCompleted_ = false;
	fineCompleted_ = false;
	maxStep_ = kMaxFocusSteps;
}

/**
 * \brief AF variance comparison
 * \param[in] context The IPA context
 * \param[in] min_step The VCM movement step
 *
 * We always pick the largest variance to replace the previous one. The image
 * with a larger variance also indicates it is a clearer image than previous
 * one. If we find a negative derivative, we return immediately.
 *
 * \return True, if it finds a AF value.
 */
bool Af::afScan(IPAContext &context, int min_step)
{
	if (focus_ > maxStep_) {
		/* If reach the max step, move lens to the position. */
		context.activeState.af.focus = bestFocus_;
		return true;
	} else {
		/*
		 * Find the maximum of the variance by estimating its
		 * derivative. If the direction changes, it means we have
		 * passed a maximum one step before.
		 */
		if ((currentVariance_ - context.activeState.af.maxVariance) >=
		    -(context.activeState.af.maxVariance * 0.1)) {
			/*
			 * Positive and zero derivative:
			 * The variance is still increasing. The focus could be
			 * increased for the next comparison. Also, the max variance
			 * and previous focus value are updated.
			 */
			bestFocus_ = focus_;
			focus_ += min_step;
			context.activeState.af.focus = focus_;
			context.activeState.af.maxVariance = currentVariance_;
		} else {
			/*
			 * Negative derivative:
			 * The variance starts to decrease which means the maximum
			 * variance is found. Set focus step to previous good one
			 * then return immediately.
			 */
			context.activeState.af.focus = bestFocus_;
			return true;
		}
	}

	previousVariance_ = currentVariance_;
	LOG(IPU3Af, Debug) << " Previous step is "
			   << bestFocus_
			   << " Current step is "
			   << focus_;
	return false;
}

/**
 * \brief Determine the frame to be ignored
 * \return Return True if the frame should be ignored, false otherwise
 */
bool Af::afNeedIgnoreFrame()
{
	if (ignoreCounter_ == 0)
		return false;
	else
		ignoreCounter_--;
	return true;
}

/**
 * \brief Reset frame ignore counter
 */
void Af::afIgnoreFrameReset()
{
	ignoreCounter_ = kIgnoreFrame;
}

/**
 * \brief Estimate variance
 * \param[in] y_items The AF filter data set from the IPU3 statistics buffer
 * \param[in] isY1 Selects between filter Y1 or Y2 to calculate the variance
 *
 * Calculate the mean of the data set provided by \a y_item, and then calculate
 * the variance of that data set from the mean.
 *
 * The operation can work on one of two sets of values contained within the
 * y_item data set supplied by the IPU3. The two data sets are the results of
 * both the Y1 and Y2 filters which are used to support coarse (Y1) and fine
 * (Y2) calculations of the contrast.
 *
 * \return The variance of the values in the data set \a y_item selected by \a isY1
 */
double Af::afEstimateVariance(Span<const y_table_item_t> y_items, bool isY1)
{
	uint32_t total = 0;
	double mean;
	double var_sum = 0;

	for (auto y : y_items)
		total += isY1 ? y.y1_avg : y.y2_avg;

	mean = total / y_items.size();

	for (auto y : y_items) {
		double avg = isY1 ? y.y1_avg : y.y2_avg;
		var_sum += pow(avg - mean, 2);
	}

	return var_sum / y_items.size();
}

/**
 * \brief Determine out-of-focus situation
 * \param[in] context The IPA context
 *
 * Out-of-focus means that the variance change rate for a focused and a new
 * variance is greater than a threshold.
 *
 * \return True if the variance threshold is crossed indicating lost focus,
 * false otherwise
 */
bool Af::afIsOutOfFocus(IPAContext &context)
{
	const uint32_t diff_var = std::abs(currentVariance_ -
					   context.activeState.af.maxVariance);
	const double var_ratio = diff_var / context.activeState.af.maxVariance;

	LOG(IPU3Af, Debug) << "Variance change rate: "
			   << var_ratio
			   << " Current VCM step: "
			   << context.activeState.af.focus;

	if (var_ratio > kMaxChange)
		return true;
	else
		return false;
}

/**
 * \brief Determine the max contrast image and lens position
 * \param[in] context The IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The current frame context
 * \param[in] stats The statistics buffer of IPU3
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * Ideally, a clear image also has a relatively higher contrast. So, every
 * image for each focus step should be tested to find an optimal focus step.
 *
 * The Hill Climbing Algorithm[1] is used to find the maximum variance of the
 * AF statistics which is the AF output of IPU3. The focus step is increased
 * then the variance of the AF statistics are estimated. If it finds the
 * negative derivative we have just passed the peak, and we infer that the best
 * focus is found.
 *
 * [1] Hill Climbing Algorithm, https://en.wikipedia.org/wiki/Hill_climbing
 */
void Af::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const ipu3_uapi_stats_3a *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	/* Evaluate the AF buffer length */
	uint32_t afRawBufferLen = context.configuration.af.afGrid.width *
				  context.configuration.af.afGrid.height;

	ASSERT(afRawBufferLen < IPU3_UAPI_AF_Y_TABLE_MAX_SIZE);

	Span<const y_table_item_t> y_items(reinterpret_cast<const y_table_item_t *>(&stats->af_raw_buffer.y_table),
					   afRawBufferLen);

	/*
	 * Calculate the mean and the variance of AF statistics for a given grid.
	 * For coarse: y1 are used.
	 * For fine: y2 results are used.
	 */
	currentVariance_ = afEstimateVariance(y_items, !coarseCompleted_);

	if (!context.activeState.af.stable) {
		afCoarseScan(context);
		afFineScan(context);
	} else {
		if (afIsOutOfFocus(context))
			afReset(context);
		else
			afIgnoreFrameReset();
	}
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
