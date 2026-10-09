/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas on Board, Oy
 *
 * IPU3 Lens Shading Correction algorithm
 */

#include "lsc.h"

#include <bit>
#include <cmath>

#include <libcamera/control_ids.h>

/**
 * \file lsc.h
 * \brief IPU3 Lens Shading Correction
 */

namespace libcamera {

namespace ipa::ipu3::algorithms {

/**
 * \class Lsc
 * \brief IPU3 Lens Shading Correction algorithm
 */

LOG_DEFINE_CATEGORY(IPU3Lsc)

static constexpr unsigned int kMaxNumHCells = 73;
static constexpr unsigned int kMaxNumVCells = 56;
static constexpr int kColourTemperatureQuantization = 10;

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Lsc::init(IPAContext &context, const ValueNode &tuningData)
{
	/*
	 * The IPU3 lens shading block expects a table of data that isn't of a
	 * fixed size, but rather is configurable based on 4 parameters:
	 *
	 * block_width_log2:	The log2 of the horizontal pixel count per cell
	 * block_height_log2:	The log2 of the vertical pixel count per cell
	 * width:		The number of horizontal cells
	 * height:		The number of vertical cells
	 *
	 * The constructed grid should be capable of covering the image, but
	 * ideally won't extend past the edges of the image. Fixing either set
	 * of parameters for the algorithm as a whole is likely to result in
	 * suboptimal situations for some sensors, so let's determine them
	 * programmatically instead.
	 *
	 * What we want is the densest possible grid that ideally doesn't extend
	 * past the edges of the image at all. The maximum grid size is 73x56,
	 * which gives us the lower bounds on cell size. Unfortunately we can
	 * only specify sizes in powers of two, which can have the effect of
	 * making the grid much more coarse. For example for a 2592x1944 input
	 * image, 2592 / 73 = 35.5...which means we need to set blockWidthLog2
	 * to 6 (I.E. 64) and have just 40.5 (or rather 41) cells horizontally.
	 */
	sensorWidth_ = context.sensorInfo.activeArea.width;
	sensorHeight_ = context.sensorInfo.activeArea.height;

	unsigned int cellWidth = (sensorWidth_ + kMaxNumHCells - 1) / kMaxNumHCells;
	unsigned int cellHeight = (sensorHeight_ + kMaxNumVCells - 1) / kMaxNumVCells;

	unsigned int minCellWidth = std::bit_ceil(cellWidth);
	unsigned int minCellHeight = std::bit_ceil(cellHeight);

	numHCells_ = (sensorWidth_ + minCellWidth - 1) / minCellWidth;
	numVCells_ = (sensorHeight_ + minCellHeight - 1) / minCellHeight;

	blockWidthLog2_ = std::bit_width(minCellWidth) - 1;
	blockHeightLog2_ = std::bit_width(minCellHeight) - 1;

	LOG(IPU3Lsc, Debug) << "Calculated Grid configuration: "
			    << numHCells_ << "x" << numVCells_ << " cells of "
			    << minCellWidth << "x" << minCellHeight << " pixels";

	/*
	 * We need to know if we're running the polynomial algorithm or not as
	 * things will behave slightly differently.
	 */
	polynomial_ = tuningData["type"].get<std::string>() == "polynomial";

	return lscAlgo_.init(tuningData, context.ctrlMap,
			     { .keys = { "r", "gr", "gb", "b" },
			       .numHSamples = numHCells_,
			       .numVSamples = numVCells_,
			       .sensorSize = context.sensorInfo.activeArea.size() });
}

std::vector<double> Lsc::calculatePositions(unsigned int dimension)
{
	std::vector<double> positions(dimension);
	for (double i = 0.0; i < dimension; i++)
		positions[i] = i / (dimension - 1);

	return positions;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Lsc::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	cropWidth_ = configInfo.sensorInfo.analogCrop.width;
	cropHeight_ = configInfo.sensorInfo.analogCrop.height;
	std::vector<double> xPos = calculatePositions(numHCells_);
	std::vector<double> yPos = calculatePositions(numVCells_);

	return lscAlgo_.configure(context.activeState.lsc,
				  configInfo.sensorInfo.analogCrop, xPos, yPos);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Lsc::queueRequest(IPAContext &context, const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	/*
	 * The base algorithm defines the LensShadingCorrectionEnable control
	 * with a default value of true, but actually the IPU3 driver defaults
	 * it to off. If this is the first frame, check for the control, but if
	 * there isn't one, force it on to fulfil the advertised default.
	 */
	if (frame == 0) {
		const auto &lscEnable = controls.get(controls::LensShadingCorrectionEnable);
		if (!lscEnable) {
			frameContext.lsc.enabled = true;
			frameContext.lsc.update = true;
		}
	}

	lscAlgo_.queueRequest(context.activeState.lsc, frameContext.lsc, controls);
}

static unsigned int quantize(unsigned int value, unsigned int step)
{
	return std::lround(value / static_cast<double>(step)) * step;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Lsc::prepare([[maybe_unused]] IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, ipu3_uapi_params *params)
{
	uint32_t ct = frameContext.awb.colourTemperature;
	unsigned int quantizedCt = quantize(ct, kColourTemperatureQuantization);

	if (!frameContext.lsc.update) {
		if (!frameContext.lsc.enabled)
			return;

		/*
		 * Add a threshold so that oscillations around a quantization
		 * step don't lead to constant changes.
		 */
		if (utils::abs_diff(ct, lastAppliedCt_) < kColourTemperatureQuantization / 2)
			return;

		if (quantizedCt == lastAppliedQuantizedCt_)
			return;
	}

	/*
	 * This flag tells the kernel driver that it should read the LSC params
	 * passed from userspace instead of using its cached copy.
	 */
	params->use.acc_shd = 1;

	/*
	 * Pass the enabled flag. If we're not enabled, we can then just bail
	 * out.
	 */
	ipu3_uapi_shd_config_static *config = &params->acc_param.shd.shd;
	config->general.shd_enable = frameContext.lsc.enabled;

	if (!frameContext.lsc.enabled)
		return;

	config->grid.width = numHCells_;
	config->grid.height = numVCells_;
	config->grid.block_width_log2 = blockWidthLog2_;
	config->grid.block_height_log2 = blockHeightLog2_;
	config->grid.grid_height_per_slice = IPU3_UAPI_SHD_MAX_CELLS_PER_SET / numHCells_;

	/*
	 * The IPU3's documentation describes the x_start and y_start members
	 * as follows:
	 *
	 * "[X/Y] value of top left corner of sensor relative to ROI
	 * s13, [-4096, 0], default 0, only negative values."
	 *
	 * I interpret that as allowing us to configure the cropped rectangle
	 * relative to the full grid. That's useful if we're running the tabular
	 * algorithm, which would otherwise apply the full grid inappropriately.
	 * If we're running the polynomial one though the calculated grid is
	 * probably more appropriate than a coarse application of the full grid
	 * so let's tell the hardware not to bother correcting in that case.
	 */
	if (polynomial_) {
		config->grid.x_start = 0;
		config->grid.y_start = 0;
	} else {
		config->grid.x_start = (cropWidth_ - sensorWidth_) / 2;
		config->grid.y_start = (cropHeight_ - sensorHeight_) / 2;
	}

	/* No idea what this is, but the docs say it should be set as so */
	config->general.init_set_vrt_offst_ul =
		config->grid.y_start >> (config->grid.block_height_log2 %
					 config->grid.grid_height_per_slice);

	/*
	 * Values in the LUT cease taking effect at 4096, and a value of 0.0 is
	 * "no correction" rather than black. The gain factor is described by
	 * the documentation like so:
	 *
	 * "Shift calculated anti shading value. Precision u2. 0x0 - gain factor
	 * [1, 5], means no shift interpolated value. 0x1 - gain factor [1, 9],
	 * means shift interpolated by 1. 0x2 - gain factor [1, 17], means shift
	 * interpolated by 2."
	 *
	 * The simplest interpretation for those pieces of information is I
	 * think that the LUT stores 12-bit Q numbers who's represented values
	 * depend on the gain_factor setting like so:
	 *
	 * 0: UQ<2, 10> representing values in range [0, 4)
	 * 1: UQ<3, 9> representing values in range [0, 8)
	 * 2: UQ<4, 8> representing values in range [0, 16)
	 *
	 * And that a base gain of 1.0 is added to those configured values. As a
	 * gain of more than 5.0 is fairly unlikely, let's fix gain_factor to 0
	 * for now and revisit if needed.
	 */
	config->general.gain_factor = 0;

	/*
	 * Disable the black level settings here - we do that through another
	 * parameters block.
	 */
	config->black_level.bl_r = 0;
	config->black_level.bl_gr = 0;
	config->black_level.bl_gb = 0;
	config->black_level.bl_b = 0;

	ipu3_uapi_shd_lut *lut = &params->acc_param.shd.shd_lut;

	const auto &set = lscAlgo_.interpolateComponents(quantizedCt);

	unsigned int totalCells = numHCells_ * numVCells_;
	unsigned int cellsPerSet = numHCells_ * config->grid.grid_height_per_slice;
	unsigned int numSets = (numHCells_ + config->grid.grid_height_per_slice - 1) /
			       config->grid.grid_height_per_slice;
	unsigned int i = 0;

	for (unsigned int s = 0; s < numSets; s++) {
		for (unsigned int c = 0; c < cellsPerSet && i < totalCells; c++, i++) {
			lut->sets[s].r_and_gr[c].r = set.at("r")[i];
			lut->sets[s].r_and_gr[c].gr = set.at("gr")[i];
			lut->sets[s].gb_and_b[c].gb = set.at("gb")[i];
			lut->sets[s].gb_and_b[c].b = set.at("b")[i];
		}
	}

	lastAppliedCt_ = ct;
	lastAppliedQuantizedCt_ = quantizedCt;
	LOG(IPU3Lsc, Debug)
		<< "ct is " << ct << ", quantized to "
		<< quantizedCt;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Lsc::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const ipu3_uapi_stats_3a *stats,
		  ControlList &metadata)
{
	lscAlgo_.process(frameContext.lsc, metadata);
}

REGISTER_IPA_ALGORITHM(Lsc, "Lsc")

} // namespace ipa::ipu3::algorithms

} // namespace libcamera
