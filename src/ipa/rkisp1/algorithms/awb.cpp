/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * awb.cpp - AWB control algorithm
 */

#include "awb.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file awb.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class Awb
 * \brief A Grey world white balance correction algorithm
 */

LOG_DEFINE_CATEGORY(RkISP1Awb)

/* Minimum mean value below which AWB can't operate. */
constexpr double kMeanMinThreshold = 2.0;

Awb::Awb()
	: rgbMode_(false)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   const IPACameraSensorInfo &configInfo)
{
	context.activeState.awb.gains.manual.red = 1.0;
	context.activeState.awb.gains.manual.blue = 1.0;
	context.activeState.awb.gains.manual.green = 1.0;
	context.activeState.awb.gains.automatic.red = 1.0;
	context.activeState.awb.gains.automatic.blue = 1.0;
	context.activeState.awb.gains.automatic.green = 1.0;
	context.activeState.awb.autoEnabled = true;

	/*
	 * Define the measurement window for AWB as a centered rectangle
	 * covering 3/4 of the image width and height.
	 */
	context.configuration.awb.measureWindow.h_offs = configInfo.outputSize.width / 8;
	context.configuration.awb.measureWindow.v_offs = configInfo.outputSize.height / 8;
	context.configuration.awb.measureWindow.h_size = 3 * configInfo.outputSize.width / 4;
	context.configuration.awb.measureWindow.v_size = 3 * configInfo.outputSize.height / 4;

	context.configuration.awb.enabled = true;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Awb::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &awb = context.activeState.awb;

	const auto &awbEnable = controls.get(controls::AwbEnable);
	if (awbEnable && *awbEnable != awb.autoEnabled) {
		awb.autoEnabled = *awbEnable;

		LOG(RkISP1Awb, Debug)
			<< (*awbEnable ? "Enabling" : "Disabling") << " AWB";
	}

	const auto &colourGains = controls.get(controls::ColourGains);
	if (colourGains && !awb.autoEnabled) {
		awb.gains.manual.red = (*colourGains)[0];
		awb.gains.manual.blue = (*colourGains)[1];

		LOG(RkISP1Awb, Debug)
			<< "Set colour gains to red: " << awb.gains.manual.red
			<< ", blue: " << awb.gains.manual.blue;
	}

	frameContext.awb.autoEnabled = awb.autoEnabled;

	if (!awb.autoEnabled) {
		frameContext.awb.gains.red = awb.gains.manual.red;
		frameContext.awb.gains.green = 1.0;
		frameContext.awb.gains.blue = awb.gains.manual.blue;
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, rkisp1_params_cfg *params)
{
	/*
	 * This is the latest time we can read the active state. This is the
	 * most up-to-date automatic values we can read.
	 */
	if (frameContext.awb.autoEnabled) {
		frameContext.awb.gains.red = context.activeState.awb.gains.automatic.red;
		frameContext.awb.gains.green = context.activeState.awb.gains.automatic.green;
		frameContext.awb.gains.blue = context.activeState.awb.gains.automatic.blue;
	}

	params->others.awb_gain_config.gain_green_b = 256 * frameContext.awb.gains.green;
	params->others.awb_gain_config.gain_blue = 256 * frameContext.awb.gains.blue;
	params->others.awb_gain_config.gain_red = 256 * frameContext.awb.gains.red;
	params->others.awb_gain_config.gain_green_r = 256 * frameContext.awb.gains.green;

	/* Update the gains. */
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;

	/* If we have already set the AWB measurement parameters, return. */
	if (frame > 0)
		return;

	rkisp1_cif_isp_awb_meas_config &awb_config = params->meas.awb_meas_config;

	/* Configure the measure window for AWB. */
	awb_config.awb_wnd = context.configuration.awb.measureWindow;

	/* Number of frames to use to estimate the means (0 means 1 frame). */
	awb_config.frames = 0;

	/* Select RGB or YCbCr means measurement. */
	if (rgbMode_) {
		awb_config.awb_mode = RKISP1_CIF_ISP_AWB_MODE_RGB;

		/*
		 * For RGB-based measurements, pixels are selected with maximum
		 * red, green and blue thresholds that are set in the
		 * awb_ref_cr, awb_min_y and awb_ref_cb respectively. The other
		 * values are not used, set them to 0.
		 */
		awb_config.awb_ref_cr = 250;
		awb_config.min_y = 250;
		awb_config.awb_ref_cb = 250;

		awb_config.max_y = 0;
		awb_config.min_c = 0;
		awb_config.max_csum = 0;
	} else {
		awb_config.awb_mode = RKISP1_CIF_ISP_AWB_MODE_YCBCR;

		/* Set the reference Cr and Cb (AWB target) to white. */
		awb_config.awb_ref_cb = 128;
		awb_config.awb_ref_cr = 128;

		/*
		 * Filter out pixels based on luminance and chrominance values.
		 * The acceptable luma values are specified as a [16, 250]
		 * range, while the acceptable chroma values are specified with
		 * a minimum of 16 and a maximum Cb+Cr sum of 250.
		 */
		awb_config.min_y = 16;
		awb_config.max_y = 250;
		awb_config.min_c = 16;
		awb_config.max_csum = 250;
	}

	/* Enable the AWB gains. */
	params->module_en_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;

	/* Update the AWB measurement parameters and enable the AWB module. */
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB;
	params->module_en_update |= RKISP1_CIF_ISP_MODULE_AWB;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_AWB;
}

uint32_t Awb::estimateCCT(double red, double green, double blue)
{
	/* Convert the RGB values to CIE tristimulus values (XYZ) */
	double X = (-0.14282) * (red) + (1.54924) * (green) + (-0.95641) * (blue);
	double Y = (-0.32466) * (red) + (1.57837) * (green) + (-0.73191) * (blue);
	double Z = (-0.68202) * (red) + (0.77073) * (green) + (0.56332) * (blue);

	/* Calculate the normalized chromaticity values */
	double x = X / (X + Y + Z);
	double y = Y / (X + Y + Z);

	/* Calculate CCT */
	double n = (x - 0.3320) / (0.1858 - y);
	return 449 * n * n * n + 3525 * n * n + 6823.3 * n + 5520.33;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const rkisp1_stat_buffer *stats,
		  ControlList &metadata)
{
	const rkisp1_cif_isp_stat *params = &stats->params;
	const rkisp1_cif_isp_awb_stat *awb = &params->awb;
	IPAActiveState &activeState = context.activeState;
	double greenMean;
	double redMean;
	double blueMean;

	if (rgbMode_) {
		greenMean = awb->awb_mean[0].mean_y_or_g;
		redMean = awb->awb_mean[0].mean_cr_or_r;
		blueMean = awb->awb_mean[0].mean_cb_or_b;
	} else {
		/* Get the YCbCr mean values */
		double yMean = awb->awb_mean[0].mean_y_or_g;
		double cbMean = awb->awb_mean[0].mean_cb_or_b;
		double crMean = awb->awb_mean[0].mean_cr_or_r;

		/*
		 * Convert from YCbCr to RGB.
		 * The hardware uses the following formulas:
		 * Y = 16 + 0.2500 R + 0.5000 G + 0.1094 B
		 * Cb = 128 - 0.1406 R - 0.2969 G + 0.4375 B
		 * Cr = 128 + 0.4375 R - 0.3750 G - 0.0625 B
		 *
		 * The inverse matrix is thus:
		 * [[1,1636, -0,0623,  1,6008]
		 *  [1,1636, -0,4045, -0,7949]
		 *  [1,1636,  1,9912, -0,0250]]
		 */
		yMean -= 16;
		cbMean -= 128;
		crMean -= 128;
		redMean = 1.1636 * yMean - 0.0623 * cbMean + 1.6008 * crMean;
		greenMean = 1.1636 * yMean - 0.4045 * cbMean - 0.7949 * crMean;
		blueMean = 1.1636 * yMean + 1.9912 * cbMean - 0.0250 * crMean;

		/*
		 * Due to hardware rounding errors in the YCbCr means, the
		 * calculated RGB means may be negative. This would lead to
		 * negative gains, messing up calculation. Prevent this by
		 * clamping the means to positive values.
		 */
		redMean = std::max(redMean, 0.0);
		greenMean = std::max(greenMean, 0.0);
		blueMean = std::max(blueMean, 0.0);
	}

	/*
	 * The ISP computes the AWB means after applying the colour gains,
	 * divide by the gains that were used to get the raw means from the
	 * sensor.
	 */
	redMean /= frameContext.awb.gains.red;
	greenMean /= frameContext.awb.gains.green;
	blueMean /= frameContext.awb.gains.blue;

	/*
	 * If the means are too small we don't have enough information to
	 * meaningfully calculate gains. Freeze the algorithm in that case.
	 */
	if (redMean < kMeanMinThreshold && greenMean < kMeanMinThreshold &&
	    blueMean < kMeanMinThreshold) {
		frameContext.awb.temperatureK = activeState.awb.temperatureK;
		return;
	}

	activeState.awb.temperatureK = estimateCCT(redMean, greenMean, blueMean);

	/*
	 * Estimate the red and blue gains to apply in a grey world. The green
	 * gain is hardcoded to 1.0. Avoid divisions by zero by clamping the
	 * divisor to a minimum value of 1.0.
	 */
	double redGain = greenMean / std::max(redMean, 1.0);
	double blueGain = greenMean / std::max(blueMean, 1.0);

	/*
	 * Clamp the gain values to the hardware, which expresses gains as Q2.8
	 * unsigned integer values. Set the minimum just above zero to avoid
	 * divisions by zero when computing the raw means in subsequent
	 * iterations.
	 */
	redGain = std::clamp(redGain, 1.0 / 256, 1023.0 / 256);
	blueGain = std::clamp(blueGain, 1.0 / 256, 1023.0 / 256);

	/* Filter the values to avoid oscillations. */
	double speed = 0.2;
	redGain = speed * redGain + (1 - speed) * activeState.awb.gains.automatic.red;
	blueGain = speed * blueGain + (1 - speed) * activeState.awb.gains.automatic.blue;

	activeState.awb.gains.automatic.red = redGain;
	activeState.awb.gains.automatic.blue = blueGain;
	activeState.awb.gains.automatic.green = 1.0;

	frameContext.awb.temperatureK = activeState.awb.temperatureK;

	metadata.set(controls::AwbEnable, frameContext.awb.autoEnabled);
	metadata.set(controls::ColourGains, {
			static_cast<float>(frameContext.awb.gains.red),
			static_cast<float>(frameContext.awb.gains.blue)
		});
	metadata.set(controls::ColourTemperature, frameContext.awb.temperatureK);

	LOG(RkISP1Awb, Debug) << std::showpoint
		<< "Means [" << redMean << ", " << greenMean << ", " << blueMean
		<< "], gains [" << activeState.awb.gains.automatic.red << ", "
		<< activeState.awb.gains.automatic.green << ", "
		<< activeState.awb.gains.automatic.blue << "], temp "
		<< frameContext.awb.temperatureK << "K";
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
