/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * awb.cpp - AWB control algorithm
 */

#include "awb.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

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

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   const IPACameraSensorInfo &configInfo)
{
	context.frameContext.awb.gains.red = 1.0;
	context.frameContext.awb.gains.blue = 1.0;
	context.frameContext.awb.gains.green = 1.0;

	/*
	 * Define the measurement window for AWB as a centered rectangle
	 * covering 3/4 of the image width and height.
	 */
	context.configuration.awb.measureWindow.h_offs = configInfo.outputSize.width / 8;
	context.configuration.awb.measureWindow.v_offs = configInfo.outputSize.height / 8;
	context.configuration.awb.measureWindow.h_size = 3 * configInfo.outputSize.width / 4;
	context.configuration.awb.measureWindow.v_size = 3 * configInfo.outputSize.height / 4;

	return 0;
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
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context, rkisp1_params_cfg *params)
{
	params->others.awb_gain_config.gain_green_b = 256 * context.frameContext.awb.gains.green;
	params->others.awb_gain_config.gain_blue = 256 * context.frameContext.awb.gains.blue;
	params->others.awb_gain_config.gain_red = 256 * context.frameContext.awb.gains.red;
	params->others.awb_gain_config.gain_green_r = 256 * context.frameContext.awb.gains.green;

	/* Update the gains. */
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;

	/* If we already have configured the gains and window, return. */
	if (context.frameContext.frameCount > 0)
		return;

	/* Configure the gains to apply. */
	params->module_en_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;
	/* Update the ISP to apply the gains configured. */
	params->module_ens |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;

	/* Configure the measure window for AWB. */
	params->meas.awb_meas_config.awb_wnd = context.configuration.awb.measureWindow;
	/*
	 * Measure Y, Cr and Cb means.
	 * \todo RGB is not working, the kernel seems to not configure it ?
	 */
	params->meas.awb_meas_config.awb_mode = RKISP1_CIF_ISP_AWB_MODE_YCBCR;
	/* Reference Cr and Cb. */
	params->meas.awb_meas_config.awb_ref_cb = 128;
	params->meas.awb_meas_config.awb_ref_cr = 128;
	/* Y values to include are between min_y and max_y only. */
	params->meas.awb_meas_config.min_y = 16;
	params->meas.awb_meas_config.max_y = 250;
	/* Maximum Cr+Cb value to take into account for awb. */
	params->meas.awb_meas_config.max_csum = 250;
	/* Minimum Cr and Cb values to take into account. */
	params->meas.awb_meas_config.min_c = 16;
	/* Number of frames to use to estimate the mean (0 means 1 frame). */
	params->meas.awb_meas_config.frames = 0;

	/* Update AWB measurement unit configuration. */
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB;
	/* Make sure the ISP is measuring the means for the next frame. */
	params->module_en_update |= RKISP1_CIF_ISP_MODULE_AWB;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_AWB;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Awb::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] IPAFrameContext *frameCtx,
		  const rkisp1_stat_buffer *stats)
{
	const rkisp1_cif_isp_stat *params = &stats->params;
	const rkisp1_cif_isp_awb_stat *awb = &params->awb;
	IPAFrameContext &frameContext = context.frameContext;

	/* Get the YCbCr mean values */
	double yMean = awb->awb_mean[0].mean_y_or_g;
	double crMean = awb->awb_mean[0].mean_cr_or_r;
	double cbMean = awb->awb_mean[0].mean_cb_or_b;

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
	double redMean = 1.1636 * yMean - 0.0623 * cbMean + 1.6008 * crMean;
	double greenMean = 1.1636 * yMean - 0.4045 * cbMean - 0.7949 * crMean;
	double blueMean = 1.1636 * yMean + 1.9912 * cbMean - 0.0250 * crMean;

	/* Estimate the red and blue gains to apply in a grey world. */
	double redGain = greenMean / (redMean + 1);
	double blueGain = greenMean / (blueMean + 1);

	/* Filter the values to avoid oscillations. */
	double speed = 0.2;
	redGain = speed * redGain + (1 - speed) * frameContext.awb.gains.red;
	blueGain = speed * blueGain + (1 - speed) * frameContext.awb.gains.blue;

	/*
	 * Gain values are unsigned integer value, range 0 to 4 with 8 bit
	 * fractional part.
	 */
	frameContext.awb.gains.red = std::clamp(redGain, 0.0, 1023.0 / 256);
	frameContext.awb.gains.blue = std::clamp(blueGain, 0.0, 1023.0 / 256);
	/* Hardcode the green gain to 1.0. */
	frameContext.awb.gains.green = 1.0;

	frameContext.awb.temperatureK = estimateCCT(redMean, greenMean, blueMean);

	LOG(RkISP1Awb, Debug) << "Gain found for red: " << context.frameContext.awb.gains.red
			      << " and for blue: " << context.frameContext.awb.gains.blue;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
