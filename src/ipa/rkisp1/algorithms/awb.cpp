/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * AWB control algorithm
 */

#include "awb.h"

#include <algorithm>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/vector.h"

/**
 * \file awb.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

LOG_DEFINE_CATEGORY(RkISP1Awb)

class RkISP1AwbStats final : public AwbStats
{
public:
	RkISP1AwbStats(const RGB<double> &rgbMeans)
		: rgbMeans_(rgbMeans)
	{
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
		constexpr double minValue = 2.0;

		return rgbMeans_.r() > minValue || rgbMeans_.g() > minValue ||
		       rgbMeans_.b() > minValue;
	}

private:
	RGB<double> rgbMeans_;
	double rg_;
	double bg_;
};

/**
 * \class Awb
 * \brief Manage the white balance with automatic and manual controls
 */

Awb::Awb()
	: rgbMode_(false)
{
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
		   const IPACameraSensorInfo &configInfo)
{
	awbAlgo_.configure(context.activeState.awb);

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
void Awb::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, RkISP1Params *params)
{
	awbAlgo_.prepare(context.activeState.awb, frameContext.awb);

	auto gainConfig = params->block<BlockType::AwbGain>();
	gainConfig.setEnabled(true);

	gainConfig->gain_green_b = std::clamp<int>(256 * frameContext.awb.gains.g(), 0, 0x3ff);
	gainConfig->gain_blue = std::clamp<int>(256 * frameContext.awb.gains.b(), 0, 0x3ff);
	gainConfig->gain_red = std::clamp<int>(256 * frameContext.awb.gains.r(), 0, 0x3ff);
	gainConfig->gain_green_r = std::clamp<int>(256 * frameContext.awb.gains.g(), 0, 0x3ff);

	/* If we have already set the AWB measurement parameters, return. */
	if (frame > 0)
		return;

	auto awbConfig = params->block<BlockType::Awb>();
	awbConfig.setEnabled(true);

	/* Configure the measure window for AWB. */
	awbConfig->awb_wnd = context.configuration.awb.measureWindow;

	/* Number of frames to use to estimate the means (0 means 1 frame). */
	awbConfig->frames = 0;

	/* Select RGB or YCbCr means measurement. */
	if (rgbMode_) {
		awbConfig->awb_mode = RKISP1_CIF_ISP_AWB_MODE_RGB;

		/*
		 * For RGB-based measurements, pixels are selected with maximum
		 * red, green and blue thresholds that are set in the
		 * awb_ref_cr, awb_min_y and awb_ref_cb respectively. The other
		 * values are not used, set them to 0.
		 */
		awbConfig->awb_ref_cr = 250;
		awbConfig->min_y = 250;
		awbConfig->awb_ref_cb = 250;

		awbConfig->max_y = 0;
		awbConfig->min_c = 0;
		awbConfig->max_csum = 0;
	} else {
		awbConfig->awb_mode = RKISP1_CIF_ISP_AWB_MODE_YCBCR;

		/* Set the reference Cr and Cb (AWB target) to white. */
		awbConfig->awb_ref_cb = 128;
		awbConfig->awb_ref_cr = 128;

		/*
		 * Filter out pixels based on luminance and chrominance values.
		 * The acceptable luma values are specified as a [16, 250]
		 * range, while the acceptable chroma values are specified with
		 * a minimum of 16 and a maximum Cb+Cr sum of 250.
		 */
		awbConfig->min_y = 16;
		awbConfig->max_y = 250;
		awbConfig->min_c = 16;
		awbConfig->max_csum = 250;
	}
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
	if (!stats || !(stats->meas_type & RKISP1_CIF_ISP_STAT_AWB)) {
		LOG(RkISP1Awb, Error) << "AWB data is missing in statistics";
		return;
	}

	const rkisp1_cif_isp_stat *params = &stats->params;
	const rkisp1_cif_isp_awb_stat *awb = &params->awb;

	if (awb->awb_mean[0].cnt == 0) {
		LOG(RkISP1Awb, Debug) << "AWB statistics are empty";
		return;
	}

	RkISP1AwbStats awbStats = calculateRgbMeans(frameContext, awb);

	awbAlgo_.process(context.activeState.awb, frameContext.awb, awbStats,
			 frameContext.lux.lux, metadata);
}

RkISP1AwbStats Awb::calculateRgbMeans(const IPAFrameContext &frameContext,
				      const rkisp1_cif_isp_awb_stat *awb) const
{
	Vector<double, 3> rgbMeans;

	if (rgbMode_) {
		rgbMeans = {{
			static_cast<double>(awb->awb_mean[0].mean_cr_or_r),
			static_cast<double>(awb->awb_mean[0].mean_y_or_g),
			static_cast<double>(awb->awb_mean[0].mean_cb_or_b)
		}};
	} else {
		/* Get the YCbCr mean values */
		Vector<double, 3> yuvMeans({
			static_cast<double>(awb->awb_mean[0].mean_y_or_g),
			static_cast<double>(awb->awb_mean[0].mean_cb_or_b),
			static_cast<double>(awb->awb_mean[0].mean_cr_or_r)
		});

		/*
		 * Convert from YCbCr to RGB. The hardware uses the following
		 * formulas:
		 *
		 * Y  =  16 + 0.2500 R + 0.5000 G + 0.1094 B
		 * Cb = 128 - 0.1406 R - 0.2969 G + 0.4375 B
		 * Cr = 128 + 0.4375 R - 0.3750 G - 0.0625 B
		 *
		 * This seems to be based on limited range BT.601 with Q1.6
		 * precision.
		 *
		 * The inverse matrix is:
		 *
		 * [[1,1636, -0,0623,  1,6008]
		 *  [1,1636, -0,4045, -0,7949]
		 *  [1,1636,  1,9912, -0,0250]]
		 */
		static const Matrix<double, 3, 3> yuv2rgbMatrix({
			1.1636, -0.0623,  1.6008,
			1.1636, -0.4045, -0.7949,
			1.1636,  1.9912, -0.0250
		});
		static const Vector<double, 3> yuv2rgbOffset({
			16, 128, 128
		});

		rgbMeans = yuv2rgbMatrix * (yuvMeans - yuv2rgbOffset);

		/*
		 * Due to hardware rounding errors in the YCbCr means, the
		 * calculated RGB means may be negative. This would lead to
		 * negative gains, messing up calculation. Prevent this by
		 * clamping the means to positive values.
		 */
		rgbMeans = rgbMeans.max(0.0);
	}

	/*
	 * The ISP computes the AWB means after applying the CCM. Apply the
	 * inverse as we want to get the raw means before the colour gains.
	 */
	rgbMeans = frameContext.ccm.ccm.inverse() * rgbMeans;

	/*
	 * The ISP computes the AWB means after applying the colour gains,
	 * divide by the gains that were used to get the raw means from the
	 * sensor. Apply a minimum value to avoid divisions by near-zero.
	 */
	rgbMeans /= frameContext.awb.gains.max(0.01);

	return RkISP1AwbStats(rgbMeans);
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
