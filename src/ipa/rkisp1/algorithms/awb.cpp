/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * AWB control algorithm
 */

#include "awb.h"

#include <algorithm>
#include <ios>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/awb_bayes.h"
#include "libipa/awb_grey.h"
#include "libipa/colours.h"

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

constexpr int32_t kMinColourTemperature = 2500;
constexpr int32_t kMaxColourTemperature = 10000;
constexpr int32_t kDefaultColourTemperature = 5000;

/* Minimum mean value below which AWB can't operate. */
constexpr double kMeanMinThreshold = 2.0;

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

private:
	RGB<double> rgbMeans_;
	double rg_;
	double bg_;
};

Awb::Awb()
	: rgbMode_(false)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Awb::init(IPAContext &context, const YamlObject &tuningData)
{
	auto &cmap = context.ctrlMap;
	cmap[&controls::ColourTemperature] = ControlInfo(kMinColourTemperature,
							 kMaxColourTemperature,
							 kDefaultColourTemperature);
	cmap[&controls::AwbEnable] = ControlInfo(false, true);
	cmap[&controls::ColourGains] = ControlInfo(0.0f, 3.996f, 1.0f);

	if (!tuningData.contains("algorithm"))
		LOG(RkISP1Awb, Info) << "No AWB algorithm specified."
				     << " Default to grey world";

	auto mode = tuningData["algorithm"].get<std::string>("grey");
	if (mode == "grey") {
		awbAlgo_ = std::make_unique<AwbGrey>();
	} else if (mode == "bayes") {
		awbAlgo_ = std::make_unique<AwbBayes>();
	} else {
		LOG(RkISP1Awb, Error) << "Unknown AWB algorithm: " << mode;
		return -EINVAL;
	}
	LOG(RkISP1Awb, Debug) << "Using AWB algorithm: " << mode;

	int ret = awbAlgo_->init(tuningData);
	if (ret) {
		LOG(RkISP1Awb, Error) << "Failed to init AWB algorithm";
		return ret;
	}

	const auto &src = awbAlgo_->controls();
	cmap.insert(src.begin(), src.end());

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   const IPACameraSensorInfo &configInfo)
{
	context.activeState.awb.manual.gains = RGB<double>{ 1.0 };
	auto gains = awbAlgo_->gainsFromColourTemperature(kDefaultColourTemperature);
	if (gains)
		context.activeState.awb.automatic.gains = *gains;
	else
		context.activeState.awb.automatic.gains = RGB<double>{ 1.0 };

	context.activeState.awb.autoEnabled = true;
	context.activeState.awb.manual.temperatureK = kDefaultColourTemperature;
	context.activeState.awb.automatic.temperatureK = kDefaultColourTemperature;

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

	awbAlgo_->handleControls(controls);

	frameContext.awb.autoEnabled = awb.autoEnabled;

	if (awb.autoEnabled)
		return;

	const auto &colourGains = controls.get(controls::ColourGains);
	const auto &colourTemperature = controls.get(controls::ColourTemperature);
	bool update = false;
	if (colourGains) {
		awb.manual.gains.r() = (*colourGains)[0];
		awb.manual.gains.b() = (*colourGains)[1];
		/*
		 * \todo Colour temperature reported in metadata is now
		 * incorrect, as we can't deduce the temperature from the gains.
		 * This will be fixed with the bayes AWB algorithm.
		 */
		update = true;
	} else if (colourTemperature) {
		awb.manual.temperatureK = *colourTemperature;
		const auto &gains = awbAlgo_->gainsFromColourTemperature(*colourTemperature);
		if (gains) {
			awb.manual.gains.r() = gains->r();
			awb.manual.gains.b() = gains->b();
			update = true;
		}
	}

	if (update)
		LOG(RkISP1Awb, Debug)
			<< "Set colour gains to " << awb.manual.gains;

	frameContext.awb.gains = awb.manual.gains;
	frameContext.awb.temperatureK = awb.manual.temperatureK;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, RkISP1Params *params)
{
	/*
	 * This is the latest time we can read the active state. This is the
	 * most up-to-date automatic values we can read.
	 */
	if (frameContext.awb.autoEnabled) {
		const auto &awb = context.activeState.awb;
		frameContext.awb.gains = awb.automatic.gains;
		frameContext.awb.temperatureK = awb.automatic.temperatureK;
	}

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
	IPAActiveState &activeState = context.activeState;

	metadata.set(controls::AwbEnable, frameContext.awb.autoEnabled);
	metadata.set(controls::ColourGains, {
			static_cast<float>(frameContext.awb.gains.r()),
			static_cast<float>(frameContext.awb.gains.b())
		});
	metadata.set(controls::ColourTemperature, frameContext.awb.temperatureK);

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

	RGB<double> rgbMeans = calculateRgbMeans(frameContext, awb);

	/*
	 * If the means are too small we don't have enough information to
	 * meaningfully calculate gains. Freeze the algorithm in that case.
	 */
	if (rgbMeans.r() < kMeanMinThreshold && rgbMeans.g() < kMeanMinThreshold &&
	    rgbMeans.b() < kMeanMinThreshold)
		return;

	RkISP1AwbStats awbStats{ rgbMeans };
	AwbResult awbResult = awbAlgo_->calculateAwb(awbStats, frameContext.lux.lux);

	/*
	 * Clamp the gain values to the hardware, which expresses gains as Q2.8
	 * unsigned integer values. Set the minimum just above zero to avoid
	 * divisions by zero when computing the raw means in subsequent
	 * iterations.
	 */
	awbResult.gains = awbResult.gains.max(1.0 / 256).min(1023.0 / 256);

	/* Filter the values to avoid oscillations. */
	double speed = 0.2;
	double ct = awbResult.colourTemperature;
	ct = ct * speed + activeState.awb.automatic.temperatureK * (1 - speed);
	awbResult.gains = awbResult.gains * speed +
			  activeState.awb.automatic.gains * (1 - speed);

	activeState.awb.automatic.temperatureK = static_cast<unsigned int>(ct);
	activeState.awb.automatic.gains = awbResult.gains;

	LOG(RkISP1Awb, Debug)
		<< std::showpoint
		<< "Means " << rgbMeans << ", gains "
		<< activeState.awb.automatic.gains << ", temp "
		<< activeState.awb.automatic.temperatureK << "K";
}

RGB<double> Awb::calculateRgbMeans(const IPAFrameContext &frameContext, const rkisp1_cif_isp_awb_stat *awb) const
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

	return rgbMeans;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
