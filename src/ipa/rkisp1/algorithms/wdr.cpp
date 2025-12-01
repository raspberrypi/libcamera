/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * RkISP1 Wide Dynamic Range control
 */

#include "wdr.h"

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include <libipa/agc_mean_luminance.h>
#include <libipa/histogram.h>
#include <libipa/pwl.h>

#include "linux/rkisp1-config.h"

/**
 * \file wdr.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class WideDynamicRange
 * \brief RkISP1 Wide Dynamic Range algorithm
 *
 * This algorithm implements automatic global tone mapping for the RkISP1.
 * Global tone mapping is done by the GWDR hardware block and applies
 * a global tone mapping curve to the image to increase the perceived dynamic
 * range. Imagine an indoor scene with bright outside visible through the
 * windows. With normal exposure settings, the windows will be completely
 * saturated and no structure (sky/clouds) will be visible because the AEGC has
 * to increase overall exposure to reach a certain level of mean brightness. In
 * WDR mode, the algorithm will artifically reduce the exposure time so that the
 * texture and colours become visible in the formerly saturated areas. Then the
 * global tone mapping curve is applied to mitigate the loss of brightness.
 *
 * Calculating that tone mapping curve is the most difficult part. This
 * algorithm implements four tone mapping strategies:
 * - Linear: The tone mapping curve is a combination of two linear functions
 *   with one kneepoint
 * - Power: The tone mapping curve follows a power function
 * - Exponential: The tone mapping curve follows an exponential function
 * - HistogramEqualization: The tone mapping curve tries to equalize the
 *   histogram
 *
 * The overall strategy is the same in all cases: Add a constraint to the AEGC
 * regulation so that the number of nearly saturated pixels goes below a given
 * threshold (default 2%). This threshold can either be specified in the tuning
 * file or set via the WdrMaxBrightPixels control.
 *
 * The global tone mapping curve is then calculated so that it accounts for the
 * reduction of brightness due to the exposure constraint. We'll call this the
 * WDR-gain. As the result of tone mapping is very difficult to quantize and is
 * by definition a lossy process there is not a single "correct" solution on how
 * this curve should look like.
 *
 * The approach taken here is based on a simple linear model. Consider a pixel
 * that was originally 50% grey. It will have its exposure pushed down by the
 * WDR's initial exposure compensation. This value then needs to be pushed back
 * up by the tone mapping curve so that it is 50% grey again. This point serves
 * as our kneepoint. To get to this kneepoint, this pixel and all darker pixels
 * (to the left of the kneepoint on the tone mapping curve) will simply have the
 * exposure compensation undone by WDR-gain. This cancels out the
 * original exposure compensation, which was 1/WDR-gain. The remaining
 * brigher pixels (to the right of the kneepoint on the tone mapping curve) will
 * be compressed. The WdrStrength control adjusts the gain of the left part of
 * the tone mapping curve.
 *
 * In the Power and Exponential modes, the curves are calculated so that they
 * pass through that kneepoint.
 *
 * The histogram equalization mode tries to equalize the histogram of the
 * image and acts independently of the calculated exposure value.
 *
 * \code{.unparsed}
 * algorithms:
 *   - WideDynamicRange:
 *       ExposureConstraint:
 *         MaxBrightPixels: 0.02
 *         yTarget: 0.95
 * \endcode
 */

LOG_DEFINE_CATEGORY(RkISP1Wdr)

static constexpr unsigned int kTonecurveXIntervals = RKISP1_CIF_ISP_WDR_CURVE_NUM_INTERV;

/*
 * Increasing interval sizes. The intervals are crafted so that they sum
 * up to 4096. This results in better fitting curves than the constant intervals
 * (all entries are 4)
 */
static constexpr std::array<int, kTonecurveXIntervals> kLoglikeIntervals = {
	{ 0, 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4,
	  4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6 }
};

WideDynamicRange::WideDynamicRange()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int WideDynamicRange::init([[maybe_unused]] IPAContext &context,
			   [[maybe_unused]] const YamlObject &tuningData)
{
	if (!(context.hw.supportedBlocks & 1 << RKISP1_EXT_PARAMS_BLOCK_TYPE_WDR)) {
		LOG(RkISP1Wdr, Error)
			<< "Wide Dynamic Range not supported by the hardware or kernel.";
		return -ENOTSUP;
	}

	toneCurveIntervalValues_ = kLoglikeIntervals;

	/* Calculate a list of normed x values */
	toneCurveX_[0] = 0.0;
	int lastValue = 0;
	for (unsigned int i = 1; i < toneCurveX_.size(); i++) {
		lastValue += std::pow(2, toneCurveIntervalValues_[i - 1] + 3);
		lastValue = std::min(lastValue, 4096);
		toneCurveX_[i] = lastValue / 4096.0;
	}

	exposureConstraintMaxBrightPixels_ = 0.02;
	exposureConstraintY_ = 0.95;

	const auto &constraint = tuningData["ExposureConstraint"];
	if (!constraint.isDictionary()) {
		LOG(RkISP1Wdr, Warning)
			<< "ExposureConstraint not found in tuning data."
			   "Using default values MaxBrightPixels: "
			<< exposureConstraintMaxBrightPixels_
			<< " yTarget: " << exposureConstraintY_;
	} else {
		exposureConstraintMaxBrightPixels_ =
			constraint["MaxBrightPixels"]
				.get<double>()
				.value_or(exposureConstraintMaxBrightPixels_);
		exposureConstraintY_ =
			constraint["yTarget"]
				.get<double>()
				.value_or(exposureConstraintY_);
	}

	context.ctrlMap[&controls::WdrMode] =
		ControlInfo(controls::WdrModeValues, controls::WdrOff);
	context.ctrlMap[&controls::WdrStrength] =
		ControlInfo(0.0f, 2.0f, 1.0f);
	context.ctrlMap[&controls::WdrMaxBrightPixels] =
		ControlInfo(0.0f, 1.0f, static_cast<float>(exposureConstraintMaxBrightPixels_));

	applyCompensationLinear(1.0, 0.0);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int WideDynamicRange::configure(IPAContext &context,
				[[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	context.activeState.wdr.mode = controls::WdrOff;
	context.activeState.wdr.gain = 1.0;
	context.activeState.wdr.strength = 1.0;
	auto &constraint = context.activeState.wdr.constraint;
	constraint.bound = AgcMeanLuminance::AgcConstraint::Bound::Upper;
	constraint.qHi = 1.0;
	constraint.qLo = 1.0 - exposureConstraintMaxBrightPixels_;
	constraint.yTarget.clear();
	constraint.yTarget.append(0, exposureConstraintY_);
	return 0;
}

void WideDynamicRange::applyHistogramEqualization(double strength)
{
	if (hist_.empty())
		return;

	/*
	 * Apply a factor on strength, so that it roughly matches the optical
	 * impression that is produced by the other algorithms. The goal is that
	 * the user can switch algorithms for different looks but similar
	 * "strength".
	 */
	strength *= 0.65;

	/*
	 * In a fully equalized histogram, all bins have the same value. Try
	 * to equalize the histogram by applying a gain or damping depending on
	 * the distance of the actual bin value from that norm.
	 */
	std::vector<double> gains;
	gains.resize(hist_.size());
	double sum = 0;
	double norm = 1.0 / (gains.size());
	for (unsigned i = 0; i < hist_.size(); i++) {
		double diff = 1.0 + strength * (hist_[i] - norm) / norm;
		gains[i] = diff;
		sum += diff;
	}

	/* Never amplify the last entry. */
	gains.back() = std::max(gains.back(), 1.0);

	double scale = gains.size() / sum;
	for (auto &v : gains)
		v *= scale;

	Pwl pwl;
	double step = 1.0 / gains.size();
	double lastX = 0;
	double lastY = 0;

	pwl.append(lastX, lastY);
	for (unsigned int i = 0; i < gains.size() - 1; i++) {
		lastY += gains[i] * step;
		lastX += step;
		pwl.append(lastX, lastY);
	}
	pwl.append(1.0, 1.0);

	for (unsigned int i = 0; i < toneCurveX_.size(); i++)
		toneCurveY_[i] = pwl.eval(toneCurveX_[i]);
}

Vector<double, 2> WideDynamicRange::kneePoint(double gain, double strength)
{
	gain = std::pow(gain, strength);
	double y = 0.5;
	double x = y / gain;

	return { { x, y } };
}

void WideDynamicRange::applyCompensationLinear(double gain, double strength)
{
	auto kp = kneePoint(gain, strength);
	double g1 = kp.y() / kp.x();
	double g2 = (kp.y() - 1) / (kp.x() - 1);

	for (unsigned int i = 0; i < toneCurveX_.size(); i++) {
		double x = toneCurveX_[i];
		double y;
		if (x <= kp.x()) {
			y = g1 * x;
		} else {
			y = g2 * x + 1 - g2;
		}
		toneCurveY_[i] = y;
	}
}

void WideDynamicRange::applyCompensationPower(double gain, double strength)
{
	double e = 1.0;
	if (strength > 1e-6) {
		auto kp = kneePoint(gain, strength);
		/* Calculate an exponent to go through the knee point. */
		e = log(kp.y()) / log(kp.x());
	}

	/*
	 * The power function tends to be extremely steep at the beginning. This
	 * leads to noise and image artifacts in the dark areas. To mitigate
	 * that, we add a short linear section at the beginning of the curve.
	 * The connection between linear and power is the point where the linear
	 * section reaches the y level yLin. The power curve is then scaled so
	 * that it starts at the connection point with the steepness it would
	 * have at y=yLin but still goes through 1,1
	 */
	double yLin = 0.1;
	/* x position of the connection point */
	double xb = yLin / gain;
	/* x offset for the scaled power function */
	double q = xb - std::exp(std::log(yLin) / e);

	for (unsigned int i = 0; i < toneCurveX_.size(); i++) {
		double x = toneCurveX_[i];
		if (x < xb) {
			toneCurveY_[i] = x * gain;
		} else {
			x = (x - q) / (1 - q);
			toneCurveY_[i] = std::pow(x, e);
		}
	}
}

void WideDynamicRange::applyCompensationExponential(double gain, double strength)
{
	double k = 0.1;
	auto kp = kneePoint(gain, strength);
	double kx = kp.x();
	double ky = kp.y();

	if (kx > ky) {
		LOG(RkISP1Wdr, Warning) << "Invalid knee point: " << kp;
		kx = ky;
	}

	/*
	 * The exponential curve is based on the function proposed by Glozman
	 * et al. in
	 * S. Glozman, T. Kats, and O. Yadid-Pecht, "Exponent Operator Based
	 * Tone Mapping Algorithm for Color Wide Dynamic Range Images," 2011.
	 *
	 * That function uses a k factor as parameter for the WDR compression
	 * curve:
	 * k=0: maximum compression
	 * k=infinity: linear curve
	 *
	 * To calculate a k factor that results in a curve that passes through
	 * the kneepoint, the equation needs to be solved for k after inserting
	 * the kneepoint.  This can be formulated as search for a zero point.
	 * Unfortunately there is no closed solution for that transformation.
	 * Using newton's method to approximate the value is numerically
	 * unstable.
	 *
	 * Luckily the function only crosses the x axis once and for the set of
	 * possible kneepoints, a negative and a positive point can be guessed.
	 * The approximation is then implemented using bisection.
	 */
	if (std::abs(kx - ky) < 0.001) {
		k = 1e8;
	} else {
		double kl = 0.0001;
		double kh = 1000;

		auto fk = [=](double v) {
			return std::exp(-kx / v) -
			       ky * std::exp(-1.0 / v) + ky - 1.0;
		};

		ASSERT(fk(kl) < 0);
		ASSERT(fk(kh) > 0);

		k = kh / 10.0;
		while (fk(k) > 0) {
			kh = k;
			k /= 10.0;
		}

		do {
			k = (kl + kh) / 2;
			if (fk(k) < 0)
				kl = k;
			else
				kh = k;
		} while (std::abs(kh - kl) > 1e-3);
	}

	double a = 1.0 / (1.0 - std::exp(-1.0 / k));
	for (unsigned int i = 0; i < toneCurveX_.size(); i++)
		toneCurveY_[i] = a * (1.0 - std::exp(-toneCurveX_[i] / k));
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void WideDynamicRange::queueRequest([[maybe_unused]] IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    IPAFrameContext &frameContext,
				    const ControlList &controls)
{
	auto &activeState = context.activeState;

	const auto &mode = controls.get(controls::WdrMode);
	if (mode)
		activeState.wdr.mode = static_cast<controls::WdrModeEnum>(*mode);

	const auto &brightPixels = controls.get(controls::WdrMaxBrightPixels);
	if (brightPixels)
		activeState.wdr.constraint.qLo = 1.0 - *brightPixels;

	const auto &strength = controls.get(controls::WdrStrength);
	if (strength)
		activeState.wdr.strength = *strength;

	frameContext.wdr.mode = activeState.wdr.mode;
	frameContext.wdr.strength = activeState.wdr.strength;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void WideDynamicRange::prepare(IPAContext &context,
			       [[maybe_unused]] const uint32_t frame,
			       IPAFrameContext &frameContext,
			       RkISP1Params *params)
{
	if (!params) {
		LOG(RkISP1Wdr, Warning) << "Params is null";
		return;
	}

	auto mode = frameContext.wdr.mode;

	auto config = params->block<BlockType::Wdr>();
	config.setEnabled(mode != controls::WdrOff);

	/* Calculate how much EV we need to compensate with the WDR curve. */
	double gain = context.activeState.wdr.gain;
	frameContext.wdr.gain = gain;

	if (mode == controls::WdrOff) {
		applyCompensationLinear(1.0, 0.0);
	} else if (mode == controls::WdrLinear) {
		applyCompensationLinear(gain, frameContext.wdr.strength);
	} else if (mode == controls::WdrPower) {
		applyCompensationPower(gain, frameContext.wdr.strength);
	} else if (mode == controls::WdrExponential) {
		applyCompensationExponential(gain, frameContext.wdr.strength);
	} else if (mode == controls::WdrHistogramEqualization) {
		applyHistogramEqualization(frameContext.wdr.strength);
	}

	/* Reset value */
	config->dmin_strength = 0x10;
	config->dmin_thresh = 0;

	for (unsigned int i = 0; i < kTonecurveXIntervals; i++) {
		int v = toneCurveIntervalValues_[i];
		config->tone_curve.dY[i / 8] |= (v & 0x07) << ((i % 8) * 4);
	}

	/*
	 * Fix the curve to adhere to the hardware constraints. Don't apply a
	 * constraint on the first element, which is most likely zero anyways.
	 */
	int lastY = toneCurveY_[0] * 4096.0;
	for (unsigned int i = 0; i < toneCurveX_.size(); i++) {
		int diff = static_cast<int>(toneCurveY_[i] * 4096.0) - lastY;
		diff = std::clamp(diff, -2048, 2048);
		lastY = lastY + diff;
		config->tone_curve.ym[i] = lastY;
	}
}

void WideDynamicRange::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
			       IPAFrameContext &frameContext,
			       const rkisp1_stat_buffer *stats,
			       ControlList &metadata)
{
	if (!stats || !(stats->meas_type & RKISP1_CIF_ISP_STAT_HIST)) {
		LOG(RkISP1Wdr, Warning) << "No histogram data in statistics";
		return;
	}

	const rkisp1_cif_isp_stat *params = &stats->params;
	auto mode = frameContext.wdr.mode;

	metadata.set(controls::WdrMode, mode);

	Histogram cumHist({ params->hist.hist_bins, context.hw.numHistogramBins },
			  [](uint32_t x) { return x >> 4; });

	/* Calculate the gain needed to reach the requested yTarget. */
	double value = cumHist.interQuantileMean(0, 1.0) / cumHist.bins();
	double gain = context.activeState.agc.automatic.yTarget / value;
	gain = std::max(gain, 1.0);

	double speed = 0.2;
	gain = gain * speed + context.activeState.wdr.gain * (1.0 - speed);

	context.activeState.wdr.gain = gain;

	std::vector<double> hist;
	double sum = 0;
	for (unsigned i = 0; i < context.hw.numHistogramBins; i++) {
		double v = params->hist.hist_bins[i] >> 4;
		hist.push_back(v);
		sum += v;
	}

	/* Scale so that the entries sum up to 1. */
	double scale = 1.0 / sum;
	for (auto &v : hist)
		v *= scale;
	hist_.swap(hist);
}

REGISTER_IPA_ALGORITHM(WideDynamicRange, "WideDynamicRange")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
