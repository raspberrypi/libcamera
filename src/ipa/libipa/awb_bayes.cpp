/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Implementation of a bayesian AWB algorithm
 */

#include "awb_bayes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <ostream>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>

#include "colours.h"

/**
 * \file awb_bayes.h
 * \brief Implementation of bayesian auto white balance algorithm
 *
 * This implementation is based on the initial implementation done by
 * RaspberryPi.
 *
 * \todo Documentation
 *
 * \todo Not all the features implemented by RaspberryPi were ported over to
 * this algorithm because they either rely on hardware features not generally
 * available or were considered not important enough at the moment.
 *
 * The following parts are not implemented:
 *
 * - min_pixels: minimum proportion of pixels counted within AWB region for it
 *   to be "useful"
 * - min_g: minimum G value of those pixels, to be regarded a "useful"
 * - min_regions: number of AWB regions that must be "useful" in order to do the
 *   AWB calculation
 * - deltaLimit: clamp on colour error term (so as not to penalize non-grey
 *   excessively)
 * - bias_proportion: The biasProportion parameter adds a small proportion of
 *   the counted pixels to a region biased to the biasCT colour temperature.
 *   A typical value for biasProportion would be between 0.05 to 0.1.
 * - bias_ct: CT target for the search bias
 * - sensitivityR: red sensitivity ratio (set to canonical sensor's R/G divided
 *   by this sensor's R/G)
 * - sensitivityB: blue sensitivity ratio (set to canonical sensor's B/G divided
 *   by this sensor's B/G)
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(Awb)

namespace {

template<typename T>
class LimitsRecorder
{
public:
	LimitsRecorder()
		: min_(std::numeric_limits<T>::max()),
		  max_(std::numeric_limits<T>::min())
	{
	}

	void record(const T &value)
	{
		min_ = std::min(min_, value);
		max_ = std::max(max_, value);
	}

	const T &min() const { return min_; }
	const T &max() const { return max_; }

private:
	T min_;
	T max_;
};

#ifndef __DOXYGEN__
template<typename T>
std::ostream &operator<<(std::ostream &out, const LimitsRecorder<T> &v)
{
	out << "[ " << v.min() << ", " << v.max() << " ]";
	return out;
}
#endif

} /* namespace */

namespace ipa {

/**
 * \brief Step size control for CT search
 */
constexpr double kSearchStep = 0.2;

/**
 * \copydoc libcamera::ipa::Interpolator::interpolate()
 */
template<>
void Interpolator<Pwl>::interpolate(const Pwl &a, const Pwl &b, Pwl &dest, double lambda)
{
	dest = Pwl::combine(a, b,
			    [=](double /*x*/, double y0, double y1) -> double {
				    return y0 * (1.0 - lambda) + y1 * lambda;
			    });
}

/**
 * \class AwbBayes
 * \brief Implementation of a bayesian auto white balance algorithm
 *
 * In a bayesian AWB algorithm the auto white balance estimation is improved by
 * taking the likelihood of a given lightsource based on the estimated lux level
 * into account. E.g. If it is very bright we can assume that we are outside and
 * that colour temperatures around 6500 are preferred.
 *
 * The second part of this algorithm is the search for the most likely colour
 * temperature. It is implemented in AwbBayes::coarseSearch() and in
 * AwbBayes::fineSearch(). The search works very well without prior likelihoods
 * and therefore the algorithm itself provides very good results even without
 * prior likelihoods.
 */

/**
 * \var AwbBayes::transversePos_
 * \brief How far to wander off CT curve towards "more purple"
 */

/**
 * \var AwbBayes::transverseNeg_
 * \brief How far to wander off CT curve towards "more green"
 */

/**
 * \var AwbBayes::currentMode_
 * \brief The currently selected mode
 */

int AwbBayes::init(const YamlObject &tuningData)
{
	int ret = colourGainCurve_.readYaml(tuningData["colourGains"], "ct", "gains");
	if (ret) {
		LOG(Awb, Error)
			<< "Failed to parse 'colourGains' "
			<< "parameter from tuning file";
		return ret;
	}

	ctR_.clear();
	ctB_.clear();
	for (const auto &[ct, g] : colourGainCurve_.data()) {
		ctR_.append(ct, 1.0 / g[0]);
		ctB_.append(ct, 1.0 / g[1]);
	}

	/* We will want the inverse functions of these too. */
	ctRInverse_ = ctR_.inverse().first;
	ctBInverse_ = ctB_.inverse().first;

	ret = readPriors(tuningData);
	if (ret) {
		LOG(Awb, Error) << "Failed to read priors";
		return ret;
	}

	ret = parseModeConfigs(tuningData, controls::AwbAuto);
	if (ret) {
		LOG(Awb, Error)
			<< "Failed to parse mode parameter from tuning file";
		return ret;
	}
	currentMode_ = &modes_[controls::AwbAuto];

	transversePos_ = tuningData["transversePos"].get<double>(0.01);
	transverseNeg_ = tuningData["transverseNeg"].get<double>(0.01);
	if (transversePos_ <= 0 || transverseNeg_ <= 0) {
		LOG(Awb, Error) << "AwbConfig: transversePos/Neg must be > 0";
		return -EINVAL;
	}

	return 0;
}

int AwbBayes::readPriors(const YamlObject &tuningData)
{
	const auto &priorsList = tuningData["priors"];
	std::map<uint32_t, Pwl> priors;

	if (!priorsList) {
		LOG(Awb, Error) << "No priors specified";
		return -EINVAL;
	}

	for (const auto &p : priorsList.asList()) {
		if (!p.contains("lux")) {
			LOG(Awb, Error) << "Missing lux value";
			return -EINVAL;
		}

		uint32_t lux = p["lux"].get<uint32_t>(0);
		if (priors.count(lux)) {
			LOG(Awb, Error) << "Duplicate prior for lux value " << lux;
			return -EINVAL;
		}

		std::vector<uint32_t> temperatures =
			p["ct"].getList<uint32_t>().value_or(std::vector<uint32_t>{});
		std::vector<double> probabilities =
			p["probability"].getList<double>().value_or(std::vector<double>{});

		if (temperatures.size() != probabilities.size()) {
			LOG(Awb, Error)
				<< "Ct and probability array sizes are unequal";
			return -EINVAL;
		}

		if (temperatures.empty()) {
			LOG(Awb, Error)
				<< "Ct and probability arrays are empty";
			return -EINVAL;
		}

		std::map<int, double> ctToProbability;
		for (unsigned int i = 0; i < temperatures.size(); i++) {
			int t = temperatures[i];
			if (ctToProbability.count(t)) {
				LOG(Awb, Error) << "Duplicate ct value";
				return -EINVAL;
			}

			ctToProbability[t] = probabilities[i];
		}

		auto &pwl = priors[lux];
		for (const auto &[ct, prob] : ctToProbability) {
			if (prob < 1e-6) {
				LOG(Awb, Error) << "Prior probability must be larger than 1e-6";
				return -EINVAL;
			}
			pwl.append(ct, prob);
		}
	}

	if (priors.empty()) {
		LOG(Awb, Error) << "No priors specified";
		return -EINVAL;
	}

	priors_.setData(std::move(priors));

	return 0;
}

void AwbBayes::handleControls(const ControlList &controls)
{
	auto mode = controls.get(controls::AwbMode);
	if (mode) {
		auto it = modes_.find(static_cast<controls::AwbModeEnum>(*mode));
		if (it != modes_.end())
			currentMode_ = &it->second;
		else
			LOG(Awb, Error) << "Unsupported AWB mode " << *mode;
	}
}

std::optional<RGB<double>> AwbBayes::gainsFromColourTemperature(double colourTemperature)
{
	/*
	 * \todo In the RaspberryPi code, the ct curve was interpolated in
	 * the white point space (1/x) not in gains space. This feels counter
	 * intuitive, as the gains are in linear space. But I can't prove it.
	 */
	const auto &gains = colourGainCurve_.getInterpolated(colourTemperature);
	return RGB<double>{ { gains[0], 1.0, gains[1] } };
}

AwbResult AwbBayes::calculateAwb(const AwbStats &stats, unsigned int lux)
{
	ipa::Pwl prior;
	if (lux > 0) {
		prior = priors_.getInterpolated(lux);
		prior.map([](double x, double y) {
			LOG(Awb, Debug) << "(" << x << "," << y << ")";
		});
	} else {
		prior.append(0, 1.0);
	}

	double t = coarseSearch(prior, stats);
	double r = ctR_.eval(t);
	double b = ctB_.eval(t);
	LOG(Awb, Debug)
		<< "After coarse search: r " << r << " b " << b << " (gains r "
		<< 1 / r << " b " << 1 / b << ")";

	/*
	 * Original comment from RaspberryPi:
	 * Not entirely sure how to handle the fine search yet. Mostly the
	 * estimated CT is already good enough, but the fine search allows us to
	 * wander transversely off the CT curve. Under some illuminants, where
	 * there may be more or less green light, this may prove beneficial,
	 * though I probably need more real datasets before deciding exactly how
	 * this should be controlled and tuned.
	 */
	fineSearch(t, r, b, prior, stats);
	LOG(Awb, Debug)
		<< "After fine search: r " << r << " b " << b << " (gains r "
		<< 1 / r << " b " << 1 / b << ")";

	return { { { 1.0 / r, 1.0, 1.0 / b } }, t };
}

double AwbBayes::coarseSearch(const ipa::Pwl &prior, const AwbStats &stats) const
{
	std::vector<Pwl::Point> points;
	size_t bestPoint = 0;
	double t = currentMode_->ctLo;
	int spanR = -1;
	int spanB = -1;
	LimitsRecorder<double> errorLimits;
	LimitsRecorder<double> priorLogLikelihoodLimits;

	/* Step down the CT curve evaluating log likelihood. */
	while (true) {
		double r = ctR_.eval(t, &spanR);
		double b = ctB_.eval(t, &spanB);
		RGB<double> gains({ 1 / r, 1.0, 1 / b });
		double delta2Sum = stats.computeColourError(gains);
		double priorLogLikelihood = log(prior.eval(prior.domain().clamp(t)));
		double finalLogLikelihood = delta2Sum - priorLogLikelihood;

		errorLimits.record(delta2Sum);
		priorLogLikelihoodLimits.record(priorLogLikelihood);

		points.push_back({ { t, finalLogLikelihood } });
		if (points.back().y() < points[bestPoint].y())
			bestPoint = points.size() - 1;

		if (t == currentMode_->ctHi)
			break;

		/*
		 * Ensure even steps along the r/b curve by scaling them by the
		 * current t.
		 */
		t = std::min(t + t / 10 * kSearchStep, currentMode_->ctHi);
	}

	t = points[bestPoint].x();
	LOG(Awb, Debug) << "Coarse search found CT " << t
			<< " error limits:" << errorLimits
			<< " prior log likelihood limits:" << priorLogLikelihoodLimits;

	/*
	 * We have the best point of the search, but refine it with a quadratic
	 * interpolation around its neighbors.
	 */
	if (points.size() > 2) {
		bestPoint = std::clamp(bestPoint, std::size_t{ 1 }, points.size() - 2);
		t = interpolateQuadratic(points[bestPoint - 1],
					 points[bestPoint],
					 points[bestPoint + 1]);
		LOG(Awb, Debug)
			<< "After quadratic refinement, coarse search has CT "
			<< t;
	}

	return t;
}

void AwbBayes::fineSearch(double &t, double &r, double &b, ipa::Pwl const &prior, const AwbStats &stats) const
{
	int spanR = -1;
	int spanB = -1;
	double step = t / 10 * kSearchStep * 0.1;
	int nsteps = 5;
	ctR_.eval(t, &spanR);
	ctB_.eval(t, &spanB);
	double rDiff = ctR_.eval(t + nsteps * step, &spanR) -
		       ctR_.eval(t - nsteps * step, &spanR);
	double bDiff = ctB_.eval(t + nsteps * step, &spanB) -
		       ctB_.eval(t - nsteps * step, &spanB);
	Pwl::Point transverse({ bDiff, -rDiff });
	if (transverse.length2() < 1e-6)
		return;
	/*
	 * transverse is a unit vector orthogonal to the b vs. r function
	 * (pointing outwards with r and b increasing)
	 */
	transverse = transverse / transverse.length();
	double bestLogLikelihood = 0;
	double bestT = 0;
	Pwl::Point bestRB(0);
	double transverseRange = transverseNeg_ + transversePos_;
	const int maxNumDeltas = 12;
	LimitsRecorder<double> errorLimits;
	LimitsRecorder<double> priorLogLikelihoodLimits;


	/* a transverse step approximately every 0.01 r/b units */
	int numDeltas = floor(transverseRange * 100 + 0.5) + 1;
	numDeltas = std::clamp(numDeltas, 3, maxNumDeltas);

	/*
	 * Step down CT curve. March a bit further if the transverse range is
	 * large.
	 */
	nsteps += numDeltas;
	for (int i = -nsteps; i <= nsteps; i++) {
		double tTest = t + i * step;
		double priorLogLikelihood =
			log(prior.eval(prior.domain().clamp(tTest)));
		priorLogLikelihoodLimits.record(priorLogLikelihood);
		Pwl::Point rbStart{ { ctR_.eval(tTest, &spanR),
				      ctB_.eval(tTest, &spanB) } };
		Pwl::Point samples[maxNumDeltas];
		int bestPoint = 0;

		/*
		 * Sample numDeltas points transversely *off* the CT curve
		 * in the range [-transverseNeg, transversePos].
		 * The x values of a sample contains the distance and the y
		 * value contains the corresponding log likelihood.
		 */
		double transverseStep = transverseRange / (numDeltas - 1);
		for (int j = 0; j < numDeltas; j++) {
			auto &p = samples[j];
			p.x() = -transverseNeg_ + transverseStep * j;
			Pwl::Point rbTest = rbStart + transverse * p.x();
			RGB<double> gains({ 1 / rbTest[0], 1.0, 1 / rbTest[1] });
			double delta2Sum = stats.computeColourError(gains);
			errorLimits.record(delta2Sum);
			p.y() = delta2Sum - priorLogLikelihood;

			if (p.y() < samples[bestPoint].y())
				bestPoint = j;
		}

		/*
		 * We have all samples transversely across the CT curve,
		 * now let's do a quadratic interpolation for the best result.
		 */
		bestPoint = std::clamp(bestPoint, 1, numDeltas - 2);
		double bestOffset = interpolateQuadratic(samples[bestPoint - 1],
							 samples[bestPoint],
							 samples[bestPoint + 1]);
		Pwl::Point rbTest = rbStart + transverse * bestOffset;
		RGB<double> gains({ 1 / rbTest[0], 1.0, 1 / rbTest[1] });
		double delta2Sum = stats.computeColourError(gains);
		errorLimits.record(delta2Sum);
		double finalLogLikelihood = delta2Sum - priorLogLikelihood;

		if (bestT == 0 || finalLogLikelihood < bestLogLikelihood) {
			bestLogLikelihood = finalLogLikelihood;
			bestT = tTest;
			bestRB = rbTest;
		}
	}

	t = bestT;
	r = bestRB[0];
	b = bestRB[1];
	LOG(Awb, Debug)
		<< "Fine search found t " << t << " r " << r << " b " << b
		<< " error limits: " << errorLimits
		<< " prior log likelihood limits: " << priorLogLikelihoodLimits;
}

/**
 * \brief Find extremum of function
 * \param[in] a First point
 * \param[in] b Second point
 * \param[in] c Third point
 *
 * Given 3 points on a curve, find the extremum of the function in that interval
 * by fitting a quadratic.
 *
 * \return The x value of the extremum clamped to the interval [a.x(), c.x()]
 */
double AwbBayes::interpolateQuadratic(Pwl::Point const &a, Pwl::Point const &b,
				      Pwl::Point const &c) const
{
	const double eps = 1e-3;
	Pwl::Point ca = c - a;
	Pwl::Point ba = b - a;
	double denominator = 2 * (ba.y() * ca.x() - ca.y() * ba.x());
	if (std::abs(denominator) > eps) {
		double numerator = ba.y() * ca.x() * ca.x() - ca.y() * ba.x() * ba.x();
		double result = numerator / denominator + a.x();
		return std::max(a.x(), std::min(c.x(), result));
	}
	/* has degenerated to straight line segment */
	return a.y() < c.y() - eps ? a.x() : (c.y() < a.y() - eps ? c.x() : b.x());
}

} /* namespace ipa */

} /* namespace libcamera */
