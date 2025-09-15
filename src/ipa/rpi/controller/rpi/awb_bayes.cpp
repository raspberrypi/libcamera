/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * AWB control algorithm
 */

#include <assert.h>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

#include <libcamera/base/log.h>

#include <libcamera/geometry.h>

#include "../awb_algorithm.h"
#include "../awb_status.h"
#include "../lux_status.h"
#include "../statistics.h"
#include "libipa/pwl.h"

#include "alsc_status.h"
#include "awb.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(RPiAwb)

constexpr double kDefaultCT = 4500.0;

#define NAME "rpi.awb"

/*
 * todo - the locking in this algorithm needs some tidying up as has been done
 * elsewhere (ALSC and AGC).
 */

namespace RPiController {

struct AwbPrior {
	int read(const libcamera::YamlObject &params);
	double lux; /* lux level */
	libcamera::ipa::Pwl prior; /* maps CT to prior log likelihood for this lux level */
};

struct AwbBayesConfig {
	AwbBayesConfig() {}
	int read(const libcamera::YamlObject &params, AwbConfig &config);
	/* table of illuminant priors at different lux levels */
	std::vector<AwbPrior> priors;
	/*
	 * minimum proportion of pixels counted within AWB region for it to be
	 * "useful"
	 */
	double minPixels;
	/* minimum G value of those pixels, to be regarded a "useful" */
	uint16_t minG;
	/*
	 * number of AWB regions that must be "useful" in order to do the AWB
	 * calculation
	 */
	uint32_t minRegions;
	/* step size control in coarse search */
	double coarseStep;
	/* The whitepoint (which we normally "aim" for) can be moved. */
	double whitepointR;
	double whitepointB;
	bool bayes; /* use Bayesian algorithm */
	/* proportion of counted samples to add for the search bias */
	double biasProportion;
	/* CT target for the search bias */
	double biasCT;
};

class AwbBayes : public Awb
{
public:
	AwbBayes(Controller *controller = NULL);
	~AwbBayes();
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;

protected:
	void prepareStats() override;
	void doAwb() override;

private:
	AwbBayesConfig bayesConfig_;
	void awbBayes();
	libcamera::ipa::Pwl interpolatePrior();
	double coarseSearch(libcamera::ipa::Pwl const &prior);
	void fineSearch(double &t, double &r, double &b, libcamera::ipa::Pwl const &prior);
	std::vector<libcamera::ipa::Pwl::Point> points_;
};

int AwbPrior::read(const libcamera::YamlObject &params)
{
	auto value = params["lux"].get<double>();
	if (!value)
		return -EINVAL;
	lux = *value;

	prior = params["prior"].get<ipa::Pwl>(ipa::Pwl{});
	return prior.empty() ? -EINVAL : 0;
}

int AwbBayesConfig::read(const libcamera::YamlObject &params, AwbConfig &config)
{
	int ret;

	bayes = params["bayes"].get<int>(1);

	if (params.contains("priors")) {
		for (const auto &p : params["priors"].asList()) {
			AwbPrior prior;
			ret = prior.read(p);
			if (ret)
				return ret;
			if (!priors.empty() && prior.lux <= priors.back().lux) {
				LOG(RPiAwb, Error) << "AwbConfig: Prior must be ordered in increasing lux value";
				return -EINVAL;
			}
			priors.push_back(prior);
		}
		if (priors.empty()) {
			LOG(RPiAwb, Error) << "AwbConfig: no AWB priors configured";
			return -EINVAL;
		}
	}

	minPixels = params["min_pixels"].get<double>(16.0);
	minG = params["min_G"].get<uint16_t>(32);
	minRegions = params["min_regions"].get<uint32_t>(10);
	coarseStep = params["coarse_step"].get<double>(0.2);

	if (bayes) {
		if (!config.hasCtCurve() || priors.empty() ||
		    config.defaultMode == nullptr) {
			LOG(RPiAwb, Warning)
				<< "Bayesian AWB mis-configured - switch to Grey method";
			bayes = false;
		}
	}
	whitepointR = params["whitepoint_r"].get<double>(0.0);
	whitepointB = params["whitepoint_b"].get<double>(0.0);
	if (bayes == false) {
		config.sensitivityR = config.sensitivityB = 1.0; /* nor do sensitivities make any sense */
		config.greyWorld = true; /* prevent the ct curve being used in manual mode */
	}
	/*
	 * The biasProportion parameter adds a small proportion of the counted
	 * pixles to a region biased to the biasCT colour temperature.
	 *
	 * A typical value for biasProportion would be between 0.05 to 0.1.
	 */
	biasProportion = params["bias_proportion"].get<double>(0.0);
	biasCT = params["bias_ct"].get<double>(kDefaultCT);
	return 0;
}

AwbBayes::AwbBayes(Controller *controller)
	: Awb(controller)
{
}

AwbBayes::~AwbBayes()
{
}

char const *AwbBayes::name() const
{
	return NAME;
}

int AwbBayes::read(const libcamera::YamlObject &params)
{
	int ret;

	ret = config_.read(params);
	if (ret)
		return ret;

	ret = bayesConfig_.read(params, config_);
	if (ret)
		return ret;

	return 0;
}

void AwbBayes::prepareStats()
{
	zones_.clear();
	/*
	 * LSC has already been applied to the stats in this pipeline, so stop
	 * any LSC compensation.  We also ignore config_.fast in this version.
	 */
	const double biasCtR = bayesConfig_.bayes ? config_.ctR.eval(bayesConfig_.biasCT) : 0;
	const double biasCtB = bayesConfig_.bayes ? config_.ctB.eval(bayesConfig_.biasCT) : 0;
	generateStats(zones_, statistics_, bayesConfig_.minPixels,
		      bayesConfig_.minG, getGlobalMetadata(),
		      bayesConfig_.biasProportion, biasCtR, biasCtB);
	/*
	 * apply sensitivities, so values appear to come from our "canonical"
	 * sensor.
	 */
	for (auto &zone : zones_) {
		zone.R *= config_.sensitivityR;
		zone.B *= config_.sensitivityB;
	}
}

ipa::Pwl AwbBayes::interpolatePrior()
{
	/*
	 * Interpolate the prior log likelihood function for our current lux
	 * value.
	 */
	if (lux_ <= bayesConfig_.priors.front().lux)
		return bayesConfig_.priors.front().prior;
	else if (lux_ >= bayesConfig_.priors.back().lux)
		return bayesConfig_.priors.back().prior;
	else {
		int idx = 0;
		/* find which two we lie between */
		while (bayesConfig_.priors[idx + 1].lux < lux_)
			idx++;
		double lux0 = bayesConfig_.priors[idx].lux,
		       lux1 = bayesConfig_.priors[idx + 1].lux;
		return ipa::Pwl::combine(bayesConfig_.priors[idx].prior,
					 bayesConfig_.priors[idx + 1].prior,
					 [&](double /*x*/, double y0, double y1) {
						 return y0 + (y1 - y0) *
								     (lux_ - lux0) / (lux1 - lux0);
					 });
	}
}

double AwbBayes::coarseSearch(ipa::Pwl const &prior)
{
	points_.clear(); /* assume doesn't deallocate memory */
	size_t bestPoint = 0;
	double t = mode_->ctLo;
	int spanR = 0, spanB = 0;
	/* Step down the CT curve evaluating log likelihood. */
	while (true) {
		double r = config_.ctR.eval(t, &spanR);
		double b = config_.ctB.eval(t, &spanB);
		double gainR = 1 / r, gainB = 1 / b;
		double delta2Sum = computeDelta2Sum(gainR, gainB, bayesConfig_.whitepointR, bayesConfig_.whitepointB);
		double priorLogLikelihood = prior.eval(prior.domain().clamp(t));
		double finalLogLikelihood = delta2Sum - priorLogLikelihood;
		LOG(RPiAwb, Debug)
			<< "t: " << t << " gain R " << gainR << " gain B "
			<< gainB << " delta2_sum " << delta2Sum
			<< " prior " << priorLogLikelihood << " final "
			<< finalLogLikelihood;
		points_.push_back(ipa::Pwl::Point({ t, finalLogLikelihood }));
		if (points_.back().y() < points_[bestPoint].y())
			bestPoint = points_.size() - 1;
		if (t == mode_->ctHi)
			break;
		/* for even steps along the r/b curve scale them by the current t */
		t = std::min(t + t / 10 * bayesConfig_.coarseStep, mode_->ctHi);
	}
	t = points_[bestPoint].x();
	LOG(RPiAwb, Debug) << "Coarse search found CT " << t;
	/*
	 * We have the best point of the search, but refine it with a quadratic
	 * interpolation around its neighbours.
	 */
	if (points_.size() > 2) {
		unsigned long bp = std::min(bestPoint, points_.size() - 2);
		bestPoint = std::max(1UL, bp);
		t = interpolateQuadatric(points_[bestPoint - 1],
					 points_[bestPoint],
					 points_[bestPoint + 1]);
		LOG(RPiAwb, Debug)
			<< "After quadratic refinement, coarse search has CT "
			<< t;
	}
	return t;
}

void AwbBayes::fineSearch(double &t, double &r, double &b, ipa::Pwl const &prior)
{
	int spanR = -1, spanB = -1;
	config_.ctR.eval(t, &spanR);
	config_.ctB.eval(t, &spanB);
	double step = t / 10 * bayesConfig_.coarseStep * 0.1;
	int nsteps = 5;
	double rDiff = config_.ctR.eval(t + nsteps * step, &spanR) -
		       config_.ctR.eval(t - nsteps * step, &spanR);
	double bDiff = config_.ctB.eval(t + nsteps * step, &spanB) -
		       config_.ctB.eval(t - nsteps * step, &spanB);
	ipa::Pwl::Point transverse({ bDiff, -rDiff });
	if (transverse.length2() < 1e-6)
		return;
	/*
	 * unit vector orthogonal to the b vs. r function (pointing outwards
	 * with r and b increasing)
	 */
	transverse = transverse / transverse.length();
	double bestLogLikelihood = 0, bestT = 0, bestR = 0, bestB = 0;
	double transverseRange = config_.transverseNeg + config_.transversePos;
	const int maxNumDeltas = 12;
	/* a transverse step approximately every 0.01 r/b units */
	int numDeltas = floor(transverseRange * 100 + 0.5) + 1;
	numDeltas = numDeltas < 3 ? 3 : (numDeltas > maxNumDeltas ? maxNumDeltas : numDeltas);
	/*
	 * Step down CT curve. March a bit further if the transverse range is
	 * large.
	 */
	nsteps += numDeltas;
	for (int i = -nsteps; i <= nsteps; i++) {
		double tTest = t + i * step;
		double priorLogLikelihood =
			prior.eval(prior.domain().clamp(tTest));
		double rCurve = config_.ctR.eval(tTest, &spanR);
		double bCurve = config_.ctB.eval(tTest, &spanB);
		/* x will be distance off the curve, y the log likelihood there */
		ipa::Pwl::Point points[maxNumDeltas];
		int bestPoint = 0;
		/* Take some measurements transversely *off* the CT curve. */
		for (int j = 0; j < numDeltas; j++) {
			points[j][0] = -config_.transverseNeg +
				       (transverseRange * j) / (numDeltas - 1);
			ipa::Pwl::Point rbTest = ipa::Pwl::Point({ rCurve, bCurve }) +
						 transverse * points[j].x();
			double rTest = rbTest.x(), bTest = rbTest.y();
			double gainR = 1 / rTest, gainB = 1 / bTest;
			double delta2Sum = computeDelta2Sum(gainR, gainB, bayesConfig_.whitepointR, bayesConfig_.whitepointB);
			points[j][1] = delta2Sum - priorLogLikelihood;
			LOG(RPiAwb, Debug)
				<< "At t " << tTest << " r " << rTest << " b "
				<< bTest << ": " << points[j].y();
			if (points[j].y() < points[bestPoint].y())
				bestPoint = j;
		}
		/*
		 * We have NUM_DELTAS points transversely across the CT curve,
		 * now let's do a quadratic interpolation for the best result.
		 */
		bestPoint = std::max(1, std::min(bestPoint, numDeltas - 2));
		ipa::Pwl::Point rbTest = ipa::Pwl::Point({ rCurve, bCurve }) +
					 transverse * interpolateQuadatric(points[bestPoint - 1],
									   points[bestPoint],
									   points[bestPoint + 1]);
		double rTest = rbTest.x(), bTest = rbTest.y();
		double gainR = 1 / rTest, gainB = 1 / bTest;
		double delta2Sum = computeDelta2Sum(gainR, gainB, bayesConfig_.whitepointR, bayesConfig_.whitepointB);
		double finalLogLikelihood = delta2Sum - priorLogLikelihood;
		LOG(RPiAwb, Debug)
			<< "Finally "
			<< tTest << " r " << rTest << " b " << bTest << ": "
			<< finalLogLikelihood
			<< (finalLogLikelihood < bestLogLikelihood ? " BEST" : "");
		if (bestT == 0 || finalLogLikelihood < bestLogLikelihood)
			bestLogLikelihood = finalLogLikelihood,
			bestT = tTest, bestR = rTest, bestB = bTest;
	}
	t = bestT, r = bestR, b = bestB;
	LOG(RPiAwb, Debug)
		<< "Fine search found t " << t << " r " << r << " b " << b;
}

void AwbBayes::awbBayes()
{
	/*
	 * May as well divide out G to save computeDelta2Sum from doing it over
	 * and over.
	 */
	for (auto &z : zones_)
		z.R = z.R / (z.G + 1), z.B = z.B / (z.G + 1);
	/*
	 * Get the current prior, and scale according to how many zones are
	 * valid... not entirely sure about this.
	 */
	ipa::Pwl prior = interpolatePrior();
	prior *= zones_.size() / (double)(statistics_->awbRegions.numRegions());
	prior.map([](double x, double y) {
		LOG(RPiAwb, Debug) << "(" << x << "," << y << ")";
	});
	double t = coarseSearch(prior);
	double r = config_.ctR.eval(t);
	double b = config_.ctB.eval(t);
	LOG(RPiAwb, Debug)
		<< "After coarse search: r " << r << " b " << b << " (gains r "
		<< 1 / r << " b " << 1 / b << ")";
	/*
	 * Not entirely sure how to handle the fine search yet. Mostly the
	 * estimated CT is already good enough, but the fine search allows us to
	 * wander transverely off the CT curve. Under some illuminants, where
	 * there may be more or less green light, this may prove beneficial,
	 * though I probably need more real datasets before deciding exactly how
	 * this should be controlled and tuned.
	 */
	fineSearch(t, r, b, prior);
	LOG(RPiAwb, Debug)
		<< "After fine search: r " << r << " b " << b << " (gains r "
		<< 1 / r << " b " << 1 / b << ")";
	/*
	 * Write results out for the main thread to pick up. Remember to adjust
	 * the gains from the ones that the "canonical sensor" would require to
	 * the ones needed by *this* sensor.
	 */
	asyncResults_.temperatureK = t;
	asyncResults_.gainR = 1.0 / r * config_.sensitivityR;
	asyncResults_.gainG = 1.0;
	asyncResults_.gainB = 1.0 / b * config_.sensitivityB;
}

void AwbBayes::doAwb()
{
	prepareStats();
	LOG(RPiAwb, Debug) << "Valid zones: " << zones_.size();
	if (zones_.size() > bayesConfig_.minRegions) {
		if (bayesConfig_.bayes)
			awbBayes();
		else
			awbGrey();
		LOG(RPiAwb, Debug)
			<< "CT found is "
			<< asyncResults_.temperatureK
			<< " with gains r " << asyncResults_.gainR
			<< " and b " << asyncResults_.gainB;
	}
	/*
	 * we're done with these; we may as well relinquish our hold on the
	 * pointer.
	 */
	statistics_.reset();
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new AwbBayes(controller);
}
static RegisterAlgorithm reg(NAME, &create);

} /* namespace RPiController */
