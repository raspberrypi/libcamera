/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * awb.cpp - AWB control algorithm
 */

#include <assert.h>
#include <functional>

#include <libcamera/base/log.h>

#include "../lux_status.h"

#include "awb.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiAwb)

#define NAME "rpi.awb"

static constexpr unsigned int AwbStatsSizeX = DEFAULT_AWB_REGIONS_X;
static constexpr unsigned int AwbStatsSizeY = DEFAULT_AWB_REGIONS_Y;

/*
 * todo - the locking in this algorithm needs some tidying up as has been done
 * elsewhere (ALSC and AGC).
 */

int AwbMode::read(const libcamera::YamlObject &params)
{
	auto value = params["lo"].get<double>();
	if (!value)
		return -EINVAL;
	ctLo = *value;

	value = params["hi"].get<double>();
	if (!value)
		return -EINVAL;
	ctHi = *value;

	return 0;
}

int AwbPrior::read(const libcamera::YamlObject &params)
{
	auto value = params["lux"].get<double>();
	if (!value)
		return -EINVAL;
	lux = *value;

	return prior.read(params["prior"]);
}

static int readCtCurve(Pwl &ctR, Pwl &ctB, const libcamera::YamlObject &params)
{
	if (params.size() % 3) {
		LOG(RPiAwb, Error) << "AwbConfig: incomplete CT curve entry";
		return -EINVAL;
	}

	if (params.size() < 6) {
		LOG(RPiAwb, Error) << "AwbConfig: insufficient points in CT curve";
		return -EINVAL;
	}

	const auto &list = params.asList();

	for (auto it = list.begin(); it != list.end(); it++) {
		auto value = it->get<double>();
		if (!value)
			return -EINVAL;
		double ct = *value;

		assert(it == list.begin() || ct != ctR.domain().end);

		value = (++it)->get<double>();
		if (!value)
			return -EINVAL;
		ctR.append(ct, *value);

		value = (++it)->get<double>();
		if (!value)
			return -EINVAL;
		ctB.append(ct, *value);
	}

	return 0;
}

int AwbConfig::read(const libcamera::YamlObject &params)
{
	int ret;

	bayes = params["bayes"].get<int>(1);
	framePeriod = params["frame_period"].get<uint16_t>(10);
	startupFrames = params["startup_frames"].get<uint16_t>(10);
	convergenceFrames = params["convergence_frames"].get<unsigned int>(3);
	speed = params["speed"].get<double>(0.05);

	if (params.contains("ct_curve")) {
		ret = readCtCurve(ctR, ctB, params["ct_curve"]);
		if (ret)
			return ret;
		/* We will want the inverse functions of these too. */
		ctRInverse = ctR.inverse();
		ctBInverse = ctB.inverse();
	}

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
			return ret;
		}
	}
	if (params.contains("modes")) {
		for (const auto &[key, value] : params["modes"].asDict()) {
			ret = modes[key].read(value);
			if (ret)
				return ret;
			if (defaultMode == nullptr)
				defaultMode = &modes[key];
		}
		if (defaultMode == nullptr) {
			LOG(RPiAwb, Error) << "AwbConfig: no AWB modes configured";
			return -EINVAL;
		}
	}

	minPixels = params["min_pixels"].get<double>(16.0);
	minG = params["min_G"].get<uint16_t>(32);
	minRegions = params["min_regions"].get<uint32_t>(10);
	deltaLimit = params["delta_limit"].get<double>(0.2);
	coarseStep = params["coarse_step"].get<double>(0.2);
	transversePos = params["transverse_pos"].get<double>(0.01);
	transverseNeg = params["transverse_neg"].get<double>(0.01);
	if (transversePos <= 0 || transverseNeg <= 0) {
		LOG(RPiAwb, Error) << "AwbConfig: transverse_pos/neg must be > 0";
		return -EINVAL;
	}

	sensitivityR = params["sensitivity_r"].get<double>(1.0);
	sensitivityB = params["sensitivity_b"].get<double>(1.0);

	if (bayes) {
		if (ctR.empty() || ctB.empty() || priors.empty() ||
		    defaultMode == nullptr) {
			LOG(RPiAwb, Warning)
				<< "Bayesian AWB mis-configured - switch to Grey method";
			bayes = false;
		}
	}
	fast = params[fast].get<int>(bayes); /* default to fast for Bayesian, otherwise slow */
	whitepointR = params["whitepoint_r"].get<double>(0.0);
	whitepointB = params["whitepoint_b"].get<double>(0.0);
	if (bayes == false)
		sensitivityR = sensitivityB = 1.0; /* nor do sensitivities make any sense */
	return 0;
}

Awb::Awb(Controller *controller)
	: AwbAlgorithm(controller)
{
	asyncAbort_ = asyncStart_ = asyncStarted_ = asyncFinished_ = false;
	mode_ = nullptr;
	manualR_ = manualB_ = 0.0;
	asyncThread_ = std::thread(std::bind(&Awb::asyncFunc, this));
}

Awb::~Awb()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		asyncAbort_ = true;
	}
	asyncSignal_.notify_one();
	asyncThread_.join();
}

char const *Awb::name() const
{
	return NAME;
}

int Awb::read(const libcamera::YamlObject &params)
{
	return config_.read(params);
}

void Awb::initialise()
{
	frameCount_ = framePhase_ = 0;
	/*
	 * Put something sane into the status that we are filtering towards,
	 * just in case the first few frames don't have anything meaningful in
	 * them.
	 */
	if (!config_.ctR.empty() && !config_.ctB.empty()) {
		syncResults_.temperatureK = config_.ctR.domain().clip(4000);
		syncResults_.gainR = 1.0 / config_.ctR.eval(syncResults_.temperatureK);
		syncResults_.gainG = 1.0;
		syncResults_.gainB = 1.0 / config_.ctB.eval(syncResults_.temperatureK);
	} else {
		/* random values just to stop the world blowing up */
		syncResults_.temperatureK = 4500;
		syncResults_.gainR = syncResults_.gainG = syncResults_.gainB = 1.0;
	}
	prevSyncResults_ = syncResults_;
	asyncResults_ = syncResults_;
}

void Awb::disableAuto()
{
	/* Freeze the most recent values, and treat them as manual gains */
	manualR_ = syncResults_.gainR = prevSyncResults_.gainR;
	manualB_ = syncResults_.gainB = prevSyncResults_.gainB;
	syncResults_.gainG = prevSyncResults_.gainG;
	syncResults_.temperatureK = prevSyncResults_.temperatureK;
}

void Awb::enableAuto()
{
	manualR_ = 0.0;
	manualB_ = 0.0;
}

unsigned int Awb::getConvergenceFrames() const
{
	/*
	 * If not in auto mode, there is no convergence
	 * to happen, so no need to drop any frames - return zero.
	 */
	if (!isAutoEnabled())
		return 0;
	else
		return config_.convergenceFrames;
}

void Awb::setMode(std::string const &modeName)
{
	modeName_ = modeName;
}

void Awb::setManualGains(double manualR, double manualB)
{
	/* If any of these are 0.0, we swich back to auto. */
	manualR_ = manualR;
	manualB_ = manualB;
	/*
	 * If not in auto mode, set these values into the syncResults which
	 * means that Prepare() will adopt them immediately.
	 */
	if (!isAutoEnabled()) {
		syncResults_.gainR = prevSyncResults_.gainR = manualR_;
		syncResults_.gainG = prevSyncResults_.gainG = 1.0;
		syncResults_.gainB = prevSyncResults_.gainB = manualB_;
		if (config_.bayes) {
			/* Also estimate the best corresponding colour temperature from the curves. */
			double ctR = config_.ctRInverse.eval(config_.ctRInverse.domain().clip(1 / manualR_));
			double ctB = config_.ctBInverse.eval(config_.ctBInverse.domain().clip(1 / manualB_));
			prevSyncResults_.temperatureK = (ctR + ctB) / 2;
			syncResults_.temperatureK = prevSyncResults_.temperatureK;
		}
	}
}

void Awb::switchMode([[maybe_unused]] CameraMode const &cameraMode,
		     Metadata *metadata)
{
	/* Let other algorithms know the current white balance values. */
	metadata->set("awb.status", prevSyncResults_);
}

bool Awb::isAutoEnabled() const
{
	return manualR_ == 0.0 || manualB_ == 0.0;
}

void Awb::fetchAsyncResults()
{
	LOG(RPiAwb, Debug) << "Fetch AWB results";
	asyncFinished_ = false;
	asyncStarted_ = false;
	/*
	 * It's possible manual gains could be set even while the async
	 * thread was running, so only copy the results if still in auto mode.
	 */
	if (isAutoEnabled())
		syncResults_ = asyncResults_;
}

void Awb::restartAsync(StatisticsPtr &stats, double lux)
{
	LOG(RPiAwb, Debug) << "Starting AWB calculation";
	/* this makes a new reference which belongs to the asynchronous thread */
	statistics_ = stats;
	/* store the mode as it could technically change */
	auto m = config_.modes.find(modeName_);
	mode_ = m != config_.modes.end()
			? &m->second
			: (mode_ == nullptr ? config_.defaultMode : mode_);
	lux_ = lux;
	framePhase_ = 0;
	asyncStarted_ = true;
	size_t len = modeName_.copy(asyncResults_.mode,
				    sizeof(asyncResults_.mode) - 1);
	asyncResults_.mode[len] = '\0';
	{
		std::lock_guard<std::mutex> lock(mutex_);
		asyncStart_ = true;
	}
	asyncSignal_.notify_one();
}

void Awb::prepare(Metadata *imageMetadata)
{
	if (frameCount_ < (int)config_.startupFrames)
		frameCount_++;
	double speed = frameCount_ < (int)config_.startupFrames
			       ? 1.0
			       : config_.speed;
	LOG(RPiAwb, Debug)
		<< "frame_count " << frameCount_ << " speed " << speed;
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (asyncStarted_ && asyncFinished_)
			fetchAsyncResults();
	}
	/* Finally apply IIR filter to results and put into metadata. */
	memcpy(prevSyncResults_.mode, syncResults_.mode,
	       sizeof(prevSyncResults_.mode));
	prevSyncResults_.temperatureK = speed * syncResults_.temperatureK +
					(1.0 - speed) * prevSyncResults_.temperatureK;
	prevSyncResults_.gainR = speed * syncResults_.gainR +
				 (1.0 - speed) * prevSyncResults_.gainR;
	prevSyncResults_.gainG = speed * syncResults_.gainG +
				 (1.0 - speed) * prevSyncResults_.gainG;
	prevSyncResults_.gainB = speed * syncResults_.gainB +
				 (1.0 - speed) * prevSyncResults_.gainB;
	imageMetadata->set("awb.status", prevSyncResults_);
	LOG(RPiAwb, Debug)
		<< "Using AWB gains r " << prevSyncResults_.gainR << " g "
		<< prevSyncResults_.gainG << " b "
		<< prevSyncResults_.gainB;
}

void Awb::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	/* Count frames since we last poked the async thread. */
	if (framePhase_ < (int)config_.framePeriod)
		framePhase_++;
	LOG(RPiAwb, Debug) << "frame_phase " << framePhase_;
	/* We do not restart the async thread if we're not in auto mode. */
	if (isAutoEnabled() &&
	    (framePhase_ >= (int)config_.framePeriod ||
	     frameCount_ < (int)config_.startupFrames)) {
		/* Update any settings and any image metadata that we need. */
		struct LuxStatus luxStatus = {};
		luxStatus.lux = 400; /* in case no metadata */
		if (imageMetadata->get("lux.status", luxStatus) != 0)
			LOG(RPiAwb, Debug) << "No lux metadata found";
		LOG(RPiAwb, Debug) << "Awb lux value is " << luxStatus.lux;

		if (asyncStarted_ == false)
			restartAsync(stats, luxStatus.lux);
	}
}

void Awb::asyncFunc()
{
	while (true) {
		{
			std::unique_lock<std::mutex> lock(mutex_);
			asyncSignal_.wait(lock, [&] {
				return asyncStart_ || asyncAbort_;
			});
			asyncStart_ = false;
			if (asyncAbort_)
				break;
		}
		doAwb();
		{
			std::lock_guard<std::mutex> lock(mutex_);
			asyncFinished_ = true;
		}
		syncSignal_.notify_one();
	}
}

static void generateStats(std::vector<Awb::RGB> &zones,
			  bcm2835_isp_stats_region *stats, double minPixels,
			  double minG)
{
	for (unsigned int i = 0; i < AwbStatsSizeX * AwbStatsSizeY; i++) {
		Awb::RGB zone;
		double counted = stats[i].counted;
		if (counted >= minPixels) {
			zone.G = stats[i].g_sum / counted;
			if (zone.G >= minG) {
				zone.R = stats[i].r_sum / counted;
				zone.B = stats[i].b_sum / counted;
				zones.push_back(zone);
			}
		}
	}
}

void Awb::prepareStats()
{
	zones_.clear();
	/*
	 * LSC has already been applied to the stats in this pipeline, so stop
	 * any LSC compensation.  We also ignore config_.fast in this version.
	 */
	generateStats(zones_, statistics_->awb_stats, config_.minPixels,
		      config_.minG);
	/*
	 * we're done with these; we may as well relinquish our hold on the
	 * pointer.
	 */
	statistics_.reset();
	/*
	 * apply sensitivities, so values appear to come from our "canonical"
	 * sensor.
	 */
	for (auto &zone : zones_) {
		zone.R *= config_.sensitivityR;
		zone.B *= config_.sensitivityB;
	}
}

double Awb::computeDelta2Sum(double gainR, double gainB)
{
	/*
	 * Compute the sum of the squared colour error (non-greyness) as it
	 * appears in the log likelihood equation.
	 */
	double delta2Sum = 0;
	for (auto &z : zones_) {
		double deltaR = gainR * z.R - 1 - config_.whitepointR;
		double deltaB = gainB * z.B - 1 - config_.whitepointB;
		double delta2 = deltaR * deltaR + deltaB * deltaB;
		/* LOG(RPiAwb, Debug) << "deltaR " << deltaR << " deltaB " << deltaB << " delta2 " << delta2; */
		delta2 = std::min(delta2, config_.deltaLimit);
		delta2Sum += delta2;
	}
	return delta2Sum;
}

Pwl Awb::interpolatePrior()
{
	/*
	 * Interpolate the prior log likelihood function for our current lux
	 * value.
	 */
	if (lux_ <= config_.priors.front().lux)
		return config_.priors.front().prior;
	else if (lux_ >= config_.priors.back().lux)
		return config_.priors.back().prior;
	else {
		int idx = 0;
		/* find which two we lie between */
		while (config_.priors[idx + 1].lux < lux_)
			idx++;
		double lux0 = config_.priors[idx].lux,
		       lux1 = config_.priors[idx + 1].lux;
		return Pwl::combine(config_.priors[idx].prior,
				    config_.priors[idx + 1].prior,
				    [&](double /*x*/, double y0, double y1) {
					    return y0 + (y1 - y0) *
							(lux_ - lux0) / (lux1 - lux0);
				    });
	}
}

static double interpolateQuadatric(Pwl::Point const &a, Pwl::Point const &b,
				   Pwl::Point const &c)
{
	/*
	 * Given 3 points on a curve, find the extremum of the function in that
	 * interval by fitting a quadratic.
	 */
	const double eps = 1e-3;
	Pwl::Point ca = c - a, ba = b - a;
	double denominator = 2 * (ba.y * ca.x - ca.y * ba.x);
	if (abs(denominator) > eps) {
		double numerator = ba.y * ca.x * ca.x - ca.y * ba.x * ba.x;
		double result = numerator / denominator + a.x;
		return std::max(a.x, std::min(c.x, result));
	}
	/* has degenerated to straight line segment */
	return a.y < c.y - eps ? a.x : (c.y < a.y - eps ? c.x : b.x);
}

double Awb::coarseSearch(Pwl const &prior)
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
		double delta2Sum = computeDelta2Sum(gainR, gainB);
		double priorLogLikelihood = prior.eval(prior.domain().clip(t));
		double finalLogLikelihood = delta2Sum - priorLogLikelihood;
		LOG(RPiAwb, Debug)
			<< "t: " << t << " gain R " << gainR << " gain B "
			<< gainB << " delta2_sum " << delta2Sum
			<< " prior " << priorLogLikelihood << " final "
			<< finalLogLikelihood;
		points_.push_back(Pwl::Point(t, finalLogLikelihood));
		if (points_.back().y < points_[bestPoint].y)
			bestPoint = points_.size() - 1;
		if (t == mode_->ctHi)
			break;
		/* for even steps along the r/b curve scale them by the current t */
		t = std::min(t + t / 10 * config_.coarseStep, mode_->ctHi);
	}
	t = points_[bestPoint].x;
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

void Awb::fineSearch(double &t, double &r, double &b, Pwl const &prior)
{
	int spanR = -1, spanB = -1;
	config_.ctR.eval(t, &spanR);
	config_.ctB.eval(t, &spanB);
	double step = t / 10 * config_.coarseStep * 0.1;
	int nsteps = 5;
	double rDiff = config_.ctR.eval(t + nsteps * step, &spanR) -
		       config_.ctR.eval(t - nsteps * step, &spanR);
	double bDiff = config_.ctB.eval(t + nsteps * step, &spanB) -
		       config_.ctB.eval(t - nsteps * step, &spanB);
	Pwl::Point transverse(bDiff, -rDiff);
	if (transverse.len2() < 1e-6)
		return;
	/*
	 * unit vector orthogonal to the b vs. r function (pointing outwards
	 * with r and b increasing)
	 */
	transverse = transverse / transverse.len();
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
			prior.eval(prior.domain().clip(tTest));
		double rCurve = config_.ctR.eval(tTest, &spanR);
		double bCurve = config_.ctB.eval(tTest, &spanB);
		/* x will be distance off the curve, y the log likelihood there */
		Pwl::Point points[maxNumDeltas];
		int bestPoint = 0;
		/* Take some measurements transversely *off* the CT curve. */
		for (int j = 0; j < numDeltas; j++) {
			points[j].x = -config_.transverseNeg +
				      (transverseRange * j) / (numDeltas - 1);
			Pwl::Point rbTest = Pwl::Point(rCurve, bCurve) +
					    transverse * points[j].x;
			double rTest = rbTest.x, bTest = rbTest.y;
			double gainR = 1 / rTest, gainB = 1 / bTest;
			double delta2Sum = computeDelta2Sum(gainR, gainB);
			points[j].y = delta2Sum - priorLogLikelihood;
			LOG(RPiAwb, Debug)
				<< "At t " << tTest << " r " << rTest << " b "
				<< bTest << ": " << points[j].y;
			if (points[j].y < points[bestPoint].y)
				bestPoint = j;
		}
		/*
		 * We have NUM_DELTAS points transversely across the CT curve,
		 * now let's do a quadratic interpolation for the best result.
		 */
		bestPoint = std::max(1, std::min(bestPoint, numDeltas - 2));
		Pwl::Point rbTest = Pwl::Point(rCurve, bCurve) +
					transverse * interpolateQuadatric(points[bestPoint - 1],
									points[bestPoint],
									points[bestPoint + 1]);
		double rTest = rbTest.x, bTest = rbTest.y;
		double gainR = 1 / rTest, gainB = 1 / bTest;
		double delta2Sum = computeDelta2Sum(gainR, gainB);
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

void Awb::awbBayes()
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
	Pwl prior = interpolatePrior();
	prior *= zones_.size() / (double)(AwbStatsSizeX * AwbStatsSizeY);
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

void Awb::awbGrey()
{
	LOG(RPiAwb, Debug) << "Grey world AWB";
	/*
	 * Make a separate list of the derivatives for each of red and blue, so
	 * that we can sort them to exclude the extreme gains.  We could
	 * consider some variations, such as normalising all the zones first, or
	 * doing an L2 average etc.
	 */
	std::vector<RGB> &derivsR(zones_);
	std::vector<RGB> derivsB(derivsR);
	std::sort(derivsR.begin(), derivsR.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.R < b.G * a.R;
		  });
	std::sort(derivsB.begin(), derivsB.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.B < b.G * a.B;
		  });
	/* Average the middle half of the values. */
	int discard = derivsR.size() / 4;
	RGB sumR(0, 0, 0), sumB(0, 0, 0);
	for (auto ri = derivsR.begin() + discard,
		  bi = derivsB.begin() + discard;
	     ri != derivsR.end() - discard; ri++, bi++)
		sumR += *ri, sumB += *bi;
	double gainR = sumR.G / (sumR.R + 1),
	       gainB = sumB.G / (sumB.B + 1);
	asyncResults_.temperatureK = 4500; /* don't know what it is */
	asyncResults_.gainR = gainR;
	asyncResults_.gainG = 1.0;
	asyncResults_.gainB = gainB;
}

void Awb::doAwb()
{
	prepareStats();
	LOG(RPiAwb, Debug) << "Valid zones: " << zones_.size();
	if (zones_.size() > config_.minRegions) {
		if (config_.bayes)
			awbBayes();
		else
			awbGrey();
		LOG(RPiAwb, Debug)
			<< "CT found is "
			<< asyncResults_.temperatureK
			<< " with gains r " << asyncResults_.gainR
			<< " and b " << asyncResults_.gainB;
	}
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Awb(controller);
}
static RegisterAlgorithm reg(NAME, &create);
