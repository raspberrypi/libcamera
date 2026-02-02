/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * AWB control algorithm
 */
#include "awb.h"

#include "../lux_status.h"

#include "alsc_status.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiAwb)

constexpr double kDefaultCT = 4500.0;

static int readCtCurve(ipa::Pwl &ctR, ipa::Pwl &ctB, const libcamera::YamlObject &params)
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

int AwbConfig::read(const libcamera::YamlObject &params)
{
	int ret;

	framePeriod = params["frame_period"].get<uint16_t>(10);
	startupFrames = params["startup_frames"].get<uint16_t>(10);
	convergenceFrames = params["convergence_frames"].get<unsigned int>(3);
	speed = params["speed"].get<double>(0.05);

	if (params.contains("ct_curve")) {
		ret = readCtCurve(ctR, ctB, params["ct_curve"]);
		if (ret)
			return ret;
		/* We will want the inverse functions of these too. */
		ctRInverse = ctR.inverse().first;
		ctBInverse = ctB.inverse().first;
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

	deltaLimit = params["delta_limit"].get<double>(0.2);
	transversePos = params["transverse_pos"].get<double>(0.01);
	transverseNeg = params["transverse_neg"].get<double>(0.01);

	if (transversePos <= 0 || transverseNeg <= 0) {
		LOG(RPiAwb, Error) << "AwbConfig: transverse_pos/neg must be > 0";
		return -EINVAL;
	}

	sensitivityR = params["sensitivity_r"].get<double>(1.0);
	sensitivityB = params["sensitivity_b"].get<double>(1.0);

	if (hasCtCurve() && defaultMode != nullptr) {
		greyWorld = false;
	} else {
		greyWorld = true;
		sensitivityR = sensitivityB = 1.0; /* nor do sensitivities make any sense */
	}

	return 0;
}

bool AwbConfig::hasCtCurve() const
{
	return !ctR.empty() && !ctB.empty();
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

void Awb::initialise()
{
	frameCount_ = framePhase_ = 0;
	/*
	 * Put something sane into the status that we are filtering towards,
	 * just in case the first few frames don't have anything meaningful in
	 * them.
	 */
	if (!config_.greyWorld) {
		syncResults_.temperatureK = config_.ctR.domain().clamp(4000);
		syncResults_.gainR = 1.0 / config_.ctR.eval(syncResults_.temperatureK);
		syncResults_.gainG = 1.0;
		syncResults_.gainB = 1.0 / config_.ctB.eval(syncResults_.temperatureK);
	} else {
		/* random values just to stop the world blowing up */
		syncResults_.temperatureK = kDefaultCT;
		syncResults_.gainR = syncResults_.gainG = syncResults_.gainB = 1.0;
	}
	prevSyncResults_ = syncResults_;
	asyncResults_ = syncResults_;
}

void Awb::initialValues(double &gainR, double &gainB)
{
	gainR = syncResults_.gainR;
	gainB = syncResults_.gainB;
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
		if (!config_.greyWorld) {
			/* Also estimate the best corresponding colour temperature from the curves. */
			double ctR = config_.ctRInverse.eval(config_.ctRInverse.domain().clamp(1 / manualR_));
			double ctB = config_.ctBInverse.eval(config_.ctBInverse.domain().clamp(1 / manualB_));
			prevSyncResults_.temperatureK = (ctR + ctB) / 2;
			syncResults_.temperatureK = prevSyncResults_.temperatureK;
		}
	}
}

void Awb::setColourTemperature(double temperatureK)
{
	if (config_.greyWorld) {
		LOG(RPiAwb, Warning) << "AWB uncalibrated - cannot set colour temperature";
		return;
	}

	temperatureK = config_.ctR.domain().clamp(temperatureK);
	manualR_ = 1 / config_.ctR.eval(temperatureK);
	manualB_ = 1 / config_.ctB.eval(temperatureK);

	syncResults_.temperatureK = temperatureK;
	syncResults_.gainR = manualR_;
	syncResults_.gainG = 1.0;
	syncResults_.gainB = manualB_;
	prevSyncResults_ = syncResults_;
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

void Awb::generateStats(std::vector<Awb::RGB> &zones,
			StatisticsPtr &stats, double minPixels,
			double minG, Metadata &globalMetadata,
			double biasProportion, double biasCtR, double biasCtB)
{
	std::scoped_lock<RPiController::Metadata> l(globalMetadata);

	for (unsigned int i = 0; i < stats->awbRegions.numRegions(); i++) {
		Awb::RGB zone;
		auto &region = stats->awbRegions.get(i);
		if (region.counted >= minPixels) {
			zone.G = region.val.gSum / region.counted;
			if (zone.G < minG)
				continue;
			zone.R = region.val.rSum / region.counted;
			zone.B = region.val.bSum / region.counted;
			/*
			* Add some bias samples to allow the search to tend to a
			* bias CT in failure cases.
			*/
			const unsigned int proportion = biasProportion * region.counted;
			zone.R += proportion * biasCtR;
			zone.B += proportion * biasCtB;
			zone.G += proportion * 1.0;
			/* Factor in the ALSC applied colour shading correction if required. */
			const AlscStatus *alscStatus = globalMetadata.getLocked<AlscStatus>("alsc.status");
			if (stats->colourStatsPos == Statistics::ColourStatsPos::PreLsc && alscStatus) {
				zone.R *= alscStatus->r[i];
				zone.G *= alscStatus->g[i];
				zone.B *= alscStatus->b[i];
			}
			zones.push_back(zone);
		}
	}
}

double Awb::computeDelta2Sum(double gainR, double gainB, double whitepointR, double whitepointB)
{
	/*
	 * Compute the sum of the squared colour error (non-greyness) as it
	 * appears in the log likelihood equation.
	 */
	double delta2Sum = 0;
	for (auto &z : zones_) {
		double deltaR = gainR * z.R - 1 - whitepointR;
		double deltaB = gainB * z.B - 1 - whitepointB;
		double delta2 = deltaR * deltaR + deltaB * deltaB;
		/* LOG(RPiAwb, Debug) << "deltaR " << deltaR << " deltaB " << deltaB << " delta2 " << delta2; */
		delta2 = std::min(delta2, config_.deltaLimit);
		delta2Sum += delta2;
	}
	return delta2Sum;
}

double Awb::interpolateQuadatric(libcamera::ipa::Pwl::Point const &a,
				 libcamera::ipa::Pwl::Point const &b,
				 libcamera::ipa::Pwl::Point const &c)
{
	/*
	* Given 3 points on a curve, find the extremum of the function in that
	* interval by fitting a quadratic.
	*/
	const double eps = 1e-3;
	ipa::Pwl::Point ca = c - a, ba = b - a;
	double denominator = 2 * (ba.y() * ca.x() - ca.y() * ba.x());
	if (std::abs(denominator) > eps) {
		double numerator = ba.y() * ca.x() * ca.x() - ca.y() * ba.x() * ba.x();
		double result = numerator / denominator + a.x();
		return std::max(a.x(), std::min(c.x(), result));
	}
	/* has degenerated to straight line segment */
	return a.y() < c.y() - eps ? a.x() : (c.y() < a.y() - eps ? c.x() : b.x());
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
	/*
	 * The grey world model can't estimate the colour temperature, use a
	 * default value.
	 */
	asyncResults_.temperatureK = kDefaultCT;
	asyncResults_.gainR = gainR;
	asyncResults_.gainG = 1.0;
	asyncResults_.gainB = gainB;
}
