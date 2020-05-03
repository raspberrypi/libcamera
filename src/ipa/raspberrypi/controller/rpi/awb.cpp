/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb.cpp - AWB control algorithm
 */

#include "../logging.hpp"
#include "../lux_status.h"

#include "awb.hpp"

using namespace RPi;

#define NAME "rpi.awb"

#define AWB_STATS_SIZE_X DEFAULT_AWB_REGIONS_X
#define AWB_STATS_SIZE_Y DEFAULT_AWB_REGIONS_Y

const double Awb::RGB::INVALID = -1.0;

void AwbMode::Read(boost::property_tree::ptree const &params)
{
	ct_lo = params.get<double>("lo");
	ct_hi = params.get<double>("hi");
}

void AwbPrior::Read(boost::property_tree::ptree const &params)
{
	lux = params.get<double>("lux");
	prior.Read(params.get_child("prior"));
}

static void read_ct_curve(Pwl &ct_r, Pwl &ct_b,
			  boost::property_tree::ptree const &params)
{
	int num = 0;
	for (auto it = params.begin(); it != params.end(); it++) {
		double ct = it->second.get_value<double>();
		assert(it == params.begin() || ct != ct_r.Domain().end);
		if (++it == params.end())
			throw std::runtime_error(
				"AwbConfig: incomplete CT curve entry");
		ct_r.Append(ct, it->second.get_value<double>());
		if (++it == params.end())
			throw std::runtime_error(
				"AwbConfig: incomplete CT curve entry");
		ct_b.Append(ct, it->second.get_value<double>());
		num++;
	}
	if (num < 2)
		throw std::runtime_error(
			"AwbConfig: insufficient points in CT curve");
}

void AwbConfig::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG("AwbConfig");
	bayes = params.get<int>("bayes", 1);
	frame_period = params.get<uint16_t>("frame_period", 10);
	startup_frames = params.get<uint16_t>("startup_frames", 10);
	speed = params.get<double>("speed", 0.05);
	if (params.get_child_optional("ct_curve"))
		read_ct_curve(ct_r, ct_b, params.get_child("ct_curve"));
	if (params.get_child_optional("priors")) {
		for (auto &p : params.get_child("priors")) {
			AwbPrior prior;
			prior.Read(p.second);
			if (!priors.empty() && prior.lux <= priors.back().lux)
				throw std::runtime_error(
					"AwbConfig: Prior must be ordered in increasing lux value");
			priors.push_back(prior);
		}
		if (priors.empty())
			throw std::runtime_error(
				"AwbConfig: no AWB priors configured");
	}
	if (params.get_child_optional("modes")) {
		for (auto &p : params.get_child("modes")) {
			modes[p.first].Read(p.second);
			if (default_mode == nullptr)
				default_mode = &modes[p.first];
		}
		if (default_mode == nullptr)
			throw std::runtime_error(
				"AwbConfig: no AWB modes configured");
	}
	min_pixels = params.get<double>("min_pixels", 16.0);
	min_G = params.get<uint16_t>("min_G", 32);
	min_regions = params.get<uint32_t>("min_regions", 10);
	delta_limit = params.get<double>("delta_limit", 0.2);
	coarse_step = params.get<double>("coarse_step", 0.2);
	transverse_pos = params.get<double>("transverse_pos", 0.01);
	transverse_neg = params.get<double>("transverse_neg", 0.01);
	if (transverse_pos <= 0 || transverse_neg <= 0)
		throw std::runtime_error(
			"AwbConfig: transverse_pos/neg must be > 0");
	sensitivity_r = params.get<double>("sensitivity_r", 1.0);
	sensitivity_b = params.get<double>("sensitivity_b", 1.0);
	if (bayes) {
		if (ct_r.Empty() || ct_b.Empty() || priors.empty() ||
		    default_mode == nullptr) {
			RPI_WARN(
				"Bayesian AWB mis-configured - switch to Grey method");
			bayes = false;
		}
	}
	fast = params.get<int>(
		"fast", bayes); // default to fast for Bayesian, otherwise slow
	whitepoint_r = params.get<double>("whitepoint_r", 0.0);
	whitepoint_b = params.get<double>("whitepoint_b", 0.0);
	if (bayes == false)
		sensitivity_r = sensitivity_b =
			1.0; // nor do sensitivities make any sense
}

Awb::Awb(Controller *controller)
	: AwbAlgorithm(controller)
{
	async_abort_ = async_start_ = async_started_ = async_finished_ = false;
	mode_ = nullptr;
	manual_r_ = manual_b_ = 0.0;
	async_thread_ = std::thread(std::bind(&Awb::asyncFunc, this));
}

Awb::~Awb()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		async_abort_ = true;
		async_signal_.notify_one();
	}
	async_thread_.join();
}

char const *Awb::Name() const
{
	return NAME;
}

void Awb::Read(boost::property_tree::ptree const &params)
{
	config_.Read(params);
}

void Awb::Initialise()
{
	frame_count2_ = frame_count_ = frame_phase_ = 0;
	// Put something sane into the status that we are filtering towards,
	// just in case the first few frames don't have anything meaningful in
	// them.
	if (!config_.ct_r.Empty() && !config_.ct_b.Empty()) {
		sync_results_.temperature_K = config_.ct_r.Domain().Clip(4000);
		sync_results_.gain_r =
			1.0 / config_.ct_r.Eval(sync_results_.temperature_K);
		sync_results_.gain_g = 1.0;
		sync_results_.gain_b =
			1.0 / config_.ct_b.Eval(sync_results_.temperature_K);
	} else {
		// random values just to stop the world blowing up
		sync_results_.temperature_K = 4500;
		sync_results_.gain_r = sync_results_.gain_g =
			sync_results_.gain_b = 1.0;
	}
	prev_sync_results_ = sync_results_;
}

void Awb::SetMode(std::string const &mode_name)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	mode_name_ = mode_name;
}

void Awb::SetManualGains(double manual_r, double manual_b)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	// If any of these are 0.0, we swich back to auto.
	manual_r_ = manual_r;
	manual_b_ = manual_b;
}

void Awb::fetchAsyncResults()
{
	RPI_LOG("Fetch AWB results");
	async_finished_ = false;
	async_started_ = false;
	sync_results_ = async_results_;
}

void Awb::restartAsync(StatisticsPtr &stats, std::string const &mode_name,
		       double lux)
{
	RPI_LOG("Starting AWB thread");
	// this makes a new reference which belongs to the asynchronous thread
	statistics_ = stats;
	// store the mode as it could technically change
	auto m = config_.modes.find(mode_name);
	mode_ = m != config_.modes.end()
			? &m->second
			: (mode_ == nullptr ? config_.default_mode : mode_);
	lux_ = lux;
	frame_phase_ = 0;
	async_start_ = true;
	async_started_ = true;
	size_t len = mode_name.copy(async_results_.mode,
				    sizeof(async_results_.mode) - 1);
	async_results_.mode[len] = '\0';
	async_signal_.notify_one();
}

void Awb::Prepare(Metadata *image_metadata)
{
	if (frame_count_ < (int)config_.startup_frames)
		frame_count_++;
	double speed = frame_count_ < (int)config_.startup_frames
			       ? 1.0
			       : config_.speed;
	RPI_LOG("Awb: frame_count " << frame_count_ << " speed " << speed);
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (async_started_ && async_finished_) {
			RPI_LOG("AWB thread finished");
			fetchAsyncResults();
		}
	}
	// Finally apply IIR filter to results and put into metadata.
	memcpy(prev_sync_results_.mode, sync_results_.mode,
	       sizeof(prev_sync_results_.mode));
	prev_sync_results_.temperature_K =
		speed * sync_results_.temperature_K +
		(1.0 - speed) * prev_sync_results_.temperature_K;
	prev_sync_results_.gain_r = speed * sync_results_.gain_r +
				    (1.0 - speed) * prev_sync_results_.gain_r;
	prev_sync_results_.gain_g = speed * sync_results_.gain_g +
				    (1.0 - speed) * prev_sync_results_.gain_g;
	prev_sync_results_.gain_b = speed * sync_results_.gain_b +
				    (1.0 - speed) * prev_sync_results_.gain_b;
	image_metadata->Set("awb.status", prev_sync_results_);
	RPI_LOG("Using AWB gains r " << prev_sync_results_.gain_r << " g "
				     << prev_sync_results_.gain_g << " b "
				     << prev_sync_results_.gain_b);
}

void Awb::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	// Count frames since we last poked the async thread.
	if (frame_phase_ < (int)config_.frame_period)
		frame_phase_++;
	if (frame_count2_ < (int)config_.startup_frames)
		frame_count2_++;
	RPI_LOG("Awb: frame_phase " << frame_phase_);
	if (frame_phase_ >= (int)config_.frame_period ||
	    frame_count2_ < (int)config_.startup_frames) {
		// Update any settings and any image metadata that we need.
		std::string mode_name;
		{
			std::unique_lock<std::mutex> lock(settings_mutex_);
			mode_name = mode_name_;
		}
		struct LuxStatus lux_status = {};
		lux_status.lux = 400; // in case no metadata
		if (image_metadata->Get("lux.status", lux_status) != 0)
			RPI_LOG("No lux metadata found");
		RPI_LOG("Awb lux value is " << lux_status.lux);

		std::unique_lock<std::mutex> lock(mutex_);
		if (async_started_ == false) {
			RPI_LOG("AWB thread starting");
			restartAsync(stats, mode_name, lux_status.lux);
		}
	}
}

void Awb::asyncFunc()
{
	while (true) {
		{
			std::unique_lock<std::mutex> lock(mutex_);
			async_signal_.wait(lock, [&] {
				return async_start_ || async_abort_;
			});
			async_start_ = false;
			if (async_abort_)
				break;
		}
		doAwb();
		{
			std::lock_guard<std::mutex> lock(mutex_);
			async_finished_ = true;
			sync_signal_.notify_one();
		}
	}
}

static void generate_stats(std::vector<Awb::RGB> &zones,
			   bcm2835_isp_stats_region *stats, double min_pixels,
			   double min_G)
{
	for (int i = 0; i < AWB_STATS_SIZE_X * AWB_STATS_SIZE_Y; i++) {
		Awb::RGB zone; // this is "invalid", unless R gets overwritten later
		double counted = stats[i].counted;
		if (counted >= min_pixels) {
			zone.G = stats[i].g_sum / counted;
			if (zone.G >= min_G) {
				zone.R = stats[i].r_sum / counted;
				zone.B = stats[i].b_sum / counted;
			}
		}
		zones.push_back(zone);
	}
}

void Awb::prepareStats()
{
	zones_.clear();
	// LSC has already been applied to the stats in this pipeline, so stop
	// any LSC compensation.  We also ignore config_.fast in this version.
	generate_stats(zones_, statistics_->awb_stats, config_.min_pixels,
		       config_.min_G);
	// we're done with these; we may as well relinquish our hold on the
	// pointer.
	statistics_.reset();
	// apply sensitivities, so values appear to come from our "canonical"
	// sensor.
	for (auto &zone : zones_)
		zone.R *= config_.sensitivity_r,
			zone.B *= config_.sensitivity_b;
}

double Awb::computeDelta2Sum(double gain_r, double gain_b)
{
	// Compute the sum of the squared colour error (non-greyness) as it
	// appears in the log likelihood equation.
	double delta2_sum = 0;
	for (auto &z : zones_) {
		double delta_r = gain_r * z.R - 1 - config_.whitepoint_r;
		double delta_b = gain_b * z.B - 1 - config_.whitepoint_b;
		double delta2 = delta_r * delta_r + delta_b * delta_b;
		//RPI_LOG("delta_r " << delta_r << " delta_b " << delta_b << " delta2 " << delta2);
		delta2 = std::min(delta2, config_.delta_limit);
		delta2_sum += delta2;
	}
	return delta2_sum;
}

Pwl Awb::interpolatePrior()
{
	// Interpolate the prior log likelihood function for our current lux
	// value.
	if (lux_ <= config_.priors.front().lux)
		return config_.priors.front().prior;
	else if (lux_ >= config_.priors.back().lux)
		return config_.priors.back().prior;
	else {
		int idx = 0;
		// find which two we lie between
		while (config_.priors[idx + 1].lux < lux_)
			idx++;
		double lux0 = config_.priors[idx].lux,
		       lux1 = config_.priors[idx + 1].lux;
		return Pwl::Combine(config_.priors[idx].prior,
				    config_.priors[idx + 1].prior,
				    [&](double /*x*/, double y0, double y1) {
					    return y0 + (y1 - y0) *
							(lux_ - lux0) / (lux1 - lux0);
				    });
	}
}

static double interpolate_quadatric(Pwl::Point const &A, Pwl::Point const &B,
				    Pwl::Point const &C)
{
	// Given 3 points on a curve, find the extremum of the function in that
	// interval by fitting a quadratic.
	const double eps = 1e-3;
	Pwl::Point CA = C - A, BA = B - A;
	double denominator = 2 * (BA.y * CA.x - CA.y * BA.x);
	if (abs(denominator) > eps) {
		double numerator = BA.y * CA.x * CA.x - CA.y * BA.x * BA.x;
		double result = numerator / denominator + A.x;
		return std::max(A.x, std::min(C.x, result));
	}
	// has degenerated to straight line segment
	return A.y < C.y - eps ? A.x : (C.y < A.y - eps ? C.x : B.x);
}

double Awb::coarseSearch(Pwl const &prior)
{
	points_.clear(); // assume doesn't deallocate memory
	size_t best_point = 0;
	double t = mode_->ct_lo;
	int span_r = 0, span_b = 0;
	// Step down the CT curve evaluating log likelihood.
	while (true) {
		double r = config_.ct_r.Eval(t, &span_r);
		double b = config_.ct_b.Eval(t, &span_b);
		double gain_r = 1 / r, gain_b = 1 / b;
		double delta2_sum = computeDelta2Sum(gain_r, gain_b);
		double prior_log_likelihood =
			prior.Eval(prior.Domain().Clip(t));
		double final_log_likelihood = delta2_sum - prior_log_likelihood;
		RPI_LOG("t: " << t << " gain_r " << gain_r << " gain_b "
			      << gain_b << " delta2_sum " << delta2_sum
			      << " prior " << prior_log_likelihood << " final "
			      << final_log_likelihood);
		points_.push_back(Pwl::Point(t, final_log_likelihood));
		if (points_.back().y < points_[best_point].y)
			best_point = points_.size() - 1;
		if (t == mode_->ct_hi)
			break;
		// for even steps along the r/b curve scale them by the current t
		t = std::min(t + t / 10 * config_.coarse_step,
			     mode_->ct_hi);
	}
	t = points_[best_point].x;
	RPI_LOG("Coarse search found CT " << t);
	// We have the best point of the search, but refine it with a quadratic
	// interpolation around its neighbours.
	if (points_.size() > 2) {
		unsigned long bp = std::min(best_point, points_.size() - 2);
		best_point = std::max(1UL, bp);
		t = interpolate_quadatric(points_[best_point - 1],
					  points_[best_point],
					  points_[best_point + 1]);
		RPI_LOG("After quadratic refinement, coarse search has CT "
			<< t);
	}
	return t;
}

void Awb::fineSearch(double &t, double &r, double &b, Pwl const &prior)
{
	int span_r, span_b;
	config_.ct_r.Eval(t, &span_r);
	config_.ct_b.Eval(t, &span_b);
	double step = t / 10 * config_.coarse_step * 0.1;
	int nsteps = 5;
	double r_diff = config_.ct_r.Eval(t + nsteps * step, &span_r) -
			config_.ct_r.Eval(t - nsteps * step, &span_r);
	double b_diff = config_.ct_b.Eval(t + nsteps * step, &span_b) -
			config_.ct_b.Eval(t - nsteps * step, &span_b);
	Pwl::Point transverse(b_diff, -r_diff);
	if (transverse.Len2() < 1e-6)
		return;
	// unit vector orthogonal to the b vs. r function (pointing outwards
	// with r and b increasing)
	transverse = transverse / transverse.Len();
	double best_log_likelihood = 0, best_t = 0, best_r = 0, best_b = 0;
	double transverse_range =
		config_.transverse_neg + config_.transverse_pos;
	const int MAX_NUM_DELTAS = 12;
	// a transverse step approximately every 0.01 r/b units
	int num_deltas = floor(transverse_range * 100 + 0.5) + 1;
	num_deltas = num_deltas < 3 ? 3 :
		     (num_deltas > MAX_NUM_DELTAS ? MAX_NUM_DELTAS : num_deltas);
	// Step down CT curve. March a bit further if the transverse range is
	// large.
	nsteps += num_deltas;
	for (int i = -nsteps; i <= nsteps; i++) {
		double t_test = t + i * step;
		double prior_log_likelihood =
			prior.Eval(prior.Domain().Clip(t_test));
		double r_curve = config_.ct_r.Eval(t_test, &span_r);
		double b_curve = config_.ct_b.Eval(t_test, &span_b);
		// x will be distance off the curve, y the log likelihood there
		Pwl::Point points[MAX_NUM_DELTAS];
		int best_point = 0;
		// Take some measurements transversely *off* the CT curve.
		for (int j = 0; j < num_deltas; j++) {
			points[j].x = -config_.transverse_neg +
				      (transverse_range * j) / (num_deltas - 1);
			Pwl::Point rb_test = Pwl::Point(r_curve, b_curve) +
					     transverse * points[j].x;
			double r_test = rb_test.x, b_test = rb_test.y;
			double gain_r = 1 / r_test, gain_b = 1 / b_test;
			double delta2_sum = computeDelta2Sum(gain_r, gain_b);
			points[j].y = delta2_sum - prior_log_likelihood;
			RPI_LOG("At t " << t_test << " r " << r_test << " b "
					<< b_test << ": " << points[j].y);
			if (points[j].y < points[best_point].y)
				best_point = j;
		}
		// We have NUM_DELTAS points transversely across the CT curve,
		// now let's do a quadratic interpolation for the best result.
		best_point = std::max(1, std::min(best_point, num_deltas - 2));
		Pwl::Point rb_test =
			Pwl::Point(r_curve, b_curve) +
			transverse *
				interpolate_quadatric(points[best_point - 1],
						      points[best_point],
						      points[best_point + 1]);
		double r_test = rb_test.x, b_test = rb_test.y;
		double gain_r = 1 / r_test, gain_b = 1 / b_test;
		double delta2_sum = computeDelta2Sum(gain_r, gain_b);
		double final_log_likelihood = delta2_sum - prior_log_likelihood;
		RPI_LOG("Finally "
			<< t_test << " r " << r_test << " b " << b_test << ": "
			<< final_log_likelihood
			<< (final_log_likelihood < best_log_likelihood ? " BEST"
								       : ""));
		if (best_t == 0 || final_log_likelihood < best_log_likelihood)
			best_log_likelihood = final_log_likelihood,
			best_t = t_test, best_r = r_test, best_b = b_test;
	}
	t = best_t, r = best_r, b = best_b;
	RPI_LOG("Fine search found t " << t << " r " << r << " b " << b);
}

void Awb::awbBayes()
{
	// May as well divide out G to save computeDelta2Sum from doing it over
	// and over.
	for (auto &z : zones_)
		z.R = z.R / (z.G + 1), z.B = z.B / (z.G + 1);
	// Get the current prior, and scale according to how many zones are
	// valid... not entirely sure about this.
	Pwl prior = interpolatePrior();
	prior *= zones_.size() / (double)(AWB_STATS_SIZE_X * AWB_STATS_SIZE_Y);
	prior.Map([](double x, double y) {
		RPI_LOG("(" << x << "," << y << ")");
	});
	double t = coarseSearch(prior);
	double r = config_.ct_r.Eval(t);
	double b = config_.ct_b.Eval(t);
	RPI_LOG("After coarse search: r " << r << " b " << b << " (gains r "
					  << 1 / r << " b " << 1 / b << ")");
	// Not entirely sure how to handle the fine search yet. Mostly the
	// estimated CT is already good enough, but the fine search allows us to
	// wander transverely off the CT curve. Under some illuminants, where
	// there may be more or less green light, this may prove beneficial,
	// though I probably need more real datasets before deciding exactly how
	// this should be controlled and tuned.
	fineSearch(t, r, b, prior);
	RPI_LOG("After fine search: r " << r << " b " << b << " (gains r "
					<< 1 / r << " b " << 1 / b << ")");
	// Write results out for the main thread to pick up. Remember to adjust
	// the gains from the ones that the "canonical sensor" would require to
	// the ones needed by *this* sensor.
	async_results_.temperature_K = t;
	async_results_.gain_r = 1.0 / r * config_.sensitivity_r;
	async_results_.gain_g = 1.0;
	async_results_.gain_b = 1.0 / b * config_.sensitivity_b;
}

void Awb::awbGrey()
{
	RPI_LOG("Grey world AWB");
	// Make a separate list of the derivatives for each of red and blue, so
	// that we can sort them to exclude the extreme gains.  We could
	// consider some variations, such as normalising all the zones first, or
	// doing an L2 average etc.
	std::vector<RGB> &derivs_R(zones_);
	std::vector<RGB> derivs_B(derivs_R);
	std::sort(derivs_R.begin(), derivs_R.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.R < b.G * a.R;
		  });
	std::sort(derivs_B.begin(), derivs_B.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.B < b.G * a.B;
		  });
	// Average the middle half of the values.
	int discard = derivs_R.size() / 4;
	RGB sum_R(0, 0, 0), sum_B(0, 0, 0);
	for (auto ri = derivs_R.begin() + discard,
		  bi = derivs_B.begin() + discard;
	     ri != derivs_R.end() - discard; ri++, bi++)
		sum_R += *ri, sum_B += *bi;
	double gain_r = sum_R.G / (sum_R.R + 1),
	       gain_b = sum_B.G / (sum_B.B + 1);
	async_results_.temperature_K = 4500; // don't know what it is
	async_results_.gain_r = gain_r;
	async_results_.gain_g = 1.0;
	async_results_.gain_b = gain_b;
}

void Awb::doAwb()
{
	if (manual_r_ != 0.0 && manual_b_ != 0.0) {
		async_results_.temperature_K = 4500; // don't know what it is
		async_results_.gain_r = manual_r_;
		async_results_.gain_g = 1.0;
		async_results_.gain_b = manual_b_;
		RPI_LOG("Using manual white balance: gain_r "
			<< async_results_.gain_r << " gain_b "
			<< async_results_.gain_b);
	} else {
		prepareStats();
		RPI_LOG("Valid zones: " << zones_.size());
		if (zones_.size() > config_.min_regions) {
			if (config_.bayes)
				awbBayes();
			else
				awbGrey();
			RPI_LOG("CT found is "
				<< async_results_.temperature_K
				<< " with gains r " << async_results_.gain_r
				<< " and b " << async_results_.gain_b);
		}
	}
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Awb(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
