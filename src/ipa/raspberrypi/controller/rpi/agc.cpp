/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc.cpp - AGC/AEC control algorithm
 */

#include <map>

#include "linux/bcm2835-isp.h"

#include "../awb_status.h"
#include "../device_status.h"
#include "../histogram.hpp"
#include "../logging.hpp"
#include "../lux_status.h"
#include "../metadata.hpp"

#include "agc.hpp"

using namespace RPi;

#define NAME "rpi.agc"

#define PIPELINE_BITS 13 // seems to be a 13-bit pipeline

void AgcMeteringMode::Read(boost::property_tree::ptree const &params)
{
	int num = 0;
	for (auto &p : params.get_child("weights")) {
		if (num == AGC_STATS_SIZE)
			throw std::runtime_error("AgcConfig: too many weights");
		weights[num++] = p.second.get_value<double>();
	}
	if (num != AGC_STATS_SIZE)
		throw std::runtime_error("AgcConfig: insufficient weights");
}

static std::string
read_metering_modes(std::map<std::string, AgcMeteringMode> &metering_modes,
		    boost::property_tree::ptree const &params)
{
	std::string first;
	for (auto &p : params) {
		AgcMeteringMode metering_mode;
		metering_mode.Read(p.second);
		metering_modes[p.first] = std::move(metering_mode);
		if (first.empty())
			first = p.first;
	}
	return first;
}

static int read_double_list(std::vector<double> &list,
			    boost::property_tree::ptree const &params)
{
	for (auto &p : params)
		list.push_back(p.second.get_value<double>());
	return list.size();
}

void AgcExposureMode::Read(boost::property_tree::ptree const &params)
{
	int num_shutters =
		read_double_list(shutter, params.get_child("shutter"));
	int num_ags = read_double_list(gain, params.get_child("gain"));
	if (num_shutters < 2 || num_ags < 2)
		throw std::runtime_error(
			"AgcConfig: must have at least two entries in exposure profile");
	if (num_shutters != num_ags)
		throw std::runtime_error(
			"AgcConfig: expect same number of exposure and gain entries in exposure profile");
}

static std::string
read_exposure_modes(std::map<std::string, AgcExposureMode> &exposure_modes,
		    boost::property_tree::ptree const &params)
{
	std::string first;
	for (auto &p : params) {
		AgcExposureMode exposure_mode;
		exposure_mode.Read(p.second);
		exposure_modes[p.first] = std::move(exposure_mode);
		if (first.empty())
			first = p.first;
	}
	return first;
}

void AgcConstraint::Read(boost::property_tree::ptree const &params)
{
	std::string bound_string = params.get<std::string>("bound", "");
	transform(bound_string.begin(), bound_string.end(),
		  bound_string.begin(), ::toupper);
	if (bound_string != "UPPER" && bound_string != "LOWER")
		throw std::runtime_error(
			"AGC constraint type should be UPPER or LOWER");
	bound = bound_string == "UPPER" ? Bound::UPPER : Bound::LOWER;
	q_lo = params.get<double>("q_lo");
	q_hi = params.get<double>("q_hi");
	Y_target.Read(params.get_child("y_target"));
}

static AgcConstraintMode
read_constraint_mode(boost::property_tree::ptree const &params)
{
	AgcConstraintMode mode;
	for (auto &p : params) {
		AgcConstraint constraint;
		constraint.Read(p.second);
		mode.push_back(std::move(constraint));
	}
	return mode;
}

static std::string read_constraint_modes(
	std::map<std::string, AgcConstraintMode> &constraint_modes,
	boost::property_tree::ptree const &params)
{
	std::string first;
	for (auto &p : params) {
		constraint_modes[p.first] = read_constraint_mode(p.second);
		if (first.empty())
			first = p.first;
	}
	return first;
}

void AgcConfig::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG("AgcConfig");
	default_metering_mode = read_metering_modes(
		metering_modes, params.get_child("metering_modes"));
	default_exposure_mode = read_exposure_modes(
		exposure_modes, params.get_child("exposure_modes"));
	default_constraint_mode = read_constraint_modes(
		constraint_modes, params.get_child("constraint_modes"));
	Y_target.Read(params.get_child("y_target"));
	speed = params.get<double>("speed", 0.2);
	startup_frames = params.get<uint16_t>("startup_frames", 10);
	fast_reduce_threshold =
		params.get<double>("fast_reduce_threshold", 0.4);
	base_ev = params.get<double>("base_ev", 1.0);
}

Agc::Agc(Controller *controller)
	: AgcAlgorithm(controller), metering_mode_(nullptr),
	  exposure_mode_(nullptr), constraint_mode_(nullptr),
	  frame_count_(0), lock_count_(0)
{
	ev_ = status_.ev = 1.0;
	flicker_period_ = status_.flicker_period = 0.0;
	fixed_shutter_ = status_.fixed_shutter = 0;
	fixed_analogue_gain_ = status_.fixed_analogue_gain = 0.0;
	// set to zero initially, so we can tell it's not been calculated
	status_.total_exposure_value = 0.0;
	status_.target_exposure_value = 0.0;
	status_.locked = false;
	output_status_ = status_;
}

char const *Agc::Name() const
{
	return NAME;
}

void Agc::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG("Agc");
	config_.Read(params);
	// Set the config's defaults (which are the first ones it read) as our
	// current modes, until someone changes them.  (they're all known to
	// exist at this point)
	metering_mode_name_ = config_.default_metering_mode;
	metering_mode_ = &config_.metering_modes[metering_mode_name_];
	exposure_mode_name_ = config_.default_exposure_mode;
	exposure_mode_ = &config_.exposure_modes[exposure_mode_name_];
	constraint_mode_name_ = config_.default_constraint_mode;
	constraint_mode_ = &config_.constraint_modes[constraint_mode_name_];
}

void Agc::SetEv(double ev)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	ev_ = ev;
}

void Agc::SetFlickerPeriod(double flicker_period)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	flicker_period_ = flicker_period;
}

void Agc::SetFixedShutter(double fixed_shutter)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	fixed_shutter_ = fixed_shutter;
}

void Agc::SetFixedAnalogueGain(double fixed_analogue_gain)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	fixed_analogue_gain_ = fixed_analogue_gain;
}

void Agc::SetMeteringMode(std::string const &metering_mode_name)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	metering_mode_name_ = metering_mode_name;
}

void Agc::SetExposureMode(std::string const &exposure_mode_name)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	exposure_mode_name_ = exposure_mode_name;
}

void Agc::SetConstraintMode(std::string const &constraint_mode_name)
{
	std::unique_lock<std::mutex> lock(settings_mutex_);
	constraint_mode_name_ = constraint_mode_name;
}

void Agc::Prepare(Metadata *image_metadata)
{
	AgcStatus status;
	{
		std::unique_lock<std::mutex> lock(output_mutex_);
		status = output_status_;
	}
	int lock_count = lock_count_;
	lock_count_ = 0;
	status.digital_gain = 1.0;
	if (status_.total_exposure_value) {
		// Process has run, so we have meaningful values.
		DeviceStatus device_status;
		if (image_metadata->Get("device.status", device_status) == 0) {
			double actual_exposure = device_status.shutter_speed *
						 device_status.analogue_gain;
			if (actual_exposure) {
				status.digital_gain =
					status_.total_exposure_value /
					actual_exposure;
				RPI_LOG("Want total exposure " << status_.total_exposure_value);
				// Never ask for a gain < 1.0, and also impose
				// some upper limit. Make it customisable?
				status.digital_gain = std::max(
					1.0,
					std::min(status.digital_gain, 4.0));
				RPI_LOG("Actual exposure " << actual_exposure);
				RPI_LOG("Use digital_gain " << status.digital_gain);
				RPI_LOG("Effective exposure " << actual_exposure * status.digital_gain);
				// Decide whether AEC/AGC has converged.
				// Insist AGC is steady for MAX_LOCK_COUNT
				// frames before we say we are "locked".
				// (The hard-coded constants may need to
				// become customisable.)
				if (status.target_exposure_value) {
#define MAX_LOCK_COUNT 3
					double err = 0.10 * status.target_exposure_value + 200;
					if (actual_exposure <
					    status.target_exposure_value + err
					    && actual_exposure >
					    status.target_exposure_value - err)
						lock_count_ =
							std::min(lock_count + 1,
							       MAX_LOCK_COUNT);
					else if (actual_exposure <
						 status.target_exposure_value
						 + 1.5 * err &&
						 actual_exposure >
						 status.target_exposure_value
						 - 1.5 * err)
						lock_count_ = lock_count;
					RPI_LOG("Lock count: " << lock_count_);
				}
			}
		} else
			RPI_LOG(Name() << ": no device metadata");
		status.locked = lock_count_ >= MAX_LOCK_COUNT;
		//printf("%s\n", status.locked ? "+++++++++" : "-");
		image_metadata->Set("agc.status", status);
	}
}

void Agc::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	frame_count_++;
	// First a little bit of housekeeping, fetching up-to-date settings and
	// configuration, that kind of thing.
	housekeepConfig();
	// Get the current exposure values for the frame that's just arrived.
	fetchCurrentExposure(image_metadata);
	// Compute the total gain we require relative to the current exposure.
	double gain, target_Y;
	computeGain(stats.get(), image_metadata, gain, target_Y);
	// Now compute the target (final) exposure which we think we want.
	computeTargetExposure(gain);
	// Some of the exposure has to be applied as digital gain, so work out
	// what that is. This function also tells us whether it's decided to
	// "desaturate" the image more quickly.
	bool desaturate = applyDigitalGain(image_metadata, gain, target_Y);
	// The results have to be filtered so as not to change too rapidly.
	filterExposure(desaturate);
	// The last thing is to divvy up the exposure value into a shutter time
	// and analogue_gain, according to the current exposure mode.
	divvyupExposure();
	// Finally advertise what we've done.
	writeAndFinish(image_metadata, desaturate);
}

static void copy_string(std::string const &s, char *d, size_t size)
{
	size_t length = s.copy(d, size - 1);
	d[length] = '\0';
}

void Agc::housekeepConfig()
{
	// First fetch all the up-to-date settings, so no one else has to do it.
	std::string new_exposure_mode_name, new_constraint_mode_name,
		new_metering_mode_name;
	{
		std::unique_lock<std::mutex> lock(settings_mutex_);
		new_metering_mode_name = metering_mode_name_;
		new_exposure_mode_name = exposure_mode_name_;
		new_constraint_mode_name = constraint_mode_name_;
		status_.ev = ev_;
		status_.fixed_shutter = fixed_shutter_;
		status_.fixed_analogue_gain = fixed_analogue_gain_;
		status_.flicker_period = flicker_period_;
	}
	RPI_LOG("ev " << status_.ev << " fixed_shutter "
		      << status_.fixed_shutter << " fixed_analogue_gain "
		      << status_.fixed_analogue_gain);
	// Make sure the "mode" pointers point to the up-to-date things, if
	// they've changed.
	if (strcmp(new_metering_mode_name.c_str(), status_.metering_mode)) {
		auto it = config_.metering_modes.find(new_metering_mode_name);
		if (it == config_.metering_modes.end())
			throw std::runtime_error("Agc: no metering mode " +
						 new_metering_mode_name);
		metering_mode_ = &it->second;
		copy_string(new_metering_mode_name, status_.metering_mode,
			    sizeof(status_.metering_mode));
	}
	if (strcmp(new_exposure_mode_name.c_str(), status_.exposure_mode)) {
		auto it = config_.exposure_modes.find(new_exposure_mode_name);
		if (it == config_.exposure_modes.end())
			throw std::runtime_error("Agc: no exposure profile " +
						 new_exposure_mode_name);
		exposure_mode_ = &it->second;
		copy_string(new_exposure_mode_name, status_.exposure_mode,
			    sizeof(status_.exposure_mode));
	}
	if (strcmp(new_constraint_mode_name.c_str(), status_.constraint_mode)) {
		auto it =
			config_.constraint_modes.find(new_constraint_mode_name);
		if (it == config_.constraint_modes.end())
			throw std::runtime_error("Agc: no constraint list " +
						 new_constraint_mode_name);
		constraint_mode_ = &it->second;
		copy_string(new_constraint_mode_name, status_.constraint_mode,
			    sizeof(status_.constraint_mode));
	}
	RPI_LOG("exposure_mode "
		<< new_exposure_mode_name << " constraint_mode "
		<< new_constraint_mode_name << " metering_mode "
		<< new_metering_mode_name);
}

void Agc::fetchCurrentExposure(Metadata *image_metadata)
{
	std::unique_lock<Metadata> lock(*image_metadata);
	DeviceStatus *device_status =
		image_metadata->GetLocked<DeviceStatus>("device.status");
	if (!device_status)
		throw std::runtime_error("Agc: no device metadata");
	current_.shutter = device_status->shutter_speed;
	current_.analogue_gain = device_status->analogue_gain;
	AgcStatus *agc_status =
		image_metadata->GetLocked<AgcStatus>("agc.status");
	current_.total_exposure = agc_status ? agc_status->total_exposure_value : 0;
	current_.total_exposure_no_dg = current_.shutter * current_.analogue_gain;
}

static double compute_initial_Y(bcm2835_isp_stats *stats, Metadata *image_metadata,
				double weights[])
{
	bcm2835_isp_stats_region *regions = stats->agc_stats;
	struct AwbStatus awb;
	awb.gain_r = awb.gain_g = awb.gain_b = 1.0; // in case no metadata
	if (image_metadata->Get("awb.status", awb) != 0)
		RPI_WARN("Agc: no AWB status found");
	double Y_sum = 0, weight_sum = 0;
	for (int i = 0; i < AGC_STATS_SIZE; i++) {
		if (regions[i].counted == 0)
			continue;
		weight_sum += weights[i];
		double Y = regions[i].r_sum * awb.gain_r * .299 +
			   regions[i].g_sum * awb.gain_g * .587 +
			   regions[i].b_sum * awb.gain_b * .114;
		Y /= regions[i].counted;
		Y_sum += Y * weights[i];
	}
	return Y_sum / weight_sum / (1 << PIPELINE_BITS);
}

// We handle extra gain through EV by adjusting our Y targets. However, you
// simply can't monitor histograms once they get very close to (or beyond!)
// saturation, so we clamp the Y targets to this value. It does mean that EV
// increases don't necessarily do quite what you might expect in certain
// (contrived) cases.

#define EV_GAIN_Y_TARGET_LIMIT 0.9

static double constraint_compute_gain(AgcConstraint &c, Histogram &h,
				      double lux, double ev_gain,
				      double &target_Y)
{
	target_Y = c.Y_target.Eval(c.Y_target.Domain().Clip(lux));
	target_Y = std::min(EV_GAIN_Y_TARGET_LIMIT, target_Y * ev_gain);
	double iqm = h.InterQuantileMean(c.q_lo, c.q_hi);
	return (target_Y * NUM_HISTOGRAM_BINS) / iqm;
}

void Agc::computeGain(bcm2835_isp_stats *statistics, Metadata *image_metadata,
		      double &gain, double &target_Y)
{
	struct LuxStatus lux = {};
	lux.lux = 400; // default lux level to 400 in case no metadata found
	if (image_metadata->Get("lux.status", lux) != 0)
		RPI_WARN("Agc: no lux level found");
	Histogram h(statistics->hist[0].g_hist, NUM_HISTOGRAM_BINS);
	double ev_gain = status_.ev * config_.base_ev;
	// The initial gain and target_Y come from some of the regions. After
	// that we consider the histogram constraints.
	target_Y =
		config_.Y_target.Eval(config_.Y_target.Domain().Clip(lux.lux));
	target_Y = std::min(EV_GAIN_Y_TARGET_LIMIT, target_Y * ev_gain);
	double initial_Y = compute_initial_Y(statistics, image_metadata,
					     metering_mode_->weights);
	gain = std::min(10.0, target_Y / (initial_Y + .001));
	RPI_LOG("Initially Y " << initial_Y << " target " << target_Y
			       << " gives gain " << gain);
	for (auto &c : *constraint_mode_) {
		double new_target_Y;
		double new_gain =
			constraint_compute_gain(c, h, lux.lux, ev_gain,
						new_target_Y);
		RPI_LOG("Constraint has target_Y "
			<< new_target_Y << " giving gain " << new_gain);
		if (c.bound == AgcConstraint::Bound::LOWER &&
		    new_gain > gain) {
			RPI_LOG("Lower bound constraint adopted");
			gain = new_gain, target_Y = new_target_Y;
		} else if (c.bound == AgcConstraint::Bound::UPPER &&
			   new_gain < gain) {
			RPI_LOG("Upper bound constraint adopted");
			gain = new_gain, target_Y = new_target_Y;
		}
	}
	RPI_LOG("Final gain " << gain << " (target_Y " << target_Y << " ev "
			      << status_.ev << " base_ev " << config_.base_ev
			      << ")");
}

void Agc::computeTargetExposure(double gain)
{
	// The statistics reflect the image without digital gain, so the final
	// total exposure we're aiming for is:
	target_.total_exposure = current_.total_exposure_no_dg * gain;
	// The final target exposure is also limited to what the exposure
	// mode allows.
	double max_total_exposure =
		(status_.fixed_shutter != 0.0
			 ? status_.fixed_shutter
			 : exposure_mode_->shutter.back()) *
		(status_.fixed_analogue_gain != 0.0
			 ? status_.fixed_analogue_gain
			 : exposure_mode_->gain.back());
	target_.total_exposure = std::min(target_.total_exposure,
					  max_total_exposure);
	RPI_LOG("Target total_exposure " << target_.total_exposure);
}

bool Agc::applyDigitalGain(Metadata *image_metadata, double gain,
			   double target_Y)
{
	double dg = 1.0;
	// I think this pipeline subtracts black level and rescales before we
	// get the stats, so no need to worry about it.
	struct AwbStatus awb;
	if (image_metadata->Get("awb.status", awb) == 0) {
		double min_gain = std::min(awb.gain_r,
					   std::min(awb.gain_g, awb.gain_b));
		dg *= std::max(1.0, 1.0 / min_gain);
	} else
		RPI_WARN("Agc: no AWB status found");
	RPI_LOG("after AWB, target dg " << dg << " gain " << gain
					<< " target_Y " << target_Y);
	// Finally, if we're trying to reduce exposure but the target_Y is
	// "close" to 1.0, then the gain computed for that constraint will be
	// only slightly less than one, because the measured Y can never be
	// larger than 1.0. When this happens, demand a large digital gain so
	// that the exposure can be reduced, de-saturating the image much more
	// quickly (and we then approach the correct value more quickly from
	// below).
	bool desaturate = target_Y > config_.fast_reduce_threshold &&
			  gain < sqrt(target_Y);
	if (desaturate)
		dg /= config_.fast_reduce_threshold;
	RPI_LOG("Digital gain " << dg << " desaturate? " << desaturate);
	target_.total_exposure_no_dg = target_.total_exposure / dg;
	RPI_LOG("Target total_exposure_no_dg " << target_.total_exposure_no_dg);
	return desaturate;
}

void Agc::filterExposure(bool desaturate)
{
	double speed = frame_count_ <= config_.startup_frames ? 1.0 : config_.speed;
	if (filtered_.total_exposure == 0.0) {
		filtered_.total_exposure = target_.total_exposure;
		filtered_.total_exposure_no_dg = target_.total_exposure_no_dg;
	} else {
		// If close to the result go faster, to save making so many
		// micro-adjustments on the way. (Make this customisable?)
		if (filtered_.total_exposure < 1.2 * target_.total_exposure &&
		    filtered_.total_exposure > 0.8 * target_.total_exposure)
			speed = sqrt(speed);
		filtered_.total_exposure = speed * target_.total_exposure +
					   filtered_.total_exposure * (1.0 - speed);
		// When desaturing, take a big jump down in exposure_no_dg,
		// which we'll hide with digital gain.
		if (desaturate)
			filtered_.total_exposure_no_dg =
				target_.total_exposure_no_dg;
		else
			filtered_.total_exposure_no_dg =
				speed * target_.total_exposure_no_dg +
				filtered_.total_exposure_no_dg * (1.0 - speed);
	}
	// We can't let the no_dg exposure deviate too far below the
	// total exposure, as there might not be enough digital gain available
	// in the ISP to hide it (which will cause nasty oscillation).
	if (filtered_.total_exposure_no_dg <
	    filtered_.total_exposure * config_.fast_reduce_threshold)
		filtered_.total_exposure_no_dg = filtered_.total_exposure *
						 config_.fast_reduce_threshold;
	RPI_LOG("After filtering, total_exposure " << filtered_.total_exposure <<
		" no dg " << filtered_.total_exposure_no_dg);
}

void Agc::divvyupExposure()
{
	// Sending the fixed shutter/gain cases through the same code may seem
	// unnecessary, but it will make more sense when extend this to cover
	// variable aperture.
	double exposure_value = filtered_.total_exposure_no_dg;
	double shutter_time, analogue_gain;
	shutter_time = status_.fixed_shutter != 0.0
			       ? status_.fixed_shutter
			       : exposure_mode_->shutter[0];
	analogue_gain = status_.fixed_analogue_gain != 0.0
				? status_.fixed_analogue_gain
				: exposure_mode_->gain[0];
	if (shutter_time * analogue_gain < exposure_value) {
		for (unsigned int stage = 1;
		     stage < exposure_mode_->gain.size(); stage++) {
			if (status_.fixed_shutter == 0.0) {
				if (exposure_mode_->shutter[stage] *
					    analogue_gain >=
				    exposure_value) {
					shutter_time =
						exposure_value / analogue_gain;
					break;
				}
				shutter_time = exposure_mode_->shutter[stage];
			}
			if (status_.fixed_analogue_gain == 0.0) {
				if (exposure_mode_->gain[stage] *
					    shutter_time >=
				    exposure_value) {
					analogue_gain =
						exposure_value / shutter_time;
					break;
				}
				analogue_gain = exposure_mode_->gain[stage];
			}
		}
	}
	RPI_LOG("Divided up shutter and gain are " << shutter_time << " and "
						   << analogue_gain);
	// Finally adjust shutter time for flicker avoidance (require both
	// shutter and gain not to be fixed).
	if (status_.fixed_shutter == 0.0 &&
	    status_.fixed_analogue_gain == 0.0 &&
	    status_.flicker_period != 0.0) {
		int flicker_periods = shutter_time / status_.flicker_period;
		if (flicker_periods > 0) {
			double new_shutter_time = flicker_periods * status_.flicker_period;
			analogue_gain *= shutter_time / new_shutter_time;
			// We should still not allow the ag to go over the
			// largest value in the exposure mode. Note that this
			// may force more of the total exposure into the digital
			// gain as a side-effect.
			analogue_gain = std::min(analogue_gain,
						 exposure_mode_->gain.back());
			shutter_time = new_shutter_time;
		}
		RPI_LOG("After flicker avoidance, shutter "
			<< shutter_time << " gain " << analogue_gain);
	}
	filtered_.shutter = shutter_time;
	filtered_.analogue_gain = analogue_gain;
}

void Agc::writeAndFinish(Metadata *image_metadata, bool desaturate)
{
	status_.total_exposure_value = filtered_.total_exposure;
	status_.target_exposure_value = desaturate ? 0 : target_.total_exposure_no_dg;
	status_.shutter_time = filtered_.shutter;
	status_.analogue_gain = filtered_.analogue_gain;
	{
		std::unique_lock<std::mutex> lock(output_mutex_);
		output_status_ = status_;
	}
	// Write to metadata as well, in case anyone wants to update the camera
	// immediately.
	image_metadata->Set("agc.status", status_);
	RPI_LOG("Output written, total exposure requested is "
		<< filtered_.total_exposure);
	RPI_LOG("Camera exposure update: shutter time " << filtered_.shutter <<
		" analogue gain " << filtered_.analogue_gain);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Agc(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
