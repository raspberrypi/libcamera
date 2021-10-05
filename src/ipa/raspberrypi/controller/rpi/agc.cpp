/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc.cpp - AGC/AEC control algorithm
 */

#include <map>

#include <linux/bcm2835-isp.h>

#include <libcamera/base/log.h>

#include "../awb_status.h"
#include "../device_status.h"
#include "../histogram.hpp"
#include "../lux_status.h"
#include "../metadata.hpp"

#include "agc.hpp"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(RPiAgc)

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

static int read_list(std::vector<double> &list,
		     boost::property_tree::ptree const &params)
{
	for (auto &p : params)
		list.push_back(p.second.get_value<double>());
	return list.size();
}

static int read_list(std::vector<Duration> &list,
		     boost::property_tree::ptree const &params)
{
	for (auto &p : params)
		list.push_back(p.second.get_value<double>() * 1us);
	return list.size();
}

void AgcExposureMode::Read(boost::property_tree::ptree const &params)
{
	int num_shutters = read_list(shutter, params.get_child("shutter"));
	int num_ags = read_list(gain, params.get_child("gain"));
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
	LOG(RPiAgc, Debug) << "AgcConfig";
	default_metering_mode = read_metering_modes(
		metering_modes, params.get_child("metering_modes"));
	default_exposure_mode = read_exposure_modes(
		exposure_modes, params.get_child("exposure_modes"));
	default_constraint_mode = read_constraint_modes(
		constraint_modes, params.get_child("constraint_modes"));
	Y_target.Read(params.get_child("y_target"));
	speed = params.get<double>("speed", 0.2);
	startup_frames = params.get<uint16_t>("startup_frames", 10);
	convergence_frames = params.get<unsigned int>("convergence_frames", 6);
	fast_reduce_threshold =
		params.get<double>("fast_reduce_threshold", 0.4);
	base_ev = params.get<double>("base_ev", 1.0);
	// Start with quite a low value as ramping up is easier than ramping down.
	default_exposure_time = params.get<double>("default_exposure_time", 1000) * 1us;
	default_analogue_gain = params.get<double>("default_analogue_gain", 1.0);
}

Agc::ExposureValues::ExposureValues()
	: shutter(0s), analogue_gain(0),
	  total_exposure(0s), total_exposure_no_dg(0s)
{
}

Agc::Agc(Controller *controller)
	: AgcAlgorithm(controller), metering_mode_(nullptr),
	  exposure_mode_(nullptr), constraint_mode_(nullptr),
	  frame_count_(0), lock_count_(0),
	  last_target_exposure_(0s), last_sensitivity_(0.0),
	  ev_(1.0), flicker_period_(0s),
	  max_shutter_(0s), fixed_shutter_(0s), fixed_analogue_gain_(0.0)
{
	memset(&awb_, 0, sizeof(awb_));
	// Setting status_.total_exposure_value_ to zero initially tells us
	// it's not been calculated yet (i.e. Process hasn't yet run).
	memset(&status_, 0, sizeof(status_));
	status_.ev = ev_;
}

char const *Agc::Name() const
{
	return NAME;
}

void Agc::Read(boost::property_tree::ptree const &params)
{
	LOG(RPiAgc, Debug) << "Agc";
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
	// Set up the "last shutter/gain" values, in case AGC starts "disabled".
	status_.shutter_time = config_.default_exposure_time;
	status_.analogue_gain = config_.default_analogue_gain;
}

bool Agc::IsPaused() const
{
	return false;
}

void Agc::Pause()
{
	fixed_shutter_ = status_.shutter_time;
	fixed_analogue_gain_ = status_.analogue_gain;
}

void Agc::Resume()
{
	fixed_shutter_ = 0s;
	fixed_analogue_gain_ = 0;
}

unsigned int Agc::GetConvergenceFrames() const
{
	// If shutter and gain have been explicitly set, there is no
	// convergence to happen, so no need to drop any frames - return zero.
	if (fixed_shutter_ && fixed_analogue_gain_)
		return 0;
	else
		return config_.convergence_frames;
}

void Agc::SetEv(double ev)
{
	ev_ = ev;
}

void Agc::SetFlickerPeriod(Duration flicker_period)
{
	flicker_period_ = flicker_period;
}

void Agc::SetMaxShutter(Duration max_shutter)
{
	max_shutter_ = max_shutter;
}

void Agc::SetFixedShutter(Duration fixed_shutter)
{
	fixed_shutter_ = fixed_shutter;
	// Set this in case someone calls Pause() straight after.
	status_.shutter_time = clipShutter(fixed_shutter_);
}

void Agc::SetFixedAnalogueGain(double fixed_analogue_gain)
{
	fixed_analogue_gain_ = fixed_analogue_gain;
	// Set this in case someone calls Pause() straight after.
	status_.analogue_gain = fixed_analogue_gain;
}

void Agc::SetMeteringMode(std::string const &metering_mode_name)
{
	metering_mode_name_ = metering_mode_name;
}

void Agc::SetExposureMode(std::string const &exposure_mode_name)
{
	exposure_mode_name_ = exposure_mode_name;
}

void Agc::SetConstraintMode(std::string const &constraint_mode_name)
{
	constraint_mode_name_ = constraint_mode_name;
}

void Agc::SwitchMode(CameraMode const &camera_mode,
		     Metadata *metadata)
{
	/* AGC expects the mode sensitivity always to be non-zero. */
	ASSERT(camera_mode.sensitivity);

	housekeepConfig();

	Duration fixed_shutter = clipShutter(fixed_shutter_);
	if (fixed_shutter && fixed_analogue_gain_) {
		// We're going to reset the algorithm here with these fixed values.

		fetchAwbStatus(metadata);
		double min_colour_gain = std::min({ awb_.gain_r, awb_.gain_g, awb_.gain_b, 1.0 });
		ASSERT(min_colour_gain != 0.0);

		// This is the equivalent of computeTargetExposure and applyDigitalGain.
		target_.total_exposure_no_dg = fixed_shutter * fixed_analogue_gain_;
		target_.total_exposure = target_.total_exposure_no_dg / min_colour_gain;

		// Equivalent of filterExposure. This resets any "history".
		filtered_ = target_;

		// Equivalent of divideUpExposure.
		filtered_.shutter = fixed_shutter;
		filtered_.analogue_gain = fixed_analogue_gain_;
	} else if (status_.total_exposure_value) {
		// On a mode switch, various things could happen:
		// - the exposure profile might change
		// - a fixed exposure or gain might be set
		// - the new mode's sensitivity might be different
		// We cope with the last of these by scaling the target values. After
		// that we just need to re-divide the exposure/gain according to the
		// current exposure profile, which takes care of everything else.

		double ratio = last_sensitivity_ / camera_mode.sensitivity;
		target_.total_exposure_no_dg *= ratio;
		target_.total_exposure *= ratio;
		filtered_.total_exposure_no_dg *= ratio;
		filtered_.total_exposure *= ratio;

		divideUpExposure();
	} else {
		// We come through here on startup, when at least one of the shutter
		// or gain has not been fixed. We must still write those values out so
		// that they will be applied immediately. We supply some arbitrary defaults
		// for any that weren't set.

		// Equivalent of divideUpExposure.
		filtered_.shutter = fixed_shutter ? fixed_shutter : config_.default_exposure_time;
		filtered_.analogue_gain = fixed_analogue_gain_ ? fixed_analogue_gain_ : config_.default_analogue_gain;
	}

	writeAndFinish(metadata, false);

	// We must remember the sensitivity of this mode for the next SwitchMode.
	last_sensitivity_ = camera_mode.sensitivity;
}

void Agc::Prepare(Metadata *image_metadata)
{
	status_.digital_gain = 1.0;
	fetchAwbStatus(image_metadata); // always fetch it so that Process knows it's been done

	if (status_.total_exposure_value) {
		// Process has run, so we have meaningful values.
		DeviceStatus device_status;
		if (image_metadata->Get("device.status", device_status) == 0) {
			Duration actual_exposure = device_status.shutter_speed *
						   device_status.analogue_gain;
			if (actual_exposure) {
				status_.digital_gain =
					status_.total_exposure_value /
					actual_exposure;
				LOG(RPiAgc, Debug) << "Want total exposure " << status_.total_exposure_value;
				// Never ask for a gain < 1.0, and also impose
				// some upper limit. Make it customisable?
				status_.digital_gain = std::max(
					1.0,
					std::min(status_.digital_gain, 4.0));
				LOG(RPiAgc, Debug) << "Actual exposure " << actual_exposure;
				LOG(RPiAgc, Debug) << "Use digital_gain " << status_.digital_gain;
				LOG(RPiAgc, Debug) << "Effective exposure "
						   << actual_exposure * status_.digital_gain;
				// Decide whether AEC/AGC has converged.
				updateLockStatus(device_status);
			}
		} else
			LOG(RPiAgc, Warning) << Name() << ": no device metadata";
		image_metadata->Set("agc.status", status_);
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
	bool desaturate = applyDigitalGain(gain, target_Y);
	// The results have to be filtered so as not to change too rapidly.
	filterExposure(desaturate);
	// The last thing is to divide up the exposure value into a shutter time
	// and analogue_gain, according to the current exposure mode.
	divideUpExposure();
	// Finally advertise what we've done.
	writeAndFinish(image_metadata, desaturate);
}

void Agc::updateLockStatus(DeviceStatus const &device_status)
{
	const double ERROR_FACTOR = 0.10; // make these customisable?
	const int MAX_LOCK_COUNT = 5;
	// Reset "lock count" when we exceed this multiple of ERROR_FACTOR
	const double RESET_MARGIN = 1.5;

	// Add 200us to the exposure time error to allow for line quantisation.
	Duration exposure_error = last_device_status_.shutter_speed * ERROR_FACTOR + 200us;
	double gain_error = last_device_status_.analogue_gain * ERROR_FACTOR;
	Duration target_error = last_target_exposure_ * ERROR_FACTOR;

	// Note that we don't know the exposure/gain limits of the sensor, so
	// the values we keep requesting may be unachievable. For this reason
	// we only insist that we're close to values in the past few frames.
	if (device_status.shutter_speed > last_device_status_.shutter_speed - exposure_error &&
	    device_status.shutter_speed < last_device_status_.shutter_speed + exposure_error &&
	    device_status.analogue_gain > last_device_status_.analogue_gain - gain_error &&
	    device_status.analogue_gain < last_device_status_.analogue_gain + gain_error &&
	    status_.target_exposure_value > last_target_exposure_ - target_error &&
	    status_.target_exposure_value < last_target_exposure_ + target_error)
		lock_count_ = std::min(lock_count_ + 1, MAX_LOCK_COUNT);
	else if (device_status.shutter_speed < last_device_status_.shutter_speed - RESET_MARGIN * exposure_error ||
		 device_status.shutter_speed > last_device_status_.shutter_speed + RESET_MARGIN * exposure_error ||
		 device_status.analogue_gain < last_device_status_.analogue_gain - RESET_MARGIN * gain_error ||
		 device_status.analogue_gain > last_device_status_.analogue_gain + RESET_MARGIN * gain_error ||
		 status_.target_exposure_value < last_target_exposure_ - RESET_MARGIN * target_error ||
		 status_.target_exposure_value > last_target_exposure_ + RESET_MARGIN * target_error)
		lock_count_ = 0;

	last_device_status_ = device_status;
	last_target_exposure_ = status_.target_exposure_value;

	LOG(RPiAgc, Debug) << "Lock count updated to " << lock_count_;
	status_.locked = lock_count_ == MAX_LOCK_COUNT;
}

static void copy_string(std::string const &s, char *d, size_t size)
{
	size_t length = s.copy(d, size - 1);
	d[length] = '\0';
}

void Agc::housekeepConfig()
{
	// First fetch all the up-to-date settings, so no one else has to do it.
	status_.ev = ev_;
	status_.fixed_shutter = clipShutter(fixed_shutter_);
	status_.fixed_analogue_gain = fixed_analogue_gain_;
	status_.flicker_period = flicker_period_;
	LOG(RPiAgc, Debug) << "ev " << status_.ev << " fixed_shutter "
			   << status_.fixed_shutter << " fixed_analogue_gain "
			   << status_.fixed_analogue_gain;
	// Make sure the "mode" pointers point to the up-to-date things, if
	// they've changed.
	if (strcmp(metering_mode_name_.c_str(), status_.metering_mode)) {
		auto it = config_.metering_modes.find(metering_mode_name_);
		if (it == config_.metering_modes.end())
			throw std::runtime_error("Agc: no metering mode " +
						 metering_mode_name_);
		metering_mode_ = &it->second;
		copy_string(metering_mode_name_, status_.metering_mode,
			    sizeof(status_.metering_mode));
	}
	if (strcmp(exposure_mode_name_.c_str(), status_.exposure_mode)) {
		auto it = config_.exposure_modes.find(exposure_mode_name_);
		if (it == config_.exposure_modes.end())
			throw std::runtime_error("Agc: no exposure profile " +
						 exposure_mode_name_);
		exposure_mode_ = &it->second;
		copy_string(exposure_mode_name_, status_.exposure_mode,
			    sizeof(status_.exposure_mode));
	}
	if (strcmp(constraint_mode_name_.c_str(), status_.constraint_mode)) {
		auto it =
			config_.constraint_modes.find(constraint_mode_name_);
		if (it == config_.constraint_modes.end())
			throw std::runtime_error("Agc: no constraint list " +
						 constraint_mode_name_);
		constraint_mode_ = &it->second;
		copy_string(constraint_mode_name_, status_.constraint_mode,
			    sizeof(status_.constraint_mode));
	}
	LOG(RPiAgc, Debug) << "exposure_mode "
			   << exposure_mode_name_ << " constraint_mode "
			   << constraint_mode_name_ << " metering_mode "
			   << metering_mode_name_;
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
	current_.total_exposure = agc_status ? agc_status->total_exposure_value : 0s;
	current_.total_exposure_no_dg = current_.shutter * current_.analogue_gain;
}

void Agc::fetchAwbStatus(Metadata *image_metadata)
{
	awb_.gain_r = 1.0; // in case not found in metadata
	awb_.gain_g = 1.0;
	awb_.gain_b = 1.0;
	if (image_metadata->Get("awb.status", awb_) != 0)
		LOG(RPiAgc, Debug) << "Agc: no AWB status found";
}

static double compute_initial_Y(bcm2835_isp_stats *stats, AwbStatus const &awb,
				double weights[], double gain)
{
	bcm2835_isp_stats_region *regions = stats->agc_stats;
	// Note how the calculation below means that equal weights give you
	// "average" metering (i.e. all pixels equally important).
	double R_sum = 0, G_sum = 0, B_sum = 0, pixel_sum = 0;
	for (int i = 0; i < AGC_STATS_SIZE; i++) {
		double counted = regions[i].counted;
		double r_sum = std::min(regions[i].r_sum * gain, ((1 << PIPELINE_BITS) - 1) * counted);
		double g_sum = std::min(regions[i].g_sum * gain, ((1 << PIPELINE_BITS) - 1) * counted);
		double b_sum = std::min(regions[i].b_sum * gain, ((1 << PIPELINE_BITS) - 1) * counted);
		R_sum += r_sum * weights[i];
		G_sum += g_sum * weights[i];
		B_sum += b_sum * weights[i];
		pixel_sum += counted * weights[i];
	}
	if (pixel_sum == 0.0) {
		LOG(RPiAgc, Warning) << "compute_initial_Y: pixel_sum is zero";
		return 0;
	}
	double Y_sum = R_sum * awb.gain_r * .299 +
		       G_sum * awb.gain_g * .587 +
		       B_sum * awb.gain_b * .114;
	return Y_sum / pixel_sum / (1 << PIPELINE_BITS);
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
		LOG(RPiAgc, Warning) << "Agc: no lux level found";
	Histogram h(statistics->hist[0].g_hist, NUM_HISTOGRAM_BINS);
	double ev_gain = status_.ev * config_.base_ev;
	// The initial gain and target_Y come from some of the regions. After
	// that we consider the histogram constraints.
	target_Y =
		config_.Y_target.Eval(config_.Y_target.Domain().Clip(lux.lux));
	target_Y = std::min(EV_GAIN_Y_TARGET_LIMIT, target_Y * ev_gain);

	// Do this calculation a few times as brightness increase can be
	// non-linear when there are saturated regions.
	gain = 1.0;
	for (int i = 0; i < 8; i++) {
		double initial_Y = compute_initial_Y(statistics, awb_,
						     metering_mode_->weights, gain);
		double extra_gain = std::min(10.0, target_Y / (initial_Y + .001));
		gain *= extra_gain;
		LOG(RPiAgc, Debug) << "Initial Y " << initial_Y << " target " << target_Y
				   << " gives gain " << gain;
		if (extra_gain < 1.01) // close enough
			break;
	}

	for (auto &c : *constraint_mode_) {
		double new_target_Y;
		double new_gain =
			constraint_compute_gain(c, h, lux.lux, ev_gain,
						new_target_Y);
		LOG(RPiAgc, Debug) << "Constraint has target_Y "
				   << new_target_Y << " giving gain " << new_gain;
		if (c.bound == AgcConstraint::Bound::LOWER &&
		    new_gain > gain) {
			LOG(RPiAgc, Debug) << "Lower bound constraint adopted";
			gain = new_gain, target_Y = new_target_Y;
		} else if (c.bound == AgcConstraint::Bound::UPPER &&
			   new_gain < gain) {
			LOG(RPiAgc, Debug) << "Upper bound constraint adopted";
			gain = new_gain, target_Y = new_target_Y;
		}
	}
	LOG(RPiAgc, Debug) << "Final gain " << gain << " (target_Y " << target_Y << " ev "
			   << status_.ev << " base_ev " << config_.base_ev
			   << ")";
}

void Agc::computeTargetExposure(double gain)
{
	if (status_.fixed_shutter && status_.fixed_analogue_gain) {
		// When ag and shutter are both fixed, we need to drive the
		// total exposure so that we end up with a digital gain of at least
		// 1/min_colour_gain. Otherwise we'd desaturate channels causing
		// white to go cyan or magenta.
		double min_colour_gain = std::min({ awb_.gain_r, awb_.gain_g, awb_.gain_b, 1.0 });
		ASSERT(min_colour_gain != 0.0);
		target_.total_exposure =
			status_.fixed_shutter * status_.fixed_analogue_gain / min_colour_gain;
	} else {
		// The statistics reflect the image without digital gain, so the final
		// total exposure we're aiming for is:
		target_.total_exposure = current_.total_exposure_no_dg * gain;
		// The final target exposure is also limited to what the exposure
		// mode allows.
		Duration max_shutter = status_.fixed_shutter
				   ? status_.fixed_shutter
				   : exposure_mode_->shutter.back();
		max_shutter = clipShutter(max_shutter);
		Duration max_total_exposure =
			max_shutter *
			(status_.fixed_analogue_gain != 0.0
				 ? status_.fixed_analogue_gain
				 : exposure_mode_->gain.back());
		target_.total_exposure = std::min(target_.total_exposure,
						  max_total_exposure);
	}
	LOG(RPiAgc, Debug) << "Target total_exposure " << target_.total_exposure;
}

bool Agc::applyDigitalGain(double gain, double target_Y)
{
	double min_colour_gain = std::min({ awb_.gain_r, awb_.gain_g, awb_.gain_b, 1.0 });
	ASSERT(min_colour_gain != 0.0);
	double dg = 1.0 / min_colour_gain;
	// I think this pipeline subtracts black level and rescales before we
	// get the stats, so no need to worry about it.
	LOG(RPiAgc, Debug) << "after AWB, target dg " << dg << " gain " << gain
			   << " target_Y " << target_Y;
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
	LOG(RPiAgc, Debug) << "Digital gain " << dg << " desaturate? " << desaturate;
	target_.total_exposure_no_dg = target_.total_exposure / dg;
	LOG(RPiAgc, Debug) << "Target total_exposure_no_dg " << target_.total_exposure_no_dg;
	return desaturate;
}

void Agc::filterExposure(bool desaturate)
{
	double speed = config_.speed;
	// AGC adapts instantly if both shutter and gain are directly specified
	// or we're in the startup phase.
	if ((status_.fixed_shutter && status_.fixed_analogue_gain) ||
	    frame_count_ <= config_.startup_frames)
		speed = 1.0;
	if (!filtered_.total_exposure) {
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
	LOG(RPiAgc, Debug) << "After filtering, total_exposure " << filtered_.total_exposure
			   << " no dg " << filtered_.total_exposure_no_dg;
}

void Agc::divideUpExposure()
{
	// Sending the fixed shutter/gain cases through the same code may seem
	// unnecessary, but it will make more sense when extend this to cover
	// variable aperture.
	Duration exposure_value = filtered_.total_exposure_no_dg;
	Duration shutter_time;
	double analogue_gain;
	shutter_time = status_.fixed_shutter
			       ? status_.fixed_shutter
			       : exposure_mode_->shutter[0];
	shutter_time = clipShutter(shutter_time);
	analogue_gain = status_.fixed_analogue_gain != 0.0
				? status_.fixed_analogue_gain
				: exposure_mode_->gain[0];
	if (shutter_time * analogue_gain < exposure_value) {
		for (unsigned int stage = 1;
		     stage < exposure_mode_->gain.size(); stage++) {
			if (!status_.fixed_shutter) {
				Duration stage_shutter =
					clipShutter(exposure_mode_->shutter[stage]);
				if (stage_shutter * analogue_gain >=
				    exposure_value) {
					shutter_time =
						exposure_value / analogue_gain;
					break;
				}
				shutter_time = stage_shutter;
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
	LOG(RPiAgc, Debug) << "Divided up shutter and gain are " << shutter_time << " and "
			   << analogue_gain;
	// Finally adjust shutter time for flicker avoidance (require both
	// shutter and gain not to be fixed).
	if (!status_.fixed_shutter && !status_.fixed_analogue_gain &&
	    status_.flicker_period) {
		int flicker_periods = shutter_time / status_.flicker_period;
		if (flicker_periods) {
			Duration new_shutter_time = flicker_periods * status_.flicker_period;
			analogue_gain *= shutter_time / new_shutter_time;
			// We should still not allow the ag to go over the
			// largest value in the exposure mode. Note that this
			// may force more of the total exposure into the digital
			// gain as a side-effect.
			analogue_gain = std::min(analogue_gain,
						 exposure_mode_->gain.back());
			shutter_time = new_shutter_time;
		}
		LOG(RPiAgc, Debug) << "After flicker avoidance, shutter "
				   << shutter_time << " gain " << analogue_gain;
	}
	filtered_.shutter = shutter_time;
	filtered_.analogue_gain = analogue_gain;
}

void Agc::writeAndFinish(Metadata *image_metadata, bool desaturate)
{
	status_.total_exposure_value = filtered_.total_exposure;
	status_.target_exposure_value = desaturate ? 0s : target_.total_exposure_no_dg;
	status_.shutter_time = filtered_.shutter;
	status_.analogue_gain = filtered_.analogue_gain;
	// Write to metadata as well, in case anyone wants to update the camera
	// immediately.
	image_metadata->Set("agc.status", status_);
	LOG(RPiAgc, Debug) << "Output written, total exposure requested is "
			   << filtered_.total_exposure;
	LOG(RPiAgc, Debug) << "Camera exposure update: shutter time " << filtered_.shutter
			   << " analogue gain " << filtered_.analogue_gain;
}

Duration Agc::clipShutter(Duration shutter)
{
	if (max_shutter_)
		shutter = std::min(shutter, max_shutter_);
	return shutter;
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Agc(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
