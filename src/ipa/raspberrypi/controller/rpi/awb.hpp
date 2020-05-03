/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb.hpp - AWB control algorithm
 */
#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>

#include "../awb_algorithm.hpp"
#include "../pwl.hpp"
#include "../awb_status.h"

namespace RPi {

// Control algorithm to perform AWB calculations.

struct AwbMode {
	void Read(boost::property_tree::ptree const &params);
	double ct_lo; // low CT value for search
	double ct_hi; // high CT value for search
};

struct AwbPrior {
	void Read(boost::property_tree::ptree const &params);
	double lux; // lux level
	Pwl prior; // maps CT to prior log likelihood for this lux level
};

struct AwbConfig {
	AwbConfig() : default_mode(nullptr) {}
	void Read(boost::property_tree::ptree const &params);
	// Only repeat the AWB calculation every "this many" frames
	uint16_t frame_period;
	// number of initial frames for which speed taken as 1.0 (maximum)
	uint16_t startup_frames;
	double speed; // IIR filter speed applied to algorithm results
	bool fast; // "fast" mode uses a 16x16 rather than 32x32 grid
	Pwl ct_r; // function maps CT to r (= R/G)
	Pwl ct_b; // function maps CT to b (= B/G)
	// table of illuminant priors at different lux levels
	std::vector<AwbPrior> priors;
	// AWB "modes" (determines the search range)
	std::map<std::string, AwbMode> modes;
	AwbMode *default_mode; // mode used if no mode selected
	// minimum proportion of pixels counted within AWB region for it to be
	// "useful"
	double min_pixels;
	// minimum G value of those pixels, to be regarded a "useful"
	uint16_t min_G;
	// number of AWB regions that must be "useful" in order to do the AWB
	// calculation
	uint32_t min_regions;
	// clamp on colour error term (so as not to penalise non-grey excessively)
	double delta_limit;
	// step size control in coarse search
	double coarse_step;
	// how far to wander off CT curve towards "more purple"
	double transverse_pos;
	// how far to wander off CT curve towards "more green"
	double transverse_neg;
	// red sensitivity ratio (set to canonical sensor's R/G divided by this
	// sensor's R/G)
	double sensitivity_r;
	// blue sensitivity ratio (set to canonical sensor's B/G divided by this
	// sensor's B/G)
	double sensitivity_b;
	// The whitepoint (which we normally "aim" for) can be moved.
	double whitepoint_r;
	double whitepoint_b;
	bool bayes; // use Bayesian algorithm
};

class Awb : public AwbAlgorithm
{
public:
	Awb(Controller *controller = NULL);
	~Awb();
	char const *Name() const override;
	void Initialise() override;
	void Read(boost::property_tree::ptree const &params) override;
	void SetMode(std::string const &name) override;
	void SetManualGains(double manual_r, double manual_b) override;
	void Prepare(Metadata *image_metadata) override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;
	struct RGB {
		RGB(double _R = INVALID, double _G = INVALID,
		    double _B = INVALID)
			: R(_R), G(_G), B(_B)
		{
		}
		double R, G, B;
		static const double INVALID;
		bool Valid() const { return G != INVALID; }
		bool Invalid() const { return G == INVALID; }
		RGB &operator+=(RGB const &other)
		{
			R += other.R, G += other.G, B += other.B;
			return *this;
		}
		RGB Square() const { return RGB(R * R, G * G, B * B); }
	};

private:
	// configuration is read-only, and available to both threads
	AwbConfig config_;
	std::thread async_thread_;
	void asyncFunc(); // asynchronous thread function
	std::mutex mutex_;
	// condvar for async thread to wait on
	std::condition_variable async_signal_;
	// condvar for synchronous thread to wait on
	std::condition_variable sync_signal_;
	// for sync thread to check  if async thread finished (requires mutex)
	bool async_finished_;
	// for async thread to check if it's been told to run (requires mutex)
	bool async_start_;
	// for async thread to check if it's been told to quit (requires mutex)
	bool async_abort_;

	// The following are only for the synchronous thread to use:
	// for sync thread to note its has asked async thread to run
	bool async_started_;
	// counts up to frame_period before restarting the async thread
	int frame_phase_;
	int frame_count_; // counts up to startup_frames
	int frame_count2_; // counts up to startup_frames for Process method
	AwbStatus sync_results_;
	AwbStatus prev_sync_results_;
	std::string mode_name_;
	std::mutex settings_mutex_;
	// The following are for the asynchronous thread to use, though the main
	// thread can set/reset them if the async thread is known to be idle:
	void restartAsync(StatisticsPtr &stats, std::string const &mode_name,
			  double lux);
	// copy out the results from the async thread so that it can be restarted
	void fetchAsyncResults();
	StatisticsPtr statistics_;
	AwbMode *mode_;
	double lux_;
	AwbStatus async_results_;
	void doAwb();
	void awbBayes();
	void awbGrey();
	void prepareStats();
	double computeDelta2Sum(double gain_r, double gain_b);
	Pwl interpolatePrior();
	double coarseSearch(Pwl const &prior);
	void fineSearch(double &t, double &r, double &b, Pwl const &prior);
	std::vector<RGB> zones_;
	std::vector<Pwl::Point> points_;
	// manual r setting
	double manual_r_;
	// manual b setting
	double manual_b_;
};

static inline Awb::RGB operator+(Awb::RGB const &a, Awb::RGB const &b)
{
	return Awb::RGB(a.R + b.R, a.G + b.G, a.B + b.B);
}
static inline Awb::RGB operator-(Awb::RGB const &a, Awb::RGB const &b)
{
	return Awb::RGB(a.R - b.R, a.G - b.G, a.B - b.B);
}
static inline Awb::RGB operator*(double d, Awb::RGB const &rgb)
{
	return Awb::RGB(d * rgb.R, d * rgb.G, d * rgb.B);
}
static inline Awb::RGB operator*(Awb::RGB const &rgb, double d)
{
	return d * rgb;
}

} // namespace RPi
