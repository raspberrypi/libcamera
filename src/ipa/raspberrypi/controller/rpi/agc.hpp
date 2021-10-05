/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc.hpp - AGC/AEC control algorithm
 */
#pragma once

#include <vector>
#include <mutex>

#include <libcamera/base/utils.h>

#include "../agc_algorithm.hpp"
#include "../agc_status.h"
#include "../pwl.hpp"

// This is our implementation of AGC.

// This is the number actually set up by the firmware, not the maximum possible
// number (which is 16).

#define AGC_STATS_SIZE 15

namespace RPiController {

struct AgcMeteringMode {
	double weights[AGC_STATS_SIZE];
	void Read(boost::property_tree::ptree const &params);
};

struct AgcExposureMode {
	std::vector<libcamera::utils::Duration> shutter;
	std::vector<double> gain;
	void Read(boost::property_tree::ptree const &params);
};

struct AgcConstraint {
	enum class Bound { LOWER = 0, UPPER = 1 };
	Bound bound;
	double q_lo;
	double q_hi;
	Pwl Y_target;
	void Read(boost::property_tree::ptree const &params);
};

typedef std::vector<AgcConstraint> AgcConstraintMode;

struct AgcConfig {
	void Read(boost::property_tree::ptree const &params);
	std::map<std::string, AgcMeteringMode> metering_modes;
	std::map<std::string, AgcExposureMode> exposure_modes;
	std::map<std::string, AgcConstraintMode> constraint_modes;
	Pwl Y_target;
	double speed;
	uint16_t startup_frames;
	unsigned int convergence_frames;
	double max_change;
	double min_change;
	double fast_reduce_threshold;
	double speed_up_threshold;
	std::string default_metering_mode;
	std::string default_exposure_mode;
	std::string default_constraint_mode;
	double base_ev;
	libcamera::utils::Duration default_exposure_time;
	double default_analogue_gain;
};

class Agc : public AgcAlgorithm
{
public:
	Agc(Controller *controller);
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	// AGC handles "pausing" for itself.
	bool IsPaused() const override;
	void Pause() override;
	void Resume() override;
	unsigned int GetConvergenceFrames() const override;
	void SetEv(double ev) override;
	void SetFlickerPeriod(libcamera::utils::Duration flicker_period) override;
	void SetMaxShutter(libcamera::utils::Duration max_shutter) override;
	void SetFixedShutter(libcamera::utils::Duration fixed_shutter) override;
	void SetFixedAnalogueGain(double fixed_analogue_gain) override;
	void SetMeteringMode(std::string const &metering_mode_name) override;
	void SetExposureMode(std::string const &exposure_mode_name) override;
	void SetConstraintMode(std::string const &contraint_mode_name) override;
	void SwitchMode(CameraMode const &camera_mode, Metadata *metadata) override;
	void Prepare(Metadata *image_metadata) override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;

private:
	void updateLockStatus(DeviceStatus const &device_status);
	AgcConfig config_;
	void housekeepConfig();
	void fetchCurrentExposure(Metadata *image_metadata);
	void fetchAwbStatus(Metadata *image_metadata);
	void computeGain(bcm2835_isp_stats *statistics, Metadata *image_metadata,
			 double &gain, double &target_Y);
	void computeTargetExposure(double gain);
	bool applyDigitalGain(double gain, double target_Y);
	void filterExposure(bool desaturate);
	void divideUpExposure();
	void writeAndFinish(Metadata *image_metadata, bool desaturate);
	libcamera::utils::Duration clipShutter(libcamera::utils::Duration shutter);
	AgcMeteringMode *metering_mode_;
	AgcExposureMode *exposure_mode_;
	AgcConstraintMode *constraint_mode_;
	uint64_t frame_count_;
	AwbStatus awb_;
	struct ExposureValues {
		ExposureValues();

		libcamera::utils::Duration shutter;
		double analogue_gain;
		libcamera::utils::Duration total_exposure;
		libcamera::utils::Duration total_exposure_no_dg; // without digital gain
	};
	ExposureValues current_;  // values for the current frame
	ExposureValues target_;   // calculate the values we want here
	ExposureValues filtered_; // these values are filtered towards target
	AgcStatus status_;
	int lock_count_;
	DeviceStatus last_device_status_;
	libcamera::utils::Duration last_target_exposure_;
	double last_sensitivity_; // sensitivity of the previous camera mode
	// Below here the "settings" that applications can change.
	std::string metering_mode_name_;
	std::string exposure_mode_name_;
	std::string constraint_mode_name_;
	double ev_;
	libcamera::utils::Duration flicker_period_;
	libcamera::utils::Duration max_shutter_;
	libcamera::utils::Duration fixed_shutter_;
	double fixed_analogue_gain_;
};

} // namespace RPiController
