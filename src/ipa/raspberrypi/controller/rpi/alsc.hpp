/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * alsc.hpp - ALSC (auto lens shading correction) control algorithm
 */
#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>

#include "../algorithm.hpp"
#include "../alsc_status.h"

namespace RPi {

// Algorithm to generate automagic LSC (Lens Shading Correction) tables.

struct AlscCalibration {
	double ct;
	double table[ALSC_CELLS_X * ALSC_CELLS_Y];
};

struct AlscConfig {
	// Only repeat the ALSC calculation every "this many" frames
	uint16_t frame_period;
	// number of initial frames for which speed taken as 1.0 (maximum)
	uint16_t startup_frames;
	// IIR filter speed applied to algorithm results
	double speed;
	double sigma_Cr;
	double sigma_Cb;
	double min_count;
	uint16_t min_G;
	double omega;
	uint32_t n_iter;
	double luminance_lut[ALSC_CELLS_X * ALSC_CELLS_Y];
	double luminance_strength;
	std::vector<AlscCalibration> calibrations_Cr;
	std::vector<AlscCalibration> calibrations_Cb;
	double default_ct; // colour temperature if no metadata found
	double threshold; // iteration termination threshold
};

class Alsc : public Algorithm
{
public:
	Alsc(Controller *controller = NULL);
	~Alsc();
	char const *Name() const override;
	void Initialise() override;
	void SwitchMode(CameraMode const &camera_mode) override;
	void Read(boost::property_tree::ptree const &params) override;
	void Prepare(Metadata *image_metadata) override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;

private:
	// configuration is read-only, and available to both threads
	AlscConfig config_;
	bool first_time_;
	std::atomic<CameraMode> camera_mode_;
	std::thread async_thread_;
	void asyncFunc(); // asynchronous thread function
	std::mutex mutex_;
	CameraMode async_camera_mode_;
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
	// counts up to startup_frames
	int frame_count_;
	// counts up to startup_frames for Process method
	int frame_count2_;
	double sync_results_[3][ALSC_CELLS_Y][ALSC_CELLS_X];
	double prev_sync_results_[3][ALSC_CELLS_Y][ALSC_CELLS_X];
	// The following are for the asynchronous thread to use, though the main
	// thread can set/reset them if the async thread is known to be idle:
	void restartAsync(StatisticsPtr &stats, Metadata *image_metadata);
	// copy out the results from the async thread so that it can be restarted
	void fetchAsyncResults();
	double ct_;
	bcm2835_isp_stats_region statistics_[ALSC_CELLS_Y * ALSC_CELLS_X];
	double async_results_[3][ALSC_CELLS_Y][ALSC_CELLS_X];
	double async_lambda_r_[ALSC_CELLS_X * ALSC_CELLS_Y];
	double async_lambda_b_[ALSC_CELLS_X * ALSC_CELLS_Y];
	void doAlsc();
	double lambda_r_[ALSC_CELLS_X * ALSC_CELLS_Y];
	double lambda_b_[ALSC_CELLS_X * ALSC_CELLS_Y];
};

} // namespace RPi
