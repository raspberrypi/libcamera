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

namespace RPiController {

/* Algorithm to generate automagic LSC (Lens Shading Correction) tables. */

struct AlscCalibration {
	double ct;
	double table[ALSC_CELLS_X * ALSC_CELLS_Y];
};

struct AlscConfig {
	/* Only repeat the ALSC calculation every "this many" frames */
	uint16_t framePeriod;
	/* number of initial frames for which speed taken as 1.0 (maximum) */
	uint16_t startupFrames;
	/* IIR filter speed applied to algorithm results */
	double speed;
	double sigmaCr;
	double sigmaCb;
	double minCount;
	uint16_t minG;
	double omega;
	uint32_t nIter;
	double luminanceLut[ALSC_CELLS_X * ALSC_CELLS_Y];
	double luminanceStrength;
	std::vector<AlscCalibration> calibrationsCr;
	std::vector<AlscCalibration> calibrationsCb;
	double defaultCt; /* colour temperature if no metadata found */
	double threshold; /* iteration termination threshold */
	double lambdaBound; /* upper/lower bound for lambda from a value of 1 */
};

class Alsc : public Algorithm
{
public:
	Alsc(Controller *controller = NULL);
	~Alsc();
	char const *name() const override;
	void initialise() override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void read(boost::property_tree::ptree const &params) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

private:
	/* configuration is read-only, and available to both threads */
	AlscConfig config_;
	bool firstTime_;
	CameraMode cameraMode_;
	double luminanceTable_[ALSC_CELLS_X * ALSC_CELLS_Y];
	std::thread asyncThread_;
	void asyncFunc(); /* asynchronous thread function */
	std::mutex mutex_;
	/* condvar for async thread to wait on */
	std::condition_variable asyncSignal_;
	/* condvar for synchronous thread to wait on */
	std::condition_variable syncSignal_;
	/* for sync thread to check  if async thread finished (requires mutex) */
	bool asyncFinished_;
	/* for async thread to check if it's been told to run (requires mutex) */
	bool asyncStart_;
	/* for async thread to check if it's been told to quit (requires mutex) */
	bool asyncAbort_;

	/*
	 * The following are only for the synchronous thread to use:
	 * for sync thread to note its has asked async thread to run
	 */
	bool asyncStarted_;
	/* counts up to framePeriod before restarting the async thread */
	int framePhase_;
	/* counts up to startupFrames */
	int frameCount_;
	/* counts up to startupFrames for Process function */
	int frameCount2_;
	double syncResults_[3][ALSC_CELLS_Y][ALSC_CELLS_X];
	double prevSyncResults_[3][ALSC_CELLS_Y][ALSC_CELLS_X];
	void waitForAysncThread();
	/*
	 * The following are for the asynchronous thread to use, though the main
	 * thread can set/reset them if the async thread is known to be idle:
	 */
	void restartAsync(StatisticsPtr &stats, Metadata *imageMetadata);
	/* copy out the results from the async thread so that it can be restarted */
	void fetchAsyncResults();
	double ct_;
	bcm2835_isp_stats_region statistics_[ALSC_CELLS_Y * ALSC_CELLS_X];
	double asyncResults_[3][ALSC_CELLS_Y][ALSC_CELLS_X];
	double asyncLambdaR_[ALSC_CELLS_X * ALSC_CELLS_Y];
	double asyncLambdaB_[ALSC_CELLS_X * ALSC_CELLS_Y];
	void doAlsc();
	double lambdaR_[ALSC_CELLS_X * ALSC_CELLS_Y];
	double lambdaB_[ALSC_CELLS_X * ALSC_CELLS_Y];
};

} /* namespace RPiController */
