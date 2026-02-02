/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * Simple builtin benchmark to measure software ISP processing times
 */

#include "libcamera/internal/software_isp/benchmark.h"

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(Benchmark)

/**
 * \class Benchmark
 * \brief Simple builtin benchmark
 *
 * Simple builtin benchmark to measure software ISP processing times.
 */

/**
 * \brief Constructs a Benchmark object
 */
Benchmark::Benchmark(const GlobalConfiguration &configuration)
{
	skipBeforeMeasure_ = configuration.option<unsigned int>(
						{ "software_isp", "measure", "skip" })
							.value_or(skipBeforeMeasure_);
	framesToMeasure_ = configuration.option<unsigned int>(
						{ "software_isp", "measure", "number" })
							.value_or(framesToMeasure_);
}

Benchmark::~Benchmark()
{
}

static inline int64_t timeDiff(timespec &after, timespec &before)
{
	return (after.tv_sec - before.tv_sec) * 1000000000LL +
	       (int64_t)after.tv_nsec - (int64_t)before.tv_nsec;
}

/**
 * \brief Start measuring process time for a single frame
 *
 * Call this function before processing frame data to start measuring
 * the process time for a frame.
 */
void Benchmark::startFrame(void)
{
	measure = framesToMeasure_ > 0 &&
		  encounteredFrames_ < skipBeforeMeasure_ + framesToMeasure_ &&
		  ++encounteredFrames_ > skipBeforeMeasure_;

	if (measure) {
		frameStartTime_ = {};
		clock_gettime(CLOCK_MONOTONIC_RAW, &frameStartTime_);
	}
}

/**
 * \brief Finish measuring process time for a single frame
 *
 * Call this function after processing frame data to finish measuring
 * the process time for a frame.
 *
 * This function will log frame processing time information after
 * Benchmark::kLastFrameToMeasure frames have been processed.
 */
void Benchmark::finishFrame(void)
{
	if (measure) {
		timespec frameEndTime = {};
		clock_gettime(CLOCK_MONOTONIC_RAW, &frameEndTime);
		frameProcessTime_ += timeDiff(frameEndTime, frameStartTime_);
		if (encounteredFrames_ == skipBeforeMeasure_ + framesToMeasure_) {
			LOG(Benchmark, Info)
				<< "Processed " << framesToMeasure_
				<< " frames in " << frameProcessTime_ / 1000 << "us, "
				<< frameProcessTime_ / (1000 * framesToMeasure_)
				<< " us/frame";
		}
	}
}

} /* namespace libcamera */
