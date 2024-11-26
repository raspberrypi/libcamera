/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * Clock recovery algorithm
 */

#include "libcamera/internal/clock_recovery.h"

#include <time.h>

#include <libcamera/base/log.h>

/**
 * \file clock_recovery.h
 * \brief Clock recovery - deriving one clock from another independent clock
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(ClockRec)

/**
 * \class ClockRecovery
 * \brief Recover an output clock from an input clock
 *
 * The ClockRecovery class derives an output clock from an input clock,
 * modelling the output clock as being linearly related to the input clock.
 * For example, we may use it to derive wall clock timestamps from timestamps
 * measured by the internal system clock which counts local time since boot.
 *
 * When pairs of corresponding input and output timestamps are available,
 * they should be submitted to the model with addSample(). The model will
 * update, and output clock values for known input clock values can be
 * obtained using getOutput().
 *
 * As a convenience, if the input clock is indeed the time since boot, and the
 * output clock represents a real wallclock time, then addSample() can be
 * called with no arguments, and a pair of timestamps will be captured at
 * that moment.
 *
 * The configure() function accepts some configuration parameters to control
 * the linear fitting process.
 */

/**
 * \brief Construct a ClockRecovery
 */
ClockRecovery::ClockRecovery()
{
	configure();
	reset();
}

/**
 * \brief Set configuration parameters
 * \param[in] numSamples The approximate duration for which the state of the model
 * is persistent
 * \param[in] maxJitter New output samples are clamped to no more than this
 * amount of jitter, to prevent sudden swings from having a large effect
 * \param[in] minSamples The fitted clock model is not used to generate outputs
 * until this many samples have been received
 * \param[in] errorThreshold If the accumulated differences between input and
 * output clocks reaches this amount over a few frames, the model is reset
 */
void ClockRecovery::configure(unsigned int numSamples, unsigned int maxJitter,
			      unsigned int minSamples, unsigned int errorThreshold)
{
	LOG(ClockRec, Debug)
		<< "configure " << numSamples << " " << maxJitter << " " << minSamples << " " << errorThreshold;

	numSamples_ = numSamples;
	maxJitter_ = maxJitter;
	minSamples_ = minSamples;
	errorThreshold_ = errorThreshold;
}

/**
 * \brief Reset the clock recovery model and start again from scratch
 */
void ClockRecovery::reset()
{
	LOG(ClockRec, Debug) << "reset";

	lastInput_ = 0;
	lastOutput_ = 0;
	xAve_ = 0;
	yAve_ = 0;
	x2Ave_ = 0;
	xyAve_ = 0;
	count_ = 0;
	error_ = 0.0;
	/*
	 * Setting slope_ and offset_ to zero initially means that the clocks
	 * advance at exactly the same rate.
	 */
	slope_ = 0.0;
	offset_ = 0.0;
}

/**
 * \brief Add a sample point to the clock recovery model, for recovering a wall
 * clock value from the internal system time since boot
 *
 * This is a convenience function to make it easy to derive a wall clock value
 * (using the Linux CLOCK_REALTIME) from the time since the system started
 * (measured by CLOCK_BOOTTIME).
 */
void ClockRecovery::addSample()
{
	LOG(ClockRec, Debug) << "addSample";

	struct timespec bootTime1;
	struct timespec bootTime2;
	struct timespec wallTime;

	/* Get boot and wall clocks in microseconds. */
	clock_gettime(CLOCK_BOOTTIME, &bootTime1);
	clock_gettime(CLOCK_REALTIME, &wallTime);
	clock_gettime(CLOCK_BOOTTIME, &bootTime2);
	uint64_t boot1 = bootTime1.tv_sec * 1000000ULL + bootTime1.tv_nsec / 1000;
	uint64_t boot2 = bootTime2.tv_sec * 1000000ULL + bootTime2.tv_nsec / 1000;
	uint64_t boot = (boot1 + boot2) / 2;
	uint64_t wall = wallTime.tv_sec * 1000000ULL + wallTime.tv_nsec / 1000;

	addSample(boot, wall);
}

/**
 * \brief Add a sample point to the clock recovery model, specifying the exact
 * input and output clock values
 * \param[in] input The input clock value
 * \param[in] output The value of the output clock at the same moment, as far
 * as possible, that the input clock was sampled
 *
 * This function should be used for corresponding clocks other than the Linux
 * BOOTTIME and REALTIME clocks.
 */
void ClockRecovery::addSample(uint64_t input, uint64_t output)
{
	LOG(ClockRec, Debug) << "addSample " << input << " " << output;

	if (count_ == 0) {
		inputBase_ = input;
		outputBase_ = output;
	}

	/*
	 * We keep an eye on cumulative drift over the last several frames. If this exceeds a
	 * threshold, then probably the system clock has been updated and we're going to have to
	 * reset everything and start over.
	 */
	if (lastOutput_) {
		int64_t inputDiff = getOutput(input) - getOutput(lastInput_);
		int64_t outputDiff = output - lastOutput_;
		error_ = error_ * 0.95 + (outputDiff - inputDiff);
		if (std::abs(error_) > errorThreshold_) {
			reset();
			inputBase_ = input;
			outputBase_ = output;
		}
	}
	lastInput_ = input;
	lastOutput_ = output;

	/*
	 * Never let the new output value be more than maxJitter_ away from what
	 * we would have expected.  This is just to reduce the effect of sudden
	 * large delays in the measured output.
	 */
	uint64_t expectedOutput = getOutput(input);
	output = std::clamp(output, expectedOutput - maxJitter_, expectedOutput + maxJitter_);

	/*
	 * We use x, y, x^2 and x*y sums to calculate the best fit line. Here we
	 * update them by pretending we have count_ samples at the previous fit,
	 * and now one new one. Gradually the effect of the older values gets
	 * lost. This is a very simple way of updating the fit (there are much
	 * more complicated ones!), but it works well enough. Using averages
	 * instead of sums makes the relative effect of old values and the new
	 * sample clearer.
	 */
	double x = static_cast<int64_t>(input - inputBase_);
	double y = static_cast<int64_t>(output - outputBase_) - x;
	unsigned int count1 = count_ + 1;
	xAve_ = (count_ * xAve_ + x) / count1;
	yAve_ = (count_ * yAve_ + y) / count1;
	x2Ave_ = (count_ * x2Ave_ + x * x) / count1;
	xyAve_ = (count_ * xyAve_ + x * y) / count1;

	/*
	 * Don't update slope and offset until we've seen "enough" sample
	 * points.  Note that the initial settings for slope_ and offset_
	 * ensures that the wallclock advances at the same rate as the realtime
	 * clock (but with their respective initial offsets).
	 */
	if (count_ > minSamples_) {
		/* These are the standard equations for least squares linear regression. */
		slope_ = (count1 * count1 * xyAve_ - count1 * xAve_ * count1 * yAve_) /
			 (count1 * count1 * x2Ave_ - count1 * xAve_ * count1 * xAve_);
		offset_ = yAve_ - slope_ * xAve_;
	}

	/*
	 * Don't increase count_ above numSamples_, as this controls the long-term
	 * amount of the residual fit.
	 */
	if (count1 < numSamples_)
		count_++;
}

/**
 * \brief Calculate the output clock value according to the model from an input
 * clock value
 * \param[in] input The input clock value
 *
 * \return Output clock value
 */
uint64_t ClockRecovery::getOutput(uint64_t input)
{
	double x = static_cast<int64_t>(input - inputBase_);
	double y = slope_ * x + offset_;
	uint64_t output = y + x + outputBase_;

	LOG(ClockRec, Debug) << "getOutput " << input << " " << output;

	return output;
}

} /* namespace libcamera */
