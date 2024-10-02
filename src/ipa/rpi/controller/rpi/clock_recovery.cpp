/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * Camera sync control algorithm
 */
#include "clock_recovery.h"

#include <libcamera/base/log.h>

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiClockRec)

ClockRecovery::ClockRecovery()
{
	initialise();
}

void ClockRecovery::initialise(unsigned int numPts, unsigned int maxJitter, unsigned int minPts)
{
	numPts_ = numPts;
	maxJitter_ = maxJitter;
	minPts_ = minPts;
	reset();
}

void ClockRecovery::reset()
{
	xAve_ = 0;
	yAve_ = 0;
	x2Ave_ = 0;
	xyAve_ = 0;
	count_ = 0;
	slope_ = 0.0;
	offset_ = 0.0;
}

void ClockRecovery::addSample(uint64_t input, uint64_t output)
{
	if (count_ == 0) {
		inputBase_ = input;
		outputBase_ = output;
	}

	/*
	 * Never let the new output value be more than maxJitter_ away from what we would have expected.
	 * This is just to filter out any rare but really crazy values.
	 */
	uint64_t expectedOutput = getOutput(input);
	output = std::clamp(output, expectedOutput - maxJitter_, expectedOutput + maxJitter_);
	double x = input - inputBase_;
	double y = output - outputBase_ - x;

	/*
	 * We use x, y, x^2 and x*y sums to calculate the best fit line. Here we update them by
	 * pretending we have count_ samples at the previous fit, and now one new one. Gradually
	 * the effect of the older values gets lost. This is a very simple way of updating the
	 * fit (there are much more complicated ones!), but it works well enough. Using averages
	 * instead of sums makes the relative effect of old values and the new sample clearer.
	 */
	unsigned int count1 = count_ + 1;
	xAve_ = (count_ * xAve_ + x) / count1;
	yAve_ = (count_ * yAve_ + y) / count1;
	x2Ave_ = (count_ * x2Ave_ + x * x) / count1;
	xyAve_ = (count_ * xyAve_ + x * y) / count1;

	/* Don't update slope and offset until we've seen "enough" sample points. */
	if (count_ > minPts_) {
		/* These are the standard equations for least squares linear regressions. */
		slope_ = (count1 * count1 * xyAve_ - count1 * xAve_ * count1 * yAve_) /
			 (count1 * count1 * x2Ave_ - count1 * xAve_ * count1 * xAve_);
		offset_ = yAve_ - slope_ * xAve_;
	}

	/* Don't increase count_ above numPts_, as this controls the long-term amount of the residual fit. */
	if (count1 < numPts_)
		count_++;
}

uint64_t ClockRecovery::getOutput(uint64_t input)
{
	double x = input - inputBase_;
	double y = slope_ * x + offset_;
	return y + x + outputBase_;
}
