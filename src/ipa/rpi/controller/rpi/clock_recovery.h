/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * Camera sync control algorithm
 */
#pragma once

#include <stdint.h>

namespace RPiController {

class ClockRecovery
{
public:
	ClockRecovery();

	/* Initialise with configuration parameters and restart the fitting process. */
	void initialise(unsigned int numPts = 100, unsigned int maxJitter = 100000, unsigned int minPts = 10);
	/* Erase all history and restart the fitting process. */
	void reset();

	// Add a new input clock / output clock sample. */
	void addSample(uint64_t input, uint64_t output);
	/* Calculate the output clock value for this input. */
	uint64_t getOutput(uint64_t input);

private:
	unsigned int numPts_; /* how many samples contribute to the history */
	unsigned int maxJitter_; /* smooth out any jitter larger than this immediately */
	unsigned int minPts_; /* number of samples below which we treat clocks as 1:1 */
	unsigned int count_; /* how many samples seen (up to numPts_) */
	uint64_t inputBase_; /* subtract this from all input values, just to make the numbers easier */
	uint64_t outputBase_; /* as above, for the output */

	/*
	 * We do a linear regression of y against x, where:
	 * x is the value input - inputBase_, and
	 * y is the value output - outputBase_ - x.
	 * We additionally subtract x from y so that y "should" be zero, again making the numnbers easier.
	 */
	double xAve_; /* average x value seen so far */
	double yAve_; /* average y value seen so far */
	double x2Ave_; /* average x^2 value seen so far */
	double xyAve_; /* average x*y value seen so far */

	/*
	 * Once we've seen more than minPts_ samples, we recalculate the slope and offset according
	 * to the linear regression normal equations.
	 */
	double slope_; /* latest slope value */
	double offset_; /* latest offset value */
};

} //namespace RPiController
