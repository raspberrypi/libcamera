/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * Camera recovery algorithm
 */
#pragma once

#include <stdint.h>

namespace libcamera {

class ClockRecovery
{
public:
	ClockRecovery();

	void configure(unsigned int numSamples = 100, unsigned int maxJitter = 2000,
		       unsigned int minSamples = 10, unsigned int errorThreshold = 50000);
	void reset();

	void addSample();
	void addSample(uint64_t input, uint64_t output);

	uint64_t getOutput(uint64_t input);

private:
	/* Approximate number of samples over which the model state persists. */
	unsigned int numSamples_;
	/* Remove any output jitter larger than this immediately. */
	unsigned int maxJitter_;
	/* Number of samples required before we start to use model estimates. */
	unsigned int minSamples_;
	/* Threshold above which we assume the wallclock has been reset. */
	unsigned int errorThreshold_;

	/* How many samples seen (up to numSamples_). */
	unsigned int count_;
	/* This gets subtracted from all input values, just to make the numbers easier. */
	uint64_t inputBase_;
	/* As above, for the output. */
	uint64_t outputBase_;
	/* The previous input sample. */
	uint64_t lastInput_;
	/* The previous output sample. */
	uint64_t lastOutput_;

	/* Average x value seen so far. */
	double xAve_;
	/* Average y value seen so far */
	double yAve_;
	/* Average x^2 value seen so far. */
	double x2Ave_;
	/* Average x*y value seen so far. */
	double xyAve_;

	/*
	 * The latest estimate of linear parameters to derive the output clock
	 * from the input.
	 */
	double slope_;
	double offset_;

	/* Use this cumulative error to monitor for spontaneous clock updates. */
	double error_;
};

} /* namespace libcamera */
