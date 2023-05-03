/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * histogram.cpp - histogram calculations
 */
#include <math.h>
#include <stdio.h>

#include "histogram.h"

using namespace RPiController;

uint64_t Histogram::cumulativeFreq(double bin) const
{
	if (bin <= 0)
		return 0;
	else if (bin >= bins())
		return total();
	int b = (int)bin;
	return cumulative_[b] +
	       (bin - b) * (cumulative_[b + 1] - cumulative_[b]);
}

double Histogram::quantile(double q, int first, int last) const
{
	if (first == -1)
		first = 0;
	if (last == -1)
		last = cumulative_.size() - 2;
	assert(first <= last);
	uint64_t items = q * total();
	while (first < last) /* binary search to find the right bin */
	{
		int middle = (first + last) / 2;
		if (cumulative_[middle + 1] > items)
			last = middle; /* between first and middle */
		else
			first = middle + 1; /* after middle */
	}
	assert(items >= cumulative_[first] && items <= cumulative_[last + 1]);
	double frac = cumulative_[first + 1] == cumulative_[first] ? 0
		      : (double)(items - cumulative_[first]) /
				  (cumulative_[first + 1] - cumulative_[first]);
	return first + frac;
}

double Histogram::interQuantileMean(double qLo, double qHi) const
{
	assert(qHi > qLo);
	double pLo = quantile(qLo);
	double pHi = quantile(qHi, (int)pLo);
	double sumBinFreq = 0, cumulFreq = 0;
	for (double pNext = floor(pLo) + 1.0; pNext <= ceil(pHi);
	     pLo = pNext, pNext += 1.0) {
		int bin = floor(pLo);
		double freq = (cumulative_[bin + 1] - cumulative_[bin]) *
			      (std::min(pNext, pHi) - pLo);
		sumBinFreq += bin * freq;
		cumulFreq += freq;
	}
	/* add 0.5 to give an average for bin mid-points */
	return sumBinFreq / cumulFreq + 0.5;
}
