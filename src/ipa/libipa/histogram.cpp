/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * histogram.cpp - histogram calculations
 */
#include "histogram.h"

#include <cmath>

#include <libcamera/base/log.h>

/**
 * \file histogram.h
 * \brief Class to represent Histograms and manipulate them
 */

namespace libcamera {

namespace ipa {

/**
 * \class Histogram
 * \brief The base class for creating histograms
 *
 * This class stores a cumulative frequency histogram, which is a mapping that
 * counts the cumulative number of observations in all of the bins up to the
 * specified bin. It can be used to find quantiles and averages between quantiles.
 */

/**
 * \brief Create a cumulative histogram
 * \param[in] data A pre-sorted histogram to be passed
 */
Histogram::Histogram(Span<const uint32_t> data)
{
	cumulative_.reserve(data.size());
	cumulative_.push_back(0);
	for (const uint32_t &value : data)
		cumulative_.push_back(cumulative_.back() + value);
}

/**
 * \fn Histogram::bins()
 * \brief Retrieve the number of bins currently used by the Histogram
 * \return Number of bins
 */

/**
 * \fn Histogram::total()
 * \brief Retrieve the total number of values in the data set
 * \return Number of values
 */

/**
 * \brief Cumulative frequency up to a (fractional) point in a bin
 * \param[in] bin The bin up to which to cumulate
 *
 * With F(p) the cumulative frequency of the histogram, the value is 0 at
 * the bottom of the histogram, and the maximum is the number of bins.
 * The pixels are spread evenly throughout the “bin” in which they lie, so that
 * F(p) is a continuous (monotonically increasing) function.
 *
 * \return The cumulative frequency from 0 up to the specified bin
 */
uint64_t Histogram::cumulativeFrequency(double bin) const
{
	if (bin <= 0)
		return 0;
	else if (bin >= bins())
		return total();
	int b = static_cast<int32_t>(bin);
	return cumulative_[b] +
	       (bin - b) * (cumulative_[b + 1] - cumulative_[b]);
}

/**
 * \brief Return the (fractional) bin of the point through the histogram
 * \param[in] q the desired point (0 <= q <= 1)
 * \param[in] first low limit (default is 0)
 * \param[in] last high limit (default is UINT_MAX)
 *
 * A quantile gives us the point p = Q(q) in the range such that a proportion
 * q of the pixels lie below p. A familiar quantile is Q(0.5) which is the median
 * of a distribution.
 *
 * \return The fractional bin of the point
 */
double Histogram::quantile(double q, uint32_t first, uint32_t last) const
{
	if (last == UINT_MAX)
		last = cumulative_.size() - 2;
	ASSERT(first <= last);

	uint64_t item = q * total();
	/* Binary search to find the right bin */
	while (first < last) {
		int middle = (first + last) / 2;
		/* Is it between first and middle ? */
		if (cumulative_[middle + 1] > item)
			last = middle;
		else
			first = middle + 1;
	}
	ASSERT(item >= cumulative_[first] && item <= cumulative_[last + 1]);

	double frac;
	if (cumulative_[first + 1] == cumulative_[first])
		frac = 0;
	else
		frac = (item - cumulative_[first]) / (cumulative_[first + 1] - cumulative_[first]);
	return first + frac;
}

/**
 * \brief Calculate the mean between two quantiles
 * \param[in] lowQuantile low Quantile
 * \param[in] highQuantile high Quantile
 *
 * Quantiles are not ideal for metering as they suffer several limitations.
 * Instead, a concept is introduced here: inter-quantile mean.
 * It returns the mean of all pixels between lowQuantile and highQuantile.
 *
 * \return The mean histogram bin value between the two quantiles
 */
double Histogram::interQuantileMean(double lowQuantile, double highQuantile) const
{
	ASSERT(highQuantile > lowQuantile);
	/* Proportion of pixels which lies below lowQuantile */
	double lowPoint = quantile(lowQuantile);
	/* Proportion of pixels which lies below highQuantile */
	double highPoint = quantile(highQuantile, static_cast<uint32_t>(lowPoint));
	double sumBinFreq = 0, cumulFreq = 0;

	for (double p_next = floor(lowPoint) + 1.0;
	     p_next <= ceil(highPoint);
	     lowPoint = p_next, p_next += 1.0) {
		int bin = floor(lowPoint);
		double freq = (cumulative_[bin + 1] - cumulative_[bin])
			* (std::min(p_next, highPoint) - lowPoint);

		/* Accumulate weighted bin */
		sumBinFreq += bin * freq;
		/* Accumulate weights */
		cumulFreq += freq;
	}
	/* add 0.5 to give an average for bin mid-points */
	return sumBinFreq / cumulFreq + 0.5;
}

} /* namespace ipa */

} /* namespace libcamera */
