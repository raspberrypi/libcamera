/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * histogram.h - histogram calculation interface
 */
#ifndef __LIBCAMERA_IPA_LIBIPA_HISTOGRAM_H__
#define __LIBCAMERA_IPA_LIBIPA_HISTOGRAM_H__

#include <assert.h>
#include <limits.h>
#include <stdint.h>

#include <vector>

#include <libcamera/span.h>

namespace libcamera {

namespace ipa {

class Histogram
{
public:
	Histogram(Span<uint32_t> data);
	size_t bins() const { return cumulative_.size() - 1; }
	uint64_t total() const { return cumulative_[cumulative_.size() - 1]; }
	uint64_t cumulativeFrequency(double bin) const;
	double quantile(double q, uint32_t first = 0, uint32_t last = UINT_MAX) const;
	double interQuantileMean(double lowQuantile, double hiQuantile) const;

private:
	std::vector<uint64_t> cumulative_;
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_HISTOGRAM_H__ */
