/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * histogram.h - histogram calculation interface
 */

#pragma once

#include <assert.h>
#include <limits.h>
#include <stdint.h>

#include <vector>

#include <libcamera/base/span.h>

namespace libcamera {

namespace ipa {

class Histogram
{
public:
	Histogram(Span<const uint32_t> data);
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
