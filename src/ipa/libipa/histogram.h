/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * histogram calculation interface
 */

#pragma once

#include <limits.h>
#include <stdint.h>
#include <type_traits>
#include <vector>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

namespace libcamera {

namespace ipa {

class Histogram
{
public:
	Histogram() { cumulative_.push_back(0); }
	Histogram(Span<const uint32_t> data);

	template<typename Transform,
		 std::enable_if_t<std::is_invocable_v<Transform, uint32_t>> * = nullptr>
	Histogram(Span<const uint32_t> data, Transform transform)
	{
		cumulative_.resize(data.size() + 1);
		cumulative_[0] = 0;
		for (const auto &[i, value] : utils::enumerate(data))
			cumulative_[i + 1] = cumulative_[i] + transform(value);
	}

	size_t bins() const { return cumulative_.size() - 1; }
	const Span<const uint64_t> data() const { return cumulative_; }
	uint64_t total() const { return cumulative_[cumulative_.size() - 1]; }
	uint64_t cumulativeFrequency(double bin) const;
	double quantile(double q, uint32_t first = 0, uint32_t last = UINT_MAX) const;
	double interQuantileMean(double lowQuantile, double hiQuantile) const;

private:
	std::vector<uint64_t> cumulative_;
};

} /* namespace ipa */

} /* namespace libcamera */
