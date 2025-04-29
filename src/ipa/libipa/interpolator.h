/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class for interpolating maps of objects
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <tuple>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Interpolator)

namespace ipa {

template<typename T>
class Interpolator
{
public:
	Interpolator() = default;
	Interpolator(const std::map<unsigned int, T> &data)
		: data_(data)
	{
	}
	Interpolator(std::map<unsigned int, T> &&data)
		: data_(std::move(data))
	{
	}

	~Interpolator() = default;

	int readYaml(const libcamera::YamlObject &yaml,
		     const std::string &key_name,
		     const std::string &value_name)
	{
		data_.clear();
		lastInterpolatedKey_.reset();

		if (!yaml.isList()) {
			LOG(Interpolator, Error) << "yaml object must be a list";
			return -EINVAL;
		}

		for (const auto &value : yaml.asList()) {
			unsigned int ct = std::stoul(value[key_name].get<std::string>(""));
			std::optional<T> data =
				value[value_name].get<T>();
			if (!data) {
				return -EINVAL;
			}

			data_[ct] = *data;
		}

		if (data_.size() < 1) {
			LOG(Interpolator, Error) << "Need at least one element";
			return -EINVAL;
		}

		return 0;
	}

	void setQuantization(const unsigned int q)
	{
		quantization_ = q;
	}

	void setData(std::map<unsigned int, T> &&data)
	{
		data_ = std::move(data);
		lastInterpolatedKey_.reset();
	}

	const std::map<unsigned int, T> &data() const
	{
		return data_;
	}

	const T &getInterpolated(unsigned int key, unsigned int *quantizedKey = nullptr)
	{
		ASSERT(data_.size() > 0);

		if (quantization_ > 0)
			key = std::lround(key / static_cast<double>(quantization_)) * quantization_;

		if (quantizedKey)
			*quantizedKey = key;

		if (lastInterpolatedKey_.has_value() &&
		    *lastInterpolatedKey_ == key)
			return lastInterpolatedValue_;

		auto it = data_.lower_bound(key);

		if (it == data_.begin())
			return it->second;

		if (it == data_.end())
			return std::prev(it)->second;

		if (it->first == key)
			return it->second;

		auto it2 = std::prev(it);
		double lambda = (key - it2->first) / static_cast<double>(it->first - it2->first);
		interpolate(it2->second, it->second, lastInterpolatedValue_, lambda);
		lastInterpolatedKey_ = key;

		return lastInterpolatedValue_;
	}

	void interpolate(const T &a, const T &b, T &dest, double lambda)
	{
		dest = a * (1.0 - lambda) + b * lambda;
	}

private:
	std::map<unsigned int, T> data_;
	T lastInterpolatedValue_;
	std::optional<unsigned int> lastInterpolatedKey_;
	unsigned int quantization_ = 0;
};

} /* namespace ipa */

} /* namespace libcamera */
