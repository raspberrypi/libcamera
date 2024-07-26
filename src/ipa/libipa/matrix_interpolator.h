/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class for interpolating maps of matrices
 */

#pragma once

#include <algorithm>
#include <map>
#include <string>
#include <tuple>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "matrix.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MatrixInterpolator)

namespace ipa {

#ifndef __DOXYGEN__
template<typename T, unsigned int R, unsigned int C,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
#else
template<typename T, unsigned int R, unsigned int C>
#endif /* __DOXYGEN__ */
class MatrixInterpolator
{
public:
	MatrixInterpolator()
	{
		reset();
	}

	MatrixInterpolator(const std::map<unsigned int, Matrix<T, R, C>> &matrices)
	{
		for (const auto &pair : matrices)
			matrices_[pair.first] = pair.second;
	}

	~MatrixInterpolator() {}

	void reset()
	{
		matrices_.clear();
		matrices_[0] = Matrix<T, R, C>::identity();
	}

	int readYaml(const libcamera::YamlObject &yaml,
		     const std::string &key_name,
		     const std::string &matrix_name)
	{
		matrices_.clear();

		if (!yaml.isList()) {
			LOG(MatrixInterpolator, Error) << "yaml object must be a list";
			return -EINVAL;
		}

		for (const auto &value : yaml.asList()) {
			unsigned int ct = std::stoul(value[key_name].get<std::string>(""));
			std::optional<Matrix<T, R, C>> matrix =
				value[matrix_name].get<Matrix<T, R, C>>();
			if (!matrix) {
				LOG(MatrixInterpolator, Error) << "Failed to read matrix";
				return -EINVAL;
			}

			matrices_[ct] = *matrix;

			LOG(MatrixInterpolator, Debug)
				<< "Read matrix '" << matrix_name << "' for key '"
				<< key_name << "' " << ct << ": "
				<< matrices_[ct].toString();
		}

		if (matrices_.size() < 1) {
			LOG(MatrixInterpolator, Error) << "Need at least one matrix";
			return -EINVAL;
		}

		return 0;
	}

	Matrix<T, R, C> get(unsigned int ct)
	{
		ASSERT(matrices_.size() > 0);

		if (matrices_.size() == 1 ||
		    ct <= matrices_.begin()->first)
			return matrices_.begin()->second;

		if (ct >= matrices_.rbegin()->first)
			return matrices_.rbegin()->second;

		if (matrices_.find(ct) != matrices_.end())
			return matrices_[ct];

		/* The above four guarantee that this will succeed */
		auto iter = matrices_.upper_bound(ct);
		unsigned int ctUpper = iter->first;
		unsigned int ctLower = (--iter)->first;

		double lambda = (ct - ctLower) / static_cast<double>(ctUpper - ctLower);
		Matrix<T, R, C> ret =
			lambda * matrices_[ctUpper] + (1.0 - lambda) * matrices_[ctLower];
		return ret;
	}

private:
	std::map<unsigned int, Matrix<T, R, C>> matrices_;
};

} /* namespace ipa */

} /* namespace libcamera */
