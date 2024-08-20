/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Helper for radial polynomial used in lens shading correction.
 */
#pragma once

#include <algorithm>
#include <array>
#include <assert.h>
#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(LscPolynomial)

namespace ipa {

class LscPolynomial
{
public:
	LscPolynomial(double cx = 0.0, double cy = 0.0, double k0 = 0.0,
		      double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
		      double k4 = 0.0)
		: cx_(cx), cy_(cy), cnx_(0), cny_(0),
		  coefficients_({ k0, k1, k2, k3, k4 })
	{
	}

	double sampleAtNormalizedPixelPos(double x, double y) const
	{
		double dx = x - cnx_;
		double dy = y - cny_;
		double r = sqrt(dx * dx + dy * dy);
		double res = 1.0;
		for (unsigned int i = 0; i < coefficients_.size(); i++) {
			res += coefficients_[i] * std::pow(r, (i + 1) * 2);
		}
		return res;
	}

	double getM() const
	{
		double cpx = imageSize_.width * cx_;
		double cpy = imageSize_.height * cy_;
		double mx = std::max(cpx, std::fabs(imageSize_.width - cpx));
		double my = std::max(cpy, std::fabs(imageSize_.height - cpy));

		return sqrt(mx * mx + my * my);
	}

	void setReferenceImageSize(const Size &size)
	{
		assert(!size.isNull());
		imageSize_ = size;

		/* Calculate normalized centers */
		double m = getM();
		cnx_ = (size.width * cx_) / m;
		cny_ = (size.height * cy_) / m;
	}

private:
	double cx_;
	double cy_;
	double cnx_;
	double cny_;
	std::array<double, 5> coefficients_;

	Size imageSize_;
};

} /* namespace ipa */

#ifndef __DOXYGEN__

template<>
struct YamlObject::Getter<ipa::LscPolynomial> {
	std::optional<ipa::LscPolynomial> get(const YamlObject &obj) const
	{
		std::optional<double> cx = obj["cx"].get<double>();
		std::optional<double> cy = obj["cy"].get<double>();
		std::optional<double> k0 = obj["k0"].get<double>();
		std::optional<double> k1 = obj["k1"].get<double>();
		std::optional<double> k2 = obj["k2"].get<double>();
		std::optional<double> k3 = obj["k3"].get<double>();
		std::optional<double> k4 = obj["k4"].get<double>();

		if (!(cx && cy && k0 && k1 && k2 && k3 && k4))
			LOG(LscPolynomial, Error)
				<< "Polynomial is missing a parameter";

		return ipa::LscPolynomial(*cx, *cy, *k0, *k1, *k2, *k3, *k4);
	}
};

#endif

} /* namespace libcamera */
