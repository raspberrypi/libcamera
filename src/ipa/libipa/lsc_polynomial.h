/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Polynomial based lens shading correction
 */
#pragma once

#include <array>
#include <map>
#include <span>
#include <vector>

#include <libcamera/geometry.h>

#include "libcamera/internal/value_node.h"

#include "lsc_base.h"

namespace libcamera {

namespace ipa {

namespace lsc {

class Polynomial
{
public:
	Polynomial(double cx = 0.0, double cy = 0.0, double k0 = 0.0,
		   double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
		   double k4 = 0.0)
		: cx_(cx), cy_(cy), cnx_(0), cny_(0),
		  coefficients_({ k0, k1, k2, k3, k4 })
	{
	}

	double sampleAtNormalizedPixelPos(double x, double y) const;
	double getM() const;
	void setReferenceImageSize(const Size &size);

private:
	double cx_;
	double cy_;
	double cnx_;
	double cny_;
	std::array<double, 5> coefficients_;
	Size imageSize_;
};

} /* namespace lsc */

class LscPolynomial : public LscImplementation
{
private:
	using PolynomialComponents = std::map<std::string, lsc::Polynomial, std::less<>>;
	using PolynomialComponentsMap = std::map<unsigned int, PolynomialComponents>;

public:
	int parseLscData(const ValueNode &sets,
			 const LscDescriptor &descriptor) override;

	LscImplementation::ComponentsMap
	sampleForCrop(const Rectangle &cropRectangle,
		      std::vector<double> xPos, std::vector<double> yPos) override;

private:
	std::vector<float> samplePolynomial(const lsc::Polynomial &poly,
					    std::span<const double> xPositions,
					    std::span<const double> yPositions,
					    const Rectangle &cropRectangle);
	PolynomialComponentsMap lscData_;
};

} /* namespace ipa */

} /* namespace libcamera */
