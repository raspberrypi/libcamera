/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Polynomial based lens shading correction
 */

#include "lsc_polynomial.h"

#include <assert.h>
#include <cmath>

#include <libcamera/base/log.h>

/**
 * \file lsc_polynomial.h
 * \brief LscPolynomial class
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(LscPolynomial)

namespace ipa {

namespace lsc {

/**
 * \class Polynomial
 * \brief Class for handling even polynomials used in lens shading correction
 *
 * Shading artifacts of camera lenses can be modeled using even radial
 * polynomials. This class implements a polynomial with 5 coefficients which
 * follows the definition of the FixVignetteRadial opcode in the Adobe DNG
 * specification.
 */

/**
 * \fn Polynomial::Polynomial(double cx = 0.0, double cy = 0.0, double k0 = 0.0,
		      double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
		      double k4 = 0.0)
 * \brief Construct a polynomial using the given coefficients
 * \param cx Center-x relative to the image in normalized coordinates (0..1)
 * \param cy Center-y relative to the image in normalized coordinates (0..1)
 * \param k0 Coefficient of the polynomial
 * \param k1 Coefficient of the polynomial
 * \param k2 Coefficient of the polynomial
 * \param k3 Coefficient of the polynomial
 * \param k4 Coefficient of the polynomial
 */

/**
 * \brief Sample the polynomial at the given normalized pixel position
 *
 * This functions samples the polynomial at the given pixel position divided by
 * the value returned by getM().
 *
 * \param x x position in normalized coordinates
 * \param y y position in normalized coordinates
 * \return The sampled value
 */
double Polynomial::sampleAtNormalizedPixelPos(double x, double y) const
{
	double dx = x - cnx_;
	double dy = y - cny_;
	double r = sqrt(dx * dx + dy * dy);
	double res = 1.0;

	for (unsigned int i = 0; i < coefficients_.size(); i++)
		res += coefficients_[i] * std::pow(r, (i + 1) * 2);

	return res;
}

/**
 * \brief Get the value m as described in the dng specification
 *
 * Returns m according to dng spec. m represents the Euclidean distance
 * (in pixels) from the optical center to the farthest pixel in the
 * image.
 *
 * \return The sampled value
 */
double Polynomial::getM() const
{
	double cpx = imageSize_.width * cx_;
	double cpy = imageSize_.height * cy_;
	double mx = std::max(cpx, std::fabs(imageSize_.width - cpx));
	double my = std::max(cpy, std::fabs(imageSize_.height - cpy));

	return sqrt(mx * mx + my * my);
}

/**
 * \brief Set the reference image size
 *
 * Set the reference image size that is used for subsequent calls to getM() and
 * sampleAtNormalizedPixelPos()
 *
 * \param size The size of the reference image
 */
void Polynomial::setReferenceImageSize(const Size &size)
{
	assert(!size.isNull());
	imageSize_ = size;

	/* Calculate normalized centers */
	double m = getM();
	cnx_ = (size.width * cx_) / m;
	cny_ = (size.height * cy_) / m;
}

} /* namespace lsc */

/**
 * \class LscPolynomial
 * \brief Radial Polynomial LSC algorithm implementation
 *
 * Polynomial-based LSC algorithm implementation. The LscPolynomial class
 * implements LSC support using a Polynomial to represent the shading artifacts
 * map.
 *
 * \sa LscImplementation
 */

/**
 * \brief Parse polynomial LSC data
 * \param[in] sets The tuning file content
 * \param[in] descriptor The LSC engine descriptor
 *
 * Parse the LSC data in polyomial form from the \a sets tuning data.
 *
 * \return 0 on success or a negative error number otherwise
 */
int LscPolynomial::parseLscData(const ValueNode &sets,
				const LscDescriptor &descriptor)
{
	for (const auto &set : sets.asList()) {
		uint32_t ct = set["ct"].get<uint32_t>(0);

		PolynomialComponents components;
		for (auto &k : descriptor.keys) {
			auto polynomial = set[k].get<lsc::Polynomial>();
			if (!polynomial) {
				LOG(LscPolynomial, Error)
					<< "Missing polynomial for component "
					<< k;
				return -EINVAL;
			}

			auto [it, inserted] =
				components.try_emplace(k, std::move(*polynomial));
			ASSERT(inserted);

			it->second.setReferenceImageSize(descriptor.sensorSize);
		}

		auto [it, inserted] = lscData_.try_emplace(ct, std::move(components));
		if (!inserted) {
			LOG(LscPolynomial, Error)
				<< "Multiple sets found for "
				<< "color temperature " << ct;
			return -EINVAL;
		}
	}

	if (lscData_.empty()) {
		LOG(LscPolynomial, Error) << "Failed to load any sets";
		return -EINVAL;
	}

	return 0;
}

/**
 * \brief Re-sample the LSC components for \a cropRectangle
 * \param[in] cropRectangle The sensor analogue crop rectangle
 * \param[in] xPos List of horizontal positions of the LSC grid nodes
 * \param[in] yPos List of vertical positions of the LSC grid nodes
 *
 * LSC tables have to be re-sampled every time a new sensor configuration is
 * used, as each streaming session might use a different sensor crop rectangle.
 *
 * Polynomial LSC tables can be re-sampled for a given sensor frame resolution
 * using a list of horizontal and vertical nodes that define the LSC grid on
 * which the polynomial is re-sampled on.
 *
 * \a cropRectangle represents the size of the frame on which the LSC tables
 * have to be re-sampled on.
 *
 * \a xPos and \a yPos represent the position of the grid nodes vertexes in
 * the [0, 1] interval. In example an equally spaced grid of 16 nodes will have
 * each segment of size 0.0625 and the list of nodes position will be
 * [0, 0.0625, 0.125, 0.1875, ... , 1]. It is expected that the first position
 * is 0 and the last position is 1.
 */
LscImplementation::ComponentsMap
LscPolynomial::sampleForCrop(const Rectangle &cropRectangle,
			     std::vector<double> xPos, std::vector<double> yPos)
{

	LscImplementation::ComponentsMap components;

	for (const auto &[t, c] : lscData_) {
		LscImplementation::Components &comp = components[t];

		for (const auto &[k, p] : c)
			comp.try_emplace(k, samplePolynomial(p, xPos, yPos,
							     cropRectangle));
	}

	return components;
}

std::vector<float>
LscPolynomial::samplePolynomial(const lsc::Polynomial &poly,
				Span<const double> xPositions,
				Span<const double> yPositions,
				const Rectangle &cropRectangle)
{
	double m = poly.getM();
	double x0 = cropRectangle.x / m;
	double y0 = cropRectangle.y / m;
	double w = cropRectangle.width / m;
	double h = cropRectangle.height / m;
	std::vector<float> samples;

	samples.reserve(xPositions.size() * yPositions.size());

	for (double y : yPositions) {
		for (double x : xPositions) {
			double xp = x0 + x * w;
			double yp = y0 + y * h;

			samples.push_back(static_cast<float>
					 (poly.sampleAtNormalizedPixelPos(xp, yp)));
		}
	}
	return samples;
}

} /* namespace ipa */

#ifndef __DOXYGEN__
template<>
std::optional<ipa::lsc::Polynomial>
ValueNode::Accessor<ipa::lsc::Polynomial>::get(const ValueNode &obj) const
{
	std::optional<double> cx = obj["cx"].get<double>();
	std::optional<double> cy = obj["cy"].get<double>();
	std::optional<double> k0 = obj["k0"].get<double>();
	std::optional<double> k1 = obj["k1"].get<double>();
	std::optional<double> k2 = obj["k2"].get<double>();
	std::optional<double> k3 = obj["k3"].get<double>();
	std::optional<double> k4 = obj["k4"].get<double>();

	if (!(cx && cy && k0 && k1 && k2 && k3 && k4)) {
		LOG(LscPolynomial, Error)
			<< "Polynomial is missing a parameter";
		return std::nullopt;
	}

	return ipa::lsc::Polynomial(*cx, *cy, *k0, *k1, *k2, *k3, *k4);
}
#endif /* __DOXYGEN__ */

} /* namespace libcamera */
