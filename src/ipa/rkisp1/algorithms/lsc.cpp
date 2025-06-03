/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 Lens Shading Correction control
 */

#include "lsc.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "libipa/lsc_polynomial.h"
#include "linux/rkisp1-config.h"

/**
 * \file lsc.h
 */

namespace libcamera {

namespace ipa {

constexpr int kColourTemperatureChangeThreshhold = 10;

template<typename T>
void interpolateVector(const std::vector<T> &a, const std::vector<T> &b,
		       std::vector<T> &dest, double lambda)
{
	assert(a.size() == b.size());
	dest.resize(a.size());
	for (size_t i = 0; i < a.size(); i++) {
		dest[i] = a[i] * (1.0 - lambda) + b[i] * lambda;
	}
}

template<>
void Interpolator<rkisp1::algorithms::LensShadingCorrection::Components>::
	interpolate(const rkisp1::algorithms::LensShadingCorrection::Components &a,
		    const rkisp1::algorithms::LensShadingCorrection::Components &b,
		    rkisp1::algorithms::LensShadingCorrection::Components &dest,
		    double lambda)
{
	interpolateVector(a.r, b.r, dest.r, lambda);
	interpolateVector(a.gr, b.gr, dest.gr, lambda);
	interpolateVector(a.gb, b.gb, dest.gb, lambda);
	interpolateVector(a.b, b.b, dest.b, lambda);
}

} /* namespace ipa */

namespace ipa::rkisp1::algorithms {

/**
 * \class LensShadingCorrection
 * \brief RkISP1 Lens Shading Correction control
 *
 * Due to the optical characteristics of the lens, the light intensity received
 * by the sensor is not uniform.
 *
 * The Lens Shading Correction algorithm applies multipliers to all pixels
 * to compensate for the lens shading effect. The coefficients are
 * specified in a downscaled table in the YAML tuning file.
 */

LOG_DEFINE_CATEGORY(RkISP1Lsc)

class LscPolynomialLoader
{
public:
	LscPolynomialLoader(const Size &sensorSize,
			    const Rectangle &cropRectangle,
			    const std::vector<double> &xSizes,
			    const std::vector<double> &ySizes)
		: sensorSize_(sensorSize),
		  cropRectangle_(cropRectangle),
		  xSizes_(xSizes),
		  ySizes_(ySizes)
	{
	}

	int parseLscData(const YamlObject &yamlSets,
			 std::map<unsigned int, LensShadingCorrection::Components> &lscData)
	{
		const auto &sets = yamlSets.asList();
		for (const auto &yamlSet : sets) {
			std::optional<LscPolynomial> pr, pgr, pgb, pb;
			uint32_t ct = yamlSet["ct"].get<uint32_t>(0);

			if (lscData.count(ct)) {
				LOG(RkISP1Lsc, Error)
					<< "Multiple sets found for "
					<< "color temperature " << ct;
				return -EINVAL;
			}

			LensShadingCorrection::Components &set = lscData[ct];
			pr = yamlSet["r"].get<LscPolynomial>();
			pgr = yamlSet["gr"].get<LscPolynomial>();
			pgb = yamlSet["gb"].get<LscPolynomial>();
			pb = yamlSet["b"].get<LscPolynomial>();

			if (!(pr || pgr || pgb || pb)) {
				LOG(RkISP1Lsc, Error)
					<< "Failed to parse polynomial for "
					<< "colour temperature " << ct;
				return -EINVAL;
			}

			set.ct = ct;
			pr->setReferenceImageSize(sensorSize_);
			pgr->setReferenceImageSize(sensorSize_);
			pgb->setReferenceImageSize(sensorSize_);
			pb->setReferenceImageSize(sensorSize_);
			set.r = samplePolynomial(*pr);
			set.gr = samplePolynomial(*pgr);
			set.gb = samplePolynomial(*pgb);
			set.b = samplePolynomial(*pb);
		}

		if (lscData.empty()) {
			LOG(RkISP1Lsc, Error) << "Failed to load any sets";
			return -EINVAL;
		}

		return 0;
	}

private:
	/*
	 * The lsc grid has custom spacing defined on half the range (see
	 * parseSizes() for details). For easier handling this function converts
	 * the spaces vector to positions and mirrors them. E.g.:
	 *
	 * input:   | 0.2 | 0.3 |
	 * output: 0.0   0.2   0.5   0.8   1.0
	 */
	std::vector<double> sizesListToPositions(const std::vector<double> &sizes)
	{
		const int half = sizes.size();
		std::vector<double> res(half * 2 + 1);
		double x = 0.0;

		res[half] = 0.5;
		for (int i = 1; i <= half; i++) {
			x += sizes[half - i];
			res[half - i] = 0.5 - x;
			res[half + i] = 0.5 + x;
		}

		return res;
	}

	std::vector<uint16_t> samplePolynomial(const LscPolynomial &poly)
	{
		constexpr int k = RKISP1_CIF_ISP_LSC_SAMPLES_MAX;

		double m = poly.getM();
		double x0 = cropRectangle_.x / m;
		double y0 = cropRectangle_.y / m;
		double w = cropRectangle_.width / m;
		double h = cropRectangle_.height / m;
		std::vector<uint16_t> res;

		assert(xSizes_.size() * 2 + 1 == k);
		assert(ySizes_.size() * 2 + 1 == k);

		res.reserve(k * k);

		std::vector<double> xPos(sizesListToPositions(xSizes_));
		std::vector<double> yPos(sizesListToPositions(ySizes_));

		for (int y = 0; y < k; y++) {
			for (int x = 0; x < k; x++) {
				double xp = x0 + xPos[x] * w;
				double yp = y0 + yPos[y] * h;
				/*
				 * The hardware uses 2.10 fixed point format and
				 * limits the legal values to [1..3.999]. Scale
				 * and clamp the sampled value accordingly.
				 */
				int v = static_cast<int>(
					poly.sampleAtNormalizedPixelPos(xp, yp) *
					1024);
				v = std::min(std::max(v, 1024), 4095);
				res.push_back(v);
			}
		}
		return res;
	}

	Size sensorSize_;
	Rectangle cropRectangle_;
	const std::vector<double> &xSizes_;
	const std::vector<double> &ySizes_;
};

class LscTableLoader
{
public:
	int parseLscData(const YamlObject &yamlSets,
			 std::map<unsigned int, LensShadingCorrection::Components> &lscData)
	{
		const auto &sets = yamlSets.asList();

		for (const auto &yamlSet : sets) {
			uint32_t ct = yamlSet["ct"].get<uint32_t>(0);

			if (lscData.count(ct)) {
				LOG(RkISP1Lsc, Error)
					<< "Multiple sets found for color temperature "
					<< ct;
				return -EINVAL;
			}

			LensShadingCorrection::Components &set = lscData[ct];

			set.ct = ct;
			set.r = parseTable(yamlSet, "r");
			set.gr = parseTable(yamlSet, "gr");
			set.gb = parseTable(yamlSet, "gb");
			set.b = parseTable(yamlSet, "b");

			if (set.r.empty() || set.gr.empty() ||
			    set.gb.empty() || set.b.empty()) {
				LOG(RkISP1Lsc, Error)
					<< "Set for color temperature " << ct
					<< " is missing tables";
				return -EINVAL;
			}
		}

		if (lscData.empty()) {
			LOG(RkISP1Lsc, Error) << "Failed to load any sets";
			return -EINVAL;
		}

		return 0;
	}

private:
	std::vector<uint16_t> parseTable(const YamlObject &tuningData,
					 const char *prop)
	{
		static constexpr unsigned int kLscNumSamples =
			RKISP1_CIF_ISP_LSC_SAMPLES_MAX * RKISP1_CIF_ISP_LSC_SAMPLES_MAX;

		std::vector<uint16_t> table =
			tuningData[prop].getList<uint16_t>().value_or(std::vector<uint16_t>{});
		if (table.size() != kLscNumSamples) {
			LOG(RkISP1Lsc, Error)
				<< "Invalid '" << prop << "' values: expected "
				<< kLscNumSamples
				<< " elements, got " << table.size();
			return {};
		}

		return table;
	}
};

static std::vector<double> parseSizes(const YamlObject &tuningData,
				      const char *prop)
{
	std::vector<double> sizes =
		tuningData[prop].getList<double>().value_or(std::vector<double>{});
	if (sizes.size() != RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE) {
		LOG(RkISP1Lsc, Error)
			<< "Invalid '" << prop << "' values: expected "
			<< RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE
			<< " elements, got " << sizes.size();
		return {};
	}

	/*
	 * The sum of all elements must be 0.5 to satisfy hardware constraints.
	 * Validate it here, allowing a 1% tolerance as rounding errors may
	 * prevent an exact match (further adjustments will be performed in
	 * LensShadingCorrection::prepare()).
	 */
	double sum = std::accumulate(sizes.begin(), sizes.end(), 0.0);
	if (sum < 0.495 || sum > 0.505) {
		LOG(RkISP1Lsc, Error)
			<< "Invalid '" << prop << "' values: sum of the elements"
			<< " should be 0.5, got " << sum;
		return {};
	}

	return sizes;
}

LensShadingCorrection::LensShadingCorrection()
	: lastAppliedCt_(0), lastAppliedQuantizedCt_(0)
{
	sets_.setQuantization(kColourTemperatureChangeThreshhold);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int LensShadingCorrection::init([[maybe_unused]] IPAContext &context,
				const YamlObject &tuningData)
{
	xSize_ = parseSizes(tuningData, "x-size");
	ySize_ = parseSizes(tuningData, "y-size");

	if (xSize_.empty() || ySize_.empty())
		return -EINVAL;

	/* Get all defined sets to apply. */
	const YamlObject &yamlSets = tuningData["sets"];
	if (!yamlSets.isList()) {
		LOG(RkISP1Lsc, Error)
			<< "'sets' parameter not found in tuning file";
		return -EINVAL;
	}

	std::map<unsigned int, Components> lscData;
	int res = 0;
	std::string type = tuningData["type"].get<std::string>("table");
	if (type == "table") {
		LOG(RkISP1Lsc, Debug) << "Loading tabular LSC data.";
		auto loader = LscTableLoader();
		res = loader.parseLscData(yamlSets, lscData);
	} else if (type == "polynomial") {
		LOG(RkISP1Lsc, Debug) << "Loading polynomial LSC data.";
		auto loader = LscPolynomialLoader(context.sensorInfo.activeAreaSize,
						  context.sensorInfo.analogCrop,
						  xSize_,
						  ySize_);
		res = loader.parseLscData(yamlSets, lscData);
	} else {
		LOG(RkISP1Lsc, Error) << "Unsupported LSC data type '"
				      << type << "'";
		res = -EINVAL;
	}

	if (res)
		return res;

	sets_.setData(std::move(lscData));

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int LensShadingCorrection::configure(IPAContext &context,
				     [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	const Size &size = context.configuration.sensor.size;
	Size totalSize{};

	for (unsigned int i = 0; i < RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE; ++i) {
		xSizes_[i] = xSize_[i] * size.width;
		ySizes_[i] = ySize_[i] * size.height;

		/*
		 * To prevent unexpected behavior of the ISP, the sum of x_size_tbl and
		 * y_size_tbl items shall be equal to respectively size.width/2 and
		 * size.height/2. Enforce it by computing the last tables value to avoid
		 * rounding-induced errors.
		 */
		if (i == RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE - 1) {
			xSizes_[i] = size.width / 2 - totalSize.width;
			ySizes_[i] = size.height / 2 - totalSize.height;
		}

		totalSize.width += xSizes_[i];
		totalSize.height += ySizes_[i];

		xGrad_[i] = std::round(32768 / xSizes_[i]);
		yGrad_[i] = std::round(32768 / ySizes_[i]);
	}

	context.configuration.lsc.enabled = true;
	return 0;
}

void LensShadingCorrection::setParameters(rkisp1_cif_isp_lsc_config &config)
{
	memcpy(config.x_grad_tbl, xGrad_, sizeof(config.x_grad_tbl));
	memcpy(config.y_grad_tbl, yGrad_, sizeof(config.y_grad_tbl));
	memcpy(config.x_size_tbl, xSizes_, sizeof(config.x_size_tbl));
	memcpy(config.y_size_tbl, ySizes_, sizeof(config.y_size_tbl));
}

void LensShadingCorrection::copyTable(rkisp1_cif_isp_lsc_config &config,
				      const Components &set)
{
	std::copy(set.r.begin(), set.r.end(), &config.r_data_tbl[0][0]);
	std::copy(set.gr.begin(), set.gr.end(), &config.gr_data_tbl[0][0]);
	std::copy(set.gb.begin(), set.gb.end(), &config.gb_data_tbl[0][0]);
	std::copy(set.b.begin(), set.b.end(), &config.b_data_tbl[0][0]);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void LensShadingCorrection::prepare([[maybe_unused]] IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    IPAFrameContext &frameContext,
				    RkISP1Params *params)
{
	uint32_t ct = frameContext.awb.temperatureK;
	if (std::abs(static_cast<int>(ct) - static_cast<int>(lastAppliedCt_)) <
	    kColourTemperatureChangeThreshhold)
		return;
	unsigned int quantizedCt;
	const Components &set = sets_.getInterpolated(ct, &quantizedCt);
	if (lastAppliedQuantizedCt_ == quantizedCt)
		return;

	auto config = params->block<BlockType::Lsc>();
	config.setEnabled(true);
	setParameters(*config);
	copyTable(*config, set);

	lastAppliedCt_ = ct;
	lastAppliedQuantizedCt_ = quantizedCt;

	LOG(RkISP1Lsc, Debug)
		<< "ct is " << ct << ", quantized to "
		<< quantizedCt;
}

REGISTER_IPA_ALGORITHM(LensShadingCorrection, "LensShadingCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
