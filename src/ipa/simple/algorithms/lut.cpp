/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Color lookup tables construction
 */

#include "lut.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "simple/ipa_context.h"

#include "adjust.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftLut)

namespace ipa::soft::algorithms {

int Lut::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	updateGammaTable(context);

	return 0;
}

void Lut::updateGammaTable(IPAContext &context)
{
	const auto blackLevel = context.activeState.blc.level;
	const auto gamma = 1.0 / context.activeState.knobs.gamma;
	const auto contrast = context.activeState.knobs.contrast.value_or(1.0);
	/* Convert 0..2 to 0..infinity; avoid actual inifinity at tan(pi/2) */
	float contrastExp = tan(std::clamp(contrast * M_PI_4, 0.0, M_PI_2 - 0.00001));

	if (!context.gpuIspEnabled) {
		auto &gammaTable = context.activeState.gamma.gammaTable;
		const unsigned int blackIndex = blackLevel * gammaTable.size() / 256;
		const float divisor = gammaTable.size() - blackIndex - 1.0;
		for (unsigned int i = blackIndex; i < gammaTable.size(); i++) {
			double normalized = (i - blackIndex) / divisor;
			/* Apply simple S-curve */
			if (normalized < 0.5)
				normalized = 0.5 * std::pow(normalized / 0.5, contrastExp);
			else
				normalized = 1.0 - 0.5 * std::pow((1.0 - normalized) / 0.5, contrastExp);
			gammaTable[i] = UINT8_MAX * std::pow(normalized, gamma);
		}
		/*
		 * Due to CCM operations, the table lookup may reach indices below the black
		 * level. Let's set the table values below black level to the minimum
		 * non-black value to prevent problems when the minimum value is
		 * significantly non-zero (for example, when the image should be all grey).
		 */
		std::fill(gammaTable.begin(), gammaTable.begin() + blackIndex,
			  gammaTable[blackIndex]);
	}

	context.activeState.gamma.gamma = gamma;
	context.activeState.gamma.blackLevel = blackLevel;
	context.activeState.gamma.contrastExp = contrastExp;
}

int16_t Lut::matrixValue(unsigned int i, float ccm) const
{
	return std::round(i * ccm);
}

void Lut::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	/*
	 * Update the gamma table if needed. This means if black level changes
	 * and since the black level gets updated only if a lower value is
	 * observed, it's not permanently prone to minor fluctuations or
	 * rounding errors.
	 */
	const bool gammaUpdateNeeded =
		context.activeState.gamma.blackLevel != context.activeState.blc.level ||
		context.activeState.gamma.contrast != context.activeState.knobs.contrast;
	if (gammaUpdateNeeded)
		updateGammaTable(context);

	auto &gains = context.activeState.awb.gains;
	auto &gammaTable = context.activeState.gamma.gammaTable;
	const unsigned int gammaTableSize = gammaTable.size();
	const double div = static_cast<double>(DebayerParams::kRGBLookupSize) /
			   gammaTableSize;

	if (!context.ccmEnabled) {
		for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
			/* Apply gamma after gain! */
			const RGB<float> lutGains = (gains * i / div).min(gammaTableSize - 1);
			params->red[i] = gammaTable[static_cast<unsigned int>(lutGains.r())];
			params->green[i] = gammaTable[static_cast<unsigned int>(lutGains.g())];
			params->blue[i] = gammaTable[static_cast<unsigned int>(lutGains.b())];
		}
	} else if (context.activeState.matrixChanged || gammaUpdateNeeded) {
		auto &matrix = context.activeState.combinedMatrix;
		auto &red = params->redCcm;
		auto &green = params->greenCcm;
		auto &blue = params->blueCcm;
		params->ccm = matrix;
		if (!context.gpuIspEnabled) {
			for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
				red[i].r = matrixValue(i, matrix[0][0]);
				red[i].g = matrixValue(i, matrix[1][0]);
				red[i].b = matrixValue(i, matrix[2][0]);
				green[i].r = matrixValue(i, matrix[0][1]);
				green[i].g = matrixValue(i, matrix[1][1]);
				green[i].b = matrixValue(i, matrix[2][1]);
				blue[i].r = matrixValue(i, matrix[0][2]);
				blue[i].g = matrixValue(i, matrix[1][2]);
				blue[i].b = matrixValue(i, matrix[2][2]);
				params->gammaLut[i] = gammaTable[i / div];
			}
		}
		context.activeState.matrixChanged = false;
	}

	params->gamma = context.activeState.gamma.gamma;
	params->contrastExp = context.activeState.gamma.contrastExp;
}

REGISTER_IPA_ALGORITHM(Lut, "Lut")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
