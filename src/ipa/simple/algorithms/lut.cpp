/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025, Red Hat Inc.
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

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftLut)

namespace ipa::soft::algorithms {

int Lut::init(IPAContext &context,
	      [[maybe_unused]] const YamlObject &tuningData)
{
	context.ctrlMap[&controls::Contrast] = ControlInfo(0.0f, 2.0f, 1.0f);
	return 0;
}

int Lut::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	/* Gamma value is fixed */
	context.configuration.gamma = 0.5;
	context.activeState.knobs.contrast = std::optional<double>();
	updateGammaTable(context);

	return 0;
}

void Lut::queueRequest(typename Module::Context &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] typename Module::FrameContext &frameContext,
		       const ControlList &controls)
{
	const auto &contrast = controls.get(controls::Contrast);
	if (contrast.has_value()) {
		context.activeState.knobs.contrast = contrast;
		LOG(IPASoftLut, Debug) << "Setting contrast to " << contrast.value();
	}
}

void Lut::updateGammaTable(IPAContext &context)
{
	auto &gammaTable = context.activeState.gamma.gammaTable;
	const auto blackLevel = context.activeState.blc.level;
	const unsigned int blackIndex = blackLevel * gammaTable.size() / 256;
	const auto contrast = context.activeState.knobs.contrast.value_or(1.0);

	std::fill(gammaTable.begin(), gammaTable.begin() + blackIndex, 0);
	const float divisor = gammaTable.size() - blackIndex - 1.0;
	for (unsigned int i = blackIndex; i < gammaTable.size(); i++) {
		double normalized = (i - blackIndex) / divisor;
		/* Convert 0..2 to 0..infinity; avoid actual inifinity at tan(pi/2) */
		double contrastExp = tan(std::clamp(contrast * M_PI_4, 0.0, M_PI_2 - 0.00001));
		/* Apply simple S-curve */
		if (normalized < 0.5)
			normalized = 0.5 * std::pow(normalized / 0.5, contrastExp);
		else
			normalized = 1.0 - 0.5 * std::pow((1.0 - normalized) / 0.5, contrastExp);
		gammaTable[i] = UINT8_MAX *
				std::pow(normalized, context.configuration.gamma);
	}

	context.activeState.gamma.blackLevel = blackLevel;
	context.activeState.gamma.contrast = contrast;
}

int16_t Lut::ccmValue(unsigned int i, float ccm) const
{
	return std::round(i * ccm);
}

void Lut::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	frameContext.contrast = context.activeState.knobs.contrast;

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
	} else if (context.activeState.ccm.changed || gammaUpdateNeeded) {
		Matrix<float, 3, 3> gainCcm = { { gains.r(), 0, 0,
						  0, gains.g(), 0,
						  0, 0, gains.b() } };
		auto ccm = context.activeState.ccm.ccm * gainCcm;
		auto &red = params->redCcm;
		auto &green = params->greenCcm;
		auto &blue = params->blueCcm;
		for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
			red[i].r = ccmValue(i, ccm[0][0]);
			red[i].g = ccmValue(i, ccm[1][0]);
			red[i].b = ccmValue(i, ccm[2][0]);
			green[i].r = ccmValue(i, ccm[0][1]);
			green[i].g = ccmValue(i, ccm[1][1]);
			green[i].b = ccmValue(i, ccm[2][1]);
			blue[i].r = ccmValue(i, ccm[0][2]);
			blue[i].g = ccmValue(i, ccm[1][2]);
			blue[i].b = ccmValue(i, ccm[2][2]);
			params->gammaLut[i] = gammaTable[i / div];
		}
	}
}

void Lut::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  [[maybe_unused]] const SwIspStats *stats,
		  ControlList &metadata)
{
	const auto &contrast = frameContext.contrast;
	if (contrast)
		metadata.set(controls::Contrast, contrast.value());
}

REGISTER_IPA_ALGORITHM(Lut, "Lut")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
